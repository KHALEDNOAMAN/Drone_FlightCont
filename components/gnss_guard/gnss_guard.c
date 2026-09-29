/* GNSS interference guard - see gnss_guard.h for why this exists and why
 * it has no dependencies. Host-tested by tools/gnss_guard_test.py. */

#include "gnss_guard.h"

/* Evidence families. The split matters: a jammer degrades the signal, and
 * this project has MEASURED that velocity survives degradation that ruins
 * position. A spoofer does not degrade anything - it substitutes a whole
 * solution, Doppler included - so spoofing evidence condemns both halves
 * and jamming evidence condemns position first. */
#define GG_SPOOF_FAMILY (GG_R_MOD_SPOOF | GG_R_MOD_SPOOF_MULT | \
                         GG_R_CNO_UNIFORM | GG_R_VEL_DIVERGE | GG_R_SAT_JUMP)

#define GG_JAM_FAMILY   (GG_R_MOD_JAM_WARN | GG_R_MOD_JAM_CRIT | \
                         GG_R_CNO_COLLAPSE | GG_R_AGC_SHIFT | GG_R_NOISE_HIGH)

/* The guard's OWN heuristics, as opposed to the module's verdicts. Each of
 * these alone can only reach SUSPECT. Two agreeing can reach DENIED. The
 * asymmetry is deliberate: every one of them has a benign explanation on
 * its own - a car roof overhead collapses C/N0, a hard turn diverges
 * velocity briefly - and losing GPS mid-flight to one noisy heuristic is a
 * worse outcome than carrying a distrusted fix for another second. */
#define GG_OWN_HEURISTICS (GG_R_CNO_UNIFORM | GG_R_CNO_COLLAPSE | \
                           GG_R_AGC_SHIFT | GG_R_NOISE_HIGH | \
                           GG_R_VEL_DIVERGE | GG_R_SAT_JUMP)

/* Samples of quiet sky averaged before the AGC/noise baselines are used.
 * At the 1 Hz MON-RF rate this is about half a minute. */
#define GG_BASE_MIN_N 20u
/* Cap, so the baseline keeps tracking slow drift (temperature, a warming
 * LNA) instead of freezing on the first thirty seconds after boot. */
#define GG_BASE_MAX_N 300u

float gg_sqrtf(float x)
{
    if (!(x > 0.0f)) return 0.0f;    /* also catches NaN */

    /* Seed by halving the exponent, then Newton.
     *
     * The seed matters more than the iteration count. Newton for sqrt only
     * doubles its correct digits once it is CLOSE; from a crude seed like
     * x/2 it spends its first several steps merely halving the error, and
     * at x = 1234.5 six iterations still land on 35.18 against a true
     * 35.135 - an error 50x wider than this function's own tolerance.
     * Shifting the exponent right is accurate to a few percent for ANY
     * magnitude, which puts Newton straight into its quadratic phase.
     *
     * The bias constant is 0x3F800000 >> 1: it restores the exponent bias
     * that the shift halves away. Exact for powers of four by inspection
     * (1.0 -> 1.0, 4.0 -> 2.0). */
    union { float f; uint32_t i; } u;
    u.f = x;
    u.i = (u.i >> 1) + 0x1FC00000u;
    float r = u.f;

    /* Four steps from a 3%-accurate seed reaches float precision with room
     * to spare; the host test checks this against Python's sqrt. */
    for (int i = 0; i < 4; i++) r = 0.5f * (r + x / r);
    return r;
}

static uint32_t popcount16(uint16_t v)
{
    uint32_t n = 0;
    while (v) { n += (uint32_t)(v & 1u); v = (uint16_t)(v >> 1); }
    return n;
}

void gg_default_cfg(gg_cfg_t *cfg)
{
    cfg->cno_sd_min_x10  = 45;    /* 4.5 dB-Hz  - MEASURE */
    cfg->cno_min_used    = 6;
    cfg->cno_mean_floor  = 20;    /* dB-Hz */
    cfg->agc_delta       = 600;   /* counts     - MEASURE */
    cfg->noise_delta     = 400;   /*            - MEASURE */
    cfg->vel_diverge_mms = 3000;  /* 3 m/s, ~27x the measured 0.11 m/s sd */
    cfg->vel_diverge_ms  = 2000;
    cfg->sat_jump        = 8;
    cfg->sat_jump_hold_ms = 3000;
    cfg->stale_ms        = 5000;
    cfg->clear_ms        = 10000;
    cfg->confirm_ms      = 1000;
}

void gg_init(gg_t *g, const gg_cfg_t *cfg)
{
    if (cfg) g->cfg = *cfg;
    else gg_default_cfg(&g->cfg);

    g->st.pos = GG_CLEAN;
    g->st.vel = GG_CLEAN;
    g->st.reasons = 0;
    g->st.reasons_latched = 0;
    g->st.cno_mean = 0.0f;
    g->st.cno_sd = 0.0f;
    g->st.n_used = 0;
    g->st.jamming_state = 0;
    g->st.spoofing_state = 0;
    g->st.agc_cnt = 0;
    g->st.agc_baseline = 0;
    g->st.noise_per_ms = 0;
    g->st.noise_baseline = 0;
    g->st.n_escalations = 0;
    g->st.denied_since_ms = 0;

    g->agc_sum = 0;
    g->noise_sum = 0;
    g->base_n = 0;
    g->base_ready = 0;

    g->live_reasons = 0;
    g->last_mon_jam = 0;
    g->last_sec_jam = 0;
    g->last_sec_spoof = 0;
    g->last_have_sec = 0;

    g->last_rf_ms = 0;
    g->last_sig_ms = 0;
    g->first_bad_ms = 0;
    g->last_clean_ms = 0;
    g->vel_bad_since_ms = 0;
    g->sat_jump_ms = 0;
    g->prev_n_used = 0;
    g->have_prev_used = 0;
    g->latched = 0;
}

static void set_bit(gg_t *g, uint16_t bit, int on)
{
    if (on) g->live_reasons |= bit;
    else    g->live_reasons = (uint16_t)(g->live_reasons & ~bit);
}

void gg_feed_rf(gg_t *g, uint32_t now_ms,
                uint8_t mon_jam_state, uint16_t agc_cnt, uint16_t noise_per_ms,
                uint8_t jam_ind,
                uint8_t have_sec, uint8_t sec_jam_state, uint8_t sec_spoof_state)
{
    (void)jam_ind;  /* reported by the module, not yet a threshold here */

    g->last_rf_ms = now_ms;
    g->last_mon_jam = mon_jam_state;
    g->last_have_sec = have_sec;
    g->last_sec_jam = have_sec ? sec_jam_state : 0;
    g->last_sec_spoof = have_sec ? sec_spoof_state : 0;

    /* Worst of the two jamming opinions. SEC-SIG and MON-RF are fed by the
     * same interference monitor, but only one of them may be enabled, and
     * a state of 0 means "unknown", never "fine". */
    uint8_t jam = mon_jam_state;
    if (have_sec && sec_jam_state > jam) jam = sec_jam_state;
    g->st.jamming_state = jam;
    g->st.spoofing_state = g->last_sec_spoof;
    g->st.agc_cnt = agc_cnt;
    g->st.noise_per_ms = noise_per_ms;

    set_bit(g, GG_R_MOD_JAM_WARN, jam == UBX_JAM_WARNING);
    set_bit(g, GG_R_MOD_JAM_CRIT, jam == UBX_JAM_CRITICAL);
    set_bit(g, GG_R_MOD_SPOOF,      g->last_sec_spoof == UBX_SPF_INDICATED);
    set_bit(g, GG_R_MOD_SPOOF_MULT, g->last_sec_spoof == UBX_SPF_MULTIPLE);

    /* Baseline learning, and the reason it is gated on CLEAN: a baseline
     * averaged while the band is already being jammed learns the jammer
     * and then reports it as normal. The guard would calibrate itself
     * blind exactly when it is needed. */
    if (g->st.pos == GG_CLEAN && g->live_reasons == 0) {
        if (g->base_n >= GG_BASE_MAX_N) {
            /* Slide rather than grow: drop one average-sized sample. */
            g->agc_sum   -= g->agc_sum / g->base_n;
            g->noise_sum -= g->noise_sum / g->base_n;
            g->base_n--;
        }
        g->agc_sum   += agc_cnt;
        g->noise_sum += noise_per_ms;
        g->base_n++;
        if (g->base_n >= GG_BASE_MIN_N) g->base_ready = 1;
    }

    if (g->base_ready && g->base_n > 0) {
        uint32_t agc_base   = g->agc_sum / g->base_n;
        uint32_t noise_base = g->noise_sum / g->base_n;
        g->st.agc_baseline = (uint16_t)agc_base;
        g->st.noise_baseline = (uint16_t)noise_base;

        /* AGC is tested in ONE direction. A strong in-band interferer
         * makes the front end back its gain off, so the count FALLS. A
         * rising count is a quieter band or a colder LNA, which is not an
         * attack, and treating it as one would trip the guard every time
         * the aircraft came out of a building's shadow. */
        int32_t agc_drop = (int32_t)agc_base - (int32_t)agc_cnt;
        set_bit(g, GG_R_AGC_SHIFT, agc_drop > (int32_t)g->cfg.agc_delta);

        int32_t noise_rise = (int32_t)noise_per_ms - (int32_t)noise_base;
        set_bit(g, GG_R_NOISE_HIGH, noise_rise > (int32_t)g->cfg.noise_delta);
    }
}

void gg_feed_cno(gg_t *g, uint32_t now_ms,
                 uint16_t n_used, uint32_t sum_cno, uint32_t sumsq_cno)
{
    g->last_sig_ms = now_ms;
    g->st.n_used = n_used;

    if (n_used == 0) {
        g->st.cno_mean = 0.0f;
        g->st.cno_sd = 0.0f;
        set_bit(g, GG_R_CNO_UNIFORM, 0);
        set_bit(g, GG_R_CNO_COLLAPSE, 0);
        return;
    }

    float mean = (float)sum_cno / (float)n_used;
    g->st.cno_mean = mean;

    /* Variance without a divide or a sqrt in the DECISION path:
     *     var  = (n*sumsq - sum^2) / n^2
     *     var < T^2   <=>   (n*sumsq - sum^2)*100 < (T*10)^2 * n^2
     * All integer, so the threshold comparison cannot drift with float
     * rounding. The float sd below is for the log line only. */
    int64_t n   = (int64_t)n_used;
    int64_t num = n * (int64_t)sumsq_cno - (int64_t)sum_cno * (int64_t)sum_cno;
    if (num < 0) num = 0;   /* only reachable on inconsistent inputs */

    g->st.cno_sd = gg_sqrtf((float)num / ((float)n * (float)n));

    int64_t lhs = num * 100;
    int64_t t   = (int64_t)g->cfg.cno_sd_min_x10;
    int64_t rhs = t * t * n * n;

    /* The spread test needs enough satellites to mean anything - the sd of
     * three signals is noise, and firing on it would condemn every fix
     * taken in a narrow street. */
    set_bit(g, GG_R_CNO_UNIFORM,
            n_used >= g->cfg.cno_min_used && lhs < rhs);

    set_bit(g, GG_R_CNO_COLLAPSE, mean < (float)g->cfg.cno_mean_floor);
}

void gg_feed_solution(gg_t *g, uint32_t now_ms,
                      uint8_t n_used_sats,
                      uint32_t gps_speed_mms,
                      uint8_t have_inertial, uint32_t inertial_speed_mms)
{
    /* A capture typically arrives as a step in the used-satellite count:
     * the receiver drops the real constellation and acquires the spoofed
     * one, or the spoofer's signals appear all at once.
     *
     * This is an EVENT and is therefore TIMESTAMPED, not set-and-cleared
     * like the conditions around it. NAV-PVT arrives at 10 Hz: the first
     * solution after a capture sees the step, and the nine behind it see
     * a steady count. Clearing on those nine would erase the finding
     * before gg_update() ever ran - which is exactly what happened, and
     * what tools/gnss_scenario.py caught by feeding ten solutions per
     * second the way the module really does. The bit is released by the
     * hold timer in gg_update(), not here. */
    if (g->have_prev_used) {
        int32_t d = (int32_t)n_used_sats - (int32_t)g->prev_n_used;
        if (d < 0) d = -d;
        if (d > (int32_t)g->cfg.sat_jump) {
            g->sat_jump_ms = now_ms ? now_ms : 1;  /* 0 means "never" */
            set_bit(g, GG_R_SAT_JUMP, 1);
        }
    }
    g->prev_n_used = n_used_sats;
    g->have_prev_used = 1;

    /* The cross-check, and the only test here that a spoofer cannot
     * satisfy by being internally consistent: it has no way to know what
     * the airframe's own accelerometers are reporting. Skipped outright
     * when the filter has nothing to offer, because comparing against a
     * number the filter does not trust would invent disagreement. */
    if (!have_inertial) {
        g->vel_bad_since_ms = 0;
        set_bit(g, GG_R_VEL_DIVERGE, 0);
        return;
    }

    uint32_t diff = gps_speed_mms > inertial_speed_mms
                  ? gps_speed_mms - inertial_speed_mms
                  : inertial_speed_mms - gps_speed_mms;

    if (diff > g->cfg.vel_diverge_mms) {
        if (g->vel_bad_since_ms == 0) g->vel_bad_since_ms = now_ms;
        /* Must PERSIST. A single disagreeing sample is a GPS outlier or a
         * hard manoeuvre, and both are common on a clean sky. */
        set_bit(g, GG_R_VEL_DIVERGE,
                (now_ms - g->vel_bad_since_ms) >= g->cfg.vel_diverge_ms);
    } else {
        g->vel_bad_since_ms = 0;
        set_bit(g, GG_R_VEL_DIVERGE, 0);
    }
}

static gg_level_t worse(gg_level_t a, gg_level_t b)
{
    return a > b ? a : b;
}

void gg_update(gg_t *g, uint32_t now_ms)
{
    /* Release the satellite-jump event once its hold has run out. Done
     * here rather than in the feed because gg_update() is the only
     * function guaranteed to be called on a clock rather than on a
     * message that may never arrive again. */
    if (g->sat_jump_ms != 0 &&
        (now_ms - g->sat_jump_ms) >= g->cfg.sat_jump_hold_ms) {
        g->sat_jump_ms = 0;
        set_bit(g, GG_R_SAT_JUMP, 0);
    }

    uint16_t r = g->live_reasons;

    /* A guard that has stopped hearing from the module is blind, and a
     * blind guard reports SUSPECT rather than CLEAN. Reporting CLEAN here
     * is the mistake in another costume: a dead sensor whose
     * last value still looked fine. */
    int stale = (g->last_rf_ms == 0) ||
                ((now_ms - g->last_rf_ms) > g->cfg.stale_ms);
    if (stale) r |= GG_R_STALE;
    else       r = (uint16_t)(r & ~GG_R_STALE);

    g->st.reasons = r;
    g->st.reasons_latched |= r;

    /* --- evidence -> level ------------------------------------------- */
    gg_level_t pos = GG_CLEAN;
    gg_level_t vel = GG_CLEAN;

    /* The module's own verdicts. These are the chip's dedicated detector,
     * not a heuristic bolted on at this level, so they are allowed to
     * reach DENIED alone. */
    if (r & GG_R_MOD_JAM_CRIT) { pos = worse(pos, GG_DENIED);  vel = worse(vel, GG_SUSPECT); }
    if (r & GG_R_MOD_JAM_WARN) { pos = worse(pos, GG_SUSPECT); }
    if (r & GG_R_MOD_SPOOF_MULT) { pos = worse(pos, GG_DENIED); vel = worse(vel, GG_DENIED); }
    if (r & GG_R_MOD_SPOOF)      { pos = worse(pos, GG_SUSPECT); vel = worse(vel, GG_SUSPECT); }
    if (r & GG_R_STALE)          { pos = worse(pos, GG_SUSPECT); vel = worse(vel, GG_SUSPECT); }

    /* Our own heuristics: one is a suspicion, two agreeing is a verdict. */
    uint16_t own = (uint16_t)(r & GG_OWN_HEURISTICS);
    uint32_t n_own = popcount16(own);
    if (n_own >= 1) {
        if (own & GG_SPOOF_FAMILY) { pos = worse(pos, GG_SUSPECT); vel = worse(vel, GG_SUSPECT); }
        if (own & GG_JAM_FAMILY)   { pos = worse(pos, GG_SUSPECT); }
    }
    if (n_own >= 2) {
        if (own & GG_SPOOF_FAMILY) { pos = worse(pos, GG_DENIED); vel = worse(vel, GG_DENIED); }
        if (own & GG_JAM_FAMILY)   { pos = worse(pos, GG_DENIED); vel = worse(vel, GG_SUSPECT); }
    }

    /* --- confirmation ------------------------------------------------- */
    if (r == 0) {
        g->first_bad_ms = 0;
        g->last_clean_ms = g->last_clean_ms ? g->last_clean_ms : now_ms;
    } else {
        if (g->first_bad_ms == 0) g->first_bad_ms = now_ms;
        g->last_clean_ms = 0;
    }

    /* A fresh escalation is held at SUSPECT until it has stood for
     * confirm_ms. Nothing that lasts one frame is allowed to drop the fix,
     * which is the difference between a guard and a tripwire. */
    if (g->first_bad_ms != 0 &&
        (now_ms - g->first_bad_ms) < g->cfg.confirm_ms) {
        if (pos == GG_DENIED) pos = GG_SUSPECT;
        if (vel == GG_DENIED) vel = GG_SUSPECT;
    }

    /* --- latch --------------------------------------------------------- */
    if (pos == GG_DENIED || vel == GG_DENIED) {
        if (!g->latched) {
            g->latched = 1;
            g->st.denied_since_ms = now_ms;
        }
    } else if (g->latched) {
        /* Hold the denial until the sky has been quiet for clear_ms
         * CONTINUOUSLY. An interferer that pulses would otherwise walk the
         * filter in and out of GPS every cycle, which is worse than either
         * state - the filter would never settle on a reference. */
        if (r == 0 && g->last_clean_ms != 0 &&
            (now_ms - g->last_clean_ms) >= g->cfg.clear_ms) {
            g->latched = 0;
            g->st.denied_since_ms = 0;
        } else {
            pos = worse(pos, GG_DENIED);
            vel = worse(vel, g->st.vel == GG_DENIED ? GG_DENIED : GG_SUSPECT);
        }
    }

    if (g->st.pos == GG_CLEAN && pos != GG_CLEAN) g->st.n_escalations++;

    g->st.pos = pos;
    g->st.vel = vel;
}

const gg_status_t *gg_status(const gg_t *g)
{
    return &g->st;
}

float gg_r_scale(const gg_t *g, gg_level_t level)
{
    (void)g;
    switch (level) {
    case GG_CLEAN:   return 1.0f;
    /* 4x the standard deviation, so 16x the variance. Enough that a
     * suspect fix cannot drag the filter, small enough that it still
     * anchors slow drift. */
    case GG_SUSPECT: return 16.0f;
    default:         return 1.0e6f;   /* drop it instead of using this */
    }
}

void gg_force_clear(gg_t *g, uint32_t now_ms)
{
    g->latched = 0;
    g->st.denied_since_ms = 0;
    g->st.pos = GG_CLEAN;
    g->st.vel = GG_CLEAN;
    g->first_bad_ms = 0;
    g->last_clean_ms = now_ms;
    /* live_reasons is NOT cleared: if the interferer is still there the
     * next update puts the guard straight back where it was. A manual
     * clear overrides the latch, not reality. */
}
