#ifndef GNSS_GUARD_H
#define GNSS_GUARD_H

/* GNSS interference guard - decides how much of the GPS solution the
 * navigation filter is still allowed to believe.
 *
 * WHY THIS EXISTS. A receiver under jamming goes quiet, and quiet is easy:
 * the fix drops, hAcc explodes, and the existing gates already refuse it.
 * A receiver under SPOOFING does the opposite - it reports a confident,
 * well-conditioned, entirely fictional position, with a small hAcc and a
 * healthy satellite count. Every gate this project already has is built
 * around "is the number plausible", and a spoofed number is plausible by
 * construction. That is the hole this file fills.
 *
 * It is the same shape as the IMU plausibility gate: a detector that can
 * refuse a sensor, plus an explicit decision about what the loop does on
 * refusal. The two share a rule learned the expensive way -
 * check the sensor's LIVENESS and its CONSISTENCY WITH SOMETHING ELSE,
 * never just whether its value looks reasonable on its own.
 *
 * DELIBERATELY FREESTANDING. stdint and float only - no libc, no libm, no
 * ESP-IDF, no FreeRTOS. tools/gnss_guard_test.py compiles this exact file
 * on the host, the same trick ubx.c and vert_filter.c use. Anything that
 * needs a clock, a mutex or a log line belongs in tbsM10.c.
 */

#include <stdint.h>

/* --- What the guard concludes ------------------------------------------
 *
 * Gated SEPARATELY for position and velocity, because this project has
 * measured that they are not equally trustworthy: on this receiver,
 * velocity scatter is 0.07-0.11 m/s while position scatter is 1.07-3.21 m,
 * and velocity survives the multipath that ruins position because the
 * Doppler solution is not fooled the way the code solution is.
 * Throwing both away over evidence that only condemns one is throwing
 * away the better sensor. */
/* The module's own state numbering, mirrored here rather than pulled in
 * from ubx.h on purpose: this component decides policy and must not
 * depend on the transport that happens to deliver the numbers. The values
 * match u-blox's MON-RF and SEC-SIG encodings, and tools/gnss_guard_test.py
 * asserts they still agree with ubx.h so the two cannot drift apart.
 *
 * Guarded because tbsM10.c includes both headers, and an unguarded second
 * definition of the same name is a warning nobody needs to read. */
#ifndef UBX_JAM_UNKNOWN
#define UBX_JAM_UNKNOWN   0
#define UBX_JAM_OK        1
#define UBX_JAM_WARNING   2
#define UBX_JAM_CRITICAL  3
#endif
#ifndef UBX_SPF_UNKNOWN
#define UBX_SPF_UNKNOWN   0
#define UBX_SPF_NONE      1
#define UBX_SPF_INDICATED 2
#define UBX_SPF_MULTIPLE  3
#endif

typedef enum {
    GG_CLEAN   = 0,  /* use it normally */
    GG_SUSPECT = 1,  /* keep using it, but inflate R - trust it less */
    GG_DENIED  = 2   /* drop it from the filter entirely */
} gg_level_t;

/* Why the guard reached its verdict. A bitmask rather than one cause,
 * because the interesting cases are the ones where two independent
 * indicators agree - and because a log line that says only "DENIED" is
 * useless at 2 a.m. */
#define GG_R_MOD_JAM_WARN   0x0001u /* module: jammingState == 2 */
#define GG_R_MOD_JAM_CRIT   0x0002u /* module: jammingState == 3 */
#define GG_R_MOD_SPOOF      0x0004u /* module: spoofingState == 2 */
#define GG_R_MOD_SPOOF_MULT 0x0008u /* module: spoofingState == 3 */
#define GG_R_CNO_UNIFORM    0x0010u /* C/N0 spread collapsed - one source */
#define GG_R_CNO_COLLAPSE   0x0020u /* C/N0 mean fell off a cliff */
#define GG_R_AGC_SHIFT      0x0040u /* AGC moved far from its quiet baseline */
#define GG_R_NOISE_HIGH     0x0080u /* noisePerMS far above baseline */
#define GG_R_VEL_DIVERGE    0x0100u /* GPS velocity disagrees with inertial */
#define GG_R_SAT_JUMP       0x0200u /* used-satellite count jumped implausibly */
#define GG_R_STALE          0x0400u /* no MON-RF/SEC-SIG for too long */

typedef struct {
    gg_level_t pos;
    gg_level_t vel;
    uint16_t reasons;        /* bitmask above, for the worst live verdict */
    uint16_t reasons_latched;/* everything seen since the last clear */

    /* Reported for telemetry and the blackbox, not used for deciding. */
    float cno_mean;
    float cno_sd;
    uint16_t n_used;
    uint8_t  jamming_state;  /* best available, module's numbering */
    uint8_t  spoofing_state;
    uint16_t agc_cnt;
    uint16_t agc_baseline;
    /* Reported alongside the AGC pair for the same reason: the two deltas
     * in gg_cfg_t are marked MEASURE, and they cannot be measured from a
     * log that never prints the quantity they threshold. */
    uint16_t noise_per_ms;
    uint16_t noise_baseline;

    uint32_t n_escalations;  /* CLEAN -> not-CLEAN transitions, ever */
    uint32_t denied_since_ms;/* 0 when not denied */
} gg_status_t;

/* --- Tuning -------------------------------------------------------------
 *
 * Every default here is a STARTING POINT, not a measured value. The three
 * that must be re-derived from a real bench log before this is allowed to
 * gate anything in flight are marked MEASURE. Shipping a guessed threshold
 * as though it were measured is how a gate either never fires or fires on
 * a clean sky. */
typedef struct {
    /* C/N0 spread, in dB-Hz x10, below which the constellation looks like
     * it came from one transmitter. MEASURE: log a clean sky for a few
     * minutes, take the minimum sd seen, and sit well under it. 4.5 dB-Hz
     * is a literature starting point, not this antenna's number. */
    uint16_t cno_sd_min_x10;
    /* Minimum used-signal count for the spread test to mean anything. The
     * sd of three satellites is noise. */
    uint16_t cno_min_used;
    /* Mean C/N0 in dB-Hz below which the band is being buried. */
    uint8_t cno_mean_floor;

    /* AGC counts away from baseline that count as a shift. MEASURE. */
    uint16_t agc_delta;
    /* noisePerMS above baseline that counts as broadband noise. MEASURE. */
    uint16_t noise_delta;

    /* GPS-vs-inertial speed disagreement, mm/s, sustained for
     * vel_diverge_ms, that condemns the fix. Set from the measured
     * velocity scatter (0.07-0.11 m/s -> 110 mm/s) with margin. */
    uint32_t vel_diverge_mms;
    uint32_t vel_diverge_ms;

    /* Used-satellite count jumping by more than this between consecutive
     * solutions - a capture, not a sky. */
    uint8_t sat_jump;
    /* How long that jump stays standing once seen.
     *
     * Unlike everything else in this struct, a satellite-count jump is an
     * EVENT, not a condition: it is true for exactly one solution and
     * false for the next. NAV-PVT arrives at 10 Hz, so without a hold the
     * ninth solution of the same second clears a jump the first one found,
     * and gg_update() - which may only run once between them - never sees
     * it at all. The hold is what makes an instant observable. */
    uint32_t sat_jump_hold_ms;

    /* No MON-RF or SEC-SIG for this long means the guard is blind, and a
     * blind guard must say so rather than keep reporting CLEAN. */
    uint32_t stale_ms;

    /* Continuous clean time before a latched DENIED is released. Stops a
     * flickering interferer making the filter flap between sources. */
    uint32_t clear_ms;

    /* How long a first escalation is held before it is allowed to reach
     * DENIED, so a single bad frame cannot drop the fix. */
    uint32_t confirm_ms;
} gg_cfg_t;

/* Fills cfg with the documented defaults above. */
void gg_default_cfg(gg_cfg_t *cfg);

/* --- The guard ---------------------------------------------------------- */
typedef struct {
    gg_cfg_t cfg;
    gg_status_t st;

    /* Quiet-sky baselines, learned while the guard is CLEAN. Learned
     * rather than hard-coded because AGC and noise counts are properties
     * of this antenna, this cable and this airframe's own switching
     * noise, and a number copied from a datasheet describes none of them. */
    uint32_t agc_sum;
    uint32_t noise_sum;
    uint32_t base_n;
    uint8_t  base_ready;

    /* Evidence currently standing. Each feed_*() owns its own bits and
     * both sets AND clears them, so a condition that goes away stops
     * counting without anyone having to sweep the word. gg_update() only
     * reads it. A source that stops arriving altogether leaves its bits
     * frozen, which is what the staleness timer is for. */
    uint16_t live_reasons;

    uint8_t  last_mon_jam;
    uint8_t  last_sec_jam;
    uint8_t  last_sec_spoof;
    uint8_t  last_have_sec;

    uint32_t last_rf_ms;      /* last MON-RF or SEC-SIG */
    uint32_t last_sig_ms;     /* last NAV-SIG */
    uint32_t first_bad_ms;    /* when the current escalation started */
    uint32_t last_clean_ms;   /* when evidence last went quiet */
    uint32_t vel_bad_since_ms;/* when GPS/inertial divergence started */
    uint32_t sat_jump_ms;     /* when the last count jump was seen, 0 = never */
    uint8_t  prev_n_used;
    uint8_t  have_prev_used;
    uint8_t  latched;         /* a DENIED is being held */
} gg_t;

void gg_init(gg_t *g, const gg_cfg_t *cfg);

/* Feed the module's RF report. `have_sec` says whether sec_* came from a
 * real SEC-SIG; when it did not, the MON-RF jamming state is used alone
 * and the spoofing half of the guard runs on our own evidence only. */
void gg_feed_rf(gg_t *g, uint32_t now_ms,
                uint8_t mon_jam_state, uint16_t agc_cnt, uint16_t noise_per_ms,
                uint8_t jam_ind,
                uint8_t have_sec, uint8_t sec_jam_state, uint8_t sec_spoof_state);

/* Feed one NAV-SIG C/N0 snapshot. sum/sumsq are over used signals. */
void gg_feed_cno(gg_t *g, uint32_t now_ms,
                 uint16_t n_used, uint32_t sum_cno, uint32_t sumsq_cno);

/* Feed the cross-check. gps_speed_mms is the NAV-PVT horizontal speed;
 * inertial_speed_mms is the filter's own, and have_inertial is 0 when the
 * filter has nothing trustworthy to compare against - in which case this
 * check is skipped rather than assumed to pass. */
void gg_feed_solution(gg_t *g, uint32_t now_ms,
                      uint8_t n_used_sats,
                      uint32_t gps_speed_mms,
                      uint8_t have_inertial, uint32_t inertial_speed_mms);

/* Recompute the verdict. Call at a steady rate; it is what applies the
 * staleness, confirmation and latch timers, so a guard that stops being
 * called stops timing out rather than silently holding its last answer. */
void gg_update(gg_t *g, uint32_t now_ms);

/* Current verdict. */
const gg_status_t *gg_status(const gg_t *g);

/* Multiplier the filter should apply to the GPS measurement-noise R while
 * SUSPECT. 1.0 when clean. Returns a large number when denied, but a
 * denied channel should be dropped, not merely distrusted. */
float gg_r_scale(const gg_t *g, gg_level_t level);

/* Release a latched DENIED by hand, for the bench. Does not clear the
 * evidence that caused it - reasons_latched survives so a log still shows
 * what happened. */
void gg_force_clear(gg_t *g, uint32_t now_ms);

/* Square root, Newton, freestanding. Exposed only so the host test can
 * check it against Python's rather than trusting it. */
float gg_sqrtf(float x);

#endif /* GNSS_GUARD_H */
