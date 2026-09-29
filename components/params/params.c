#include "params.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_rom_crc.h"
#include <string.h>
#include <math.h>       /* isfinite - a NaN in NVS cannot be clamped */

static const char *TAG = "PARAMS";

/* PARAMS_VERSION now lives in params.h - blackbox.c needs it too. */
#define NVS_NAMESPACE   "fc"
#define NVS_KEY         "params1"

params_t g_params;

/* Defaults = the values that flew the bench up to now (fc_tasks.c
 * hardcoded gains, config.h limits, the measured idle and
 * sweep-derived filters). Changing a default here does nothing for a
 * board with a saved blob: SET + SAVE it, or PARAMS,DEFAULT + SAVE.
 * Bumping PARAMS_VERSION no longer resets anything - older blobs are
 * migrated by prefix (params_load_blob). */
static const params_t k_defaults = {
    .rate_kp = 1.2f,  .rate_ki = 0.05f, .rate_kd = 0.03f,
    .yaw_kp  = 3.0f,  .yaw_ki  = 0.10f, .yaw_kd  = 0.00f,
    .angle_kp = 4.0f,
    .angle_max_deg = 30.0f,
    .yaw_rate_max  = 200.0f,
    .rc_deadband   = 15.0f,
    .min_idle_us   = 1050.0f,
    .gyro_lpf_hz   = 60.0f,
    .notch_hz      = 78.0f,
    .notch_q       = 5.0f,
    /* 1.0 = no trim. the author's airframe flies m1_scale 0.88 (SET+SAVE) -
     * an airframe-specific ESC quirk, so it is not baked as a default. */
    .m1_scale = 1.0f, .m2_scale = 1.0f, .m3_scale = 1.0f, .m4_scale = 1.0f,
    /* 0 = trust the mount. Measured per airframe by LEVELCAL, never a
     * default: a level reference copied from another build is worse than
     * none, because it looks calibrated. */
    .level_roll_deg = 0.0f, .level_pitch_deg = 0.0f,
    /* P7 alt-hold. hover_thr is MEASURED (flight4 s19-s22); the gains
     * are derived from it and checked in tools/alt_hold_sim.py.
     * alt_acc_sigma is MEASURED too (was 0.588, an idle
     * bench figure doubled as a guess; real flight is 1.77-2.83). It
     * feeds vert_filter's process noise, and setting it too low is what
     * let a vibrating accelerometer drive the estimate through the floor
     * and the controller answer with full thrust. Do not lower it
     * without a hover log that justifies the number.
     * Full reasoning in components/common/alt_hold.c. */
    .alt_hover_thr = 420.0f,
    .alt_vref      = 16.0f,
    .alt_vz_max    = 1.0f,
    .alt_kp_z      = 0.8f,
    .alt_kp_vz     = 60.0f,
    .alt_ki_vz     = 30.0f,
    .alt_i_max     = 200.0f,
    .alt_thr_db    = 80.0f,
    .alt_hold_auto = 0.0f,
    .vel_hold_auto = 0.0f,
    .vel_kp        = 4.0f,
    .vel_ki        = 1.0f,
    .vel_i_max     = 5.0f,
    .vel_tilt_max  = 12.0f,
    .vel_max       = 3.0f,
    .alt_acc_sigma = 2.0f,
    /* MEASURED on this airframe's TOF200C,, not copied from a
     * preset: these four numbers together are the configuration that
     * reached 1700 mm on a bare ESP32 while ST's stock settings (0.25,
     * 14/10, ~33 ms) capped the SAME module at 24 cm. Stock is the
     * setting that has been shown not to work here, so it is not a safe
     * default just because it is the vendor's. Reasoning in params.h. */
    .tof_sig_lim   = 0.05f,
    .tof_pre_pclk  = 18.0f,
    .tof_fin_pclk  = 14.0f,
    .tof_budget_ms = 200.0f,
    .thr_vcomp     = 0.0f,
};

typedef struct {
    const char *name;
    size_t      off;
    float       min, max;
    bool        boot_only;
    /* Never inherit this field from an older blob. On any migration it
     * goes back to its default, unconditionally.
     *
     * For MODE ENABLES. The asymmetry is the whole argument: losing the
     * setting costs one console command, inheriting a wrong one costs a
     * flight. hold_auto inherited alt_acc_sigma's 2.0 through a
     * positional load, clamped to 1.0, and flew two
     * sessions with both holds on that nobody selected. */
    bool        no_migrate;
} param_desc_t;

/* no_migrate spelled out rather than left to C's zero-init, so the plain
 * P() rows need no edit AND the build stays clean under
 * -Wmissing-field-initializers. */
#define P(field, lo, hi, boot) \
    { #field, offsetof(params_t, field), lo, hi, boot, false }

/* A mode enable: 0 or 1, and never migrated. */
#define P_ENABLE(field, boot) \
    { #field, offsetof(params_t, field), 0.0f, 1.0f, boot, true }

/* Bounds are sanity rails, not tuning advice: wide enough for any value
 * someone would defend on purpose, tight enough that a typo (1200 for
 * 1.2) cannot arm a bomb. */
static const param_desc_t k_table[] = {
    P(rate_kp,       0.0f,   20.0f, false),
    P(rate_ki,       0.0f,    5.0f, false),
    P(rate_kd,       0.0f,    1.0f, false),
    P(yaw_kp,        0.0f,   20.0f, false),
    P(yaw_ki,        0.0f,    5.0f, false),
    P(yaw_kd,        0.0f,    1.0f, false),
    P(angle_kp,      0.0f,   20.0f, false),
    P(angle_max_deg, 5.0f,   60.0f, false),
    P(yaw_rate_max, 20.0f,  500.0f, false),
    P(rc_deadband,   0.0f,   50.0f, false),
    P(min_idle_us, 1000.0f, 1300.0f, false),
    P(gyro_lpf_hz,   0.0f,  180.0f, true),
    P(notch_hz,      0.0f,  180.0f, true),
    P(notch_q,       1.0f,   30.0f, true),
    /* upper bound 1.0 is the physics, not caution: >1 would just clip
     * at 2000 and lie about the headroom the mixer thinks it has. */
    P(m1_scale,      0.7f,    1.0f, false),
    P(m2_scale,      0.7f,    1.0f, false),
    P(m3_scale,      0.7f,    1.0f, false),
    P(m4_scale,      0.7f,    1.0f, false),
    /* +-10 deg is a mount tolerance, not a trim range. Past that the
     * board is not "slightly crooked", it is mounted wrong or the
     * surface LEVELCAL ran on was not level - either way the answer is
     * a screwdriver, not a parameter. */
    P(level_roll_deg,  -10.0f, 10.0f, false),
    P(level_pitch_deg, -10.0f, 10.0f, false),
    /* P7 alt-hold, all LIVE so the loop can be tuned without a reboot.
     * hover_thr's ceiling is 900 not 1000: a hover that needs 90% of the
     * usable span has no authority left to CLIMB with, and an alt-hold
     * with no headroom is a controller that can only ever descend. */
    P(alt_hover_thr,  100.0f, 900.0f, false),
    P(alt_vref,         6.0f,  30.0f, false),
    P(alt_vz_max,       0.1f,   3.0f, false),
    P(alt_kp_z,         0.0f,   5.0f, false),
    P(alt_kp_vz,        0.0f, 300.0f, false),
    P(alt_ki_vz,        0.0f, 300.0f, false),
    P(alt_i_max,        0.0f, 500.0f, false),
    P(alt_thr_db,       0.0f, 300.0f, false),
    P_ENABLE(alt_hold_auto, false),
    P(vel_kp,           0.0f,  20.0f, false),
    P(vel_ki,           0.0f,  10.0f, false),
    P(vel_i_max,        0.0f,  20.0f, false),
    P(vel_tilt_max,     1.0f,  30.0f, false),
    P(vel_max,          0.5f,  10.0f, false),
    P(alt_acc_sigma,   0.05f,   3.0f, false),
    /* BOOT: the range profile is written into the chip during its init
     * sequence, and changing it live would mean re-running that sequence
     * under the flight loop. SET + SAVE + power-cycle.
     *
     * tof_sig_lim's floor is 0.05 rather than 0: at 0 the chip accepts
     * literally any return and the "ground" becomes whatever reflected
     * last. The pclk bounds are ST's published lists; anything between
     * snaps to the nearest legal value at init. */
    P(tof_sig_lim,    0.02f,    1.0f, true),
    P(tof_pre_pclk,  12.0f,    18.0f, true),
    P(tof_fin_pclk,   8.0f,    14.0f, true),
    /* Upper bound 400 is not caution about the sensor, it is the
     * dead-sensor detector: TOF_EMPTY_BLANK counts consecutive polls with
     * nothing new, and a budget long enough would make a healthy chip
     * trip it. Read that comment before raising this. */
    P(tof_budget_ms, 20.0f,   400.0f, true),
    /* Appended when hold_auto was split. At the END, not
     * beside alt_hold_auto - see the note on vel_hold_auto in params.h. */
    P_ENABLE(vel_hold_auto, false),
    /* Appended. LIVE, not boot: the whole point of a throttle
     * law you can toggle is being able to feel it on and off in the same
     * bench session. */
    P_ENABLE(thr_vcomp, false),
};
#define N_PARAMS  (int)(sizeof(k_table) / sizeof(k_table[0]))

/* The NVS migration below copies the saved blob's floats straight onto
 * g_params, so params_t MUST be exactly N_PARAMS contiguous floats in
 * k_table's order. Both halves of that are load-bearing and neither is
 * obvious from reading either list, so assert what can be asserted at
 * compile time and check the rest once at init. */
_Static_assert(sizeof(params_t) == sizeof(float) * (sizeof(k_table) /
               sizeof(k_table[0])),
               "params_t must be exactly N_PARAMS floats with no padding "
               "- the NVS migration and the blackbox snapshot both index "
               "it as a flat float array");

typedef struct {
    uint32_t version;
    uint32_t count;
    /* crc32 over the NAMES of k_table[0..count-1], NUL separators
     * included. The blob's self-description, and the fix for the defect
     * that shipped.
     *
     * The load below maps saved floats onto fields BY POSITION. That is
     * only correct if slot i means the same thing in the saving build
     * and the loading build, which the append-only rule is supposed to
     * guarantee - and which nothing enforced at load time. The
     * _Static_assert catches a size change; the boot order-check catches
     * a reorder within one build; neither can see that YESTERDAY's build
     * had different names in those slots.
     *
     * What happened: six fields were inserted before alt_acc_sigma
     * without a version bump. Every saved value from slot 28 on landed
     * on the wrong field. hold_auto received alt_acc_sigma's 2.0, the
     * clamp below laundered it into a legal-looking 1.0, and the
     * aircraft flew two sessions with both holds enabled by nobody.
     *
     * Digesting the names makes that arithmetic. Append leaves the first
     * N names untouched, so a prefix load still proceeds exactly as
     * designed. Insert, rename or reorder changes the digest, and the
     * blob is refused instead of silently reinterpreted. */
    uint32_t names_crc;
    params_t p;
    uint32_t crc;        /* crc32 over version..p */
} blob_t;

/* Set when a blob EXISTED and was refused. Not set when there is simply
 * no blob - a fresh board on defaults is normal, a tuned board that
 * silently reverted to them is not. See params_were_defaulted(). */
static bool s_defaulted = false;

static float *field_ptr(const param_desc_t *d)
{
    return (float *)((uint8_t *)&g_params + d->off);
}

/* Digest of the first `n` field names. Feeding the NUL terminators too
 * is deliberate: without them "ab"+"c" and "a"+"bc" hash identically,
 * and a rename that only moves a boundary would slip through. */
static uint32_t names_crc(int n)
{
    uint32_t c = 0;
    if (n > N_PARAMS) n = N_PARAMS;
    for (int i = 0; i < n; i++) {
        c = esp_rom_crc32_le(c, (const uint8_t *)k_table[i].name,
                             strlen(k_table[i].name) + 1);
    }
    return c;
}

static uint32_t blob_crc(const blob_t *b)
{
    return esp_rom_crc32_le(0, (const uint8_t *)b,
                            offsetof(blob_t, crc));
}

/* Adopt an OLDER saved blob instead of throwing it away.
 *
 * Until now every PARAMS_VERSION bump wiped the table back to defaults,
 * and the cost of that is not theoretical: the v2 -> v3 bump silently
 * erased m1_scale 0.88 and yaw_rate_max 90 - a shakedown flight's worth
 * of tuning - and the loss was only noticed because the next log's
 * params snapshot was read carefully. A version bump is supposed to
 * mean "there are new fields", not "your aircraft is untuned again".
 *
 * Only v7+ blobs can be adopted: older ones have no names_crc field, so
 * their length never matches and they are refused (defaulted). Within v7+
 * it works because the on-flash layout is exactly
 *
 *     [u32 version][u32 count][u32 names_crc][count floats][u32 crc]
 *
 * and params fields are APPEND-ONLY ABI - a rule that predates this and
 * that the blackbox decoder already depends on, since it maps snapshot
 * floats onto names by COUNT. So an older blob's floats are a prefix of
 * the current struct, byte for byte. Copy the prefix, leave the rest at
 * their defaults, and say out loud which fields were defaulted.
 *
 * names_crc was added in v7 because that "so" was doing unearned work.
 * The prefix property holds only while the rule is FOLLOWED, and on
 * it was not: six fields were inserted mid-struct and every
 * saved value from slot 28 on was read onto the wrong field. The digest
 * turns the rule from a convention into a check.
 *
 * The CRC still has to pass over the blob's OWN length, so this is not
 * a weakening: corrupt data is still rejected, only the version-number
 * mismatch stops being fatal. A NEWER blob than the firmware (a
 * downgrade) is still refused outright - its extra fields are unknown,
 * and guessing at them is exactly the kind of silent reinterpretation
 * the version check exists to prevent. */
static bool params_load_blob(const uint8_t *raw, size_t len)
{
    /* version + count + names_crc + at least the trailing crc */
    if (len < 4 * sizeof(uint32_t)) return false;

    uint32_t version, count;
    memcpy(&version, raw + 0, sizeof(version));
    memcpy(&count,   raw + 4, sizeof(count));

    if (version > PARAMS_VERSION) {
        ESP_LOGW(TAG, "saved params are v%u, firmware is v%d - refusing to "
                 "guess at newer fields, using defaults",
                 (unsigned)version, PARAMS_VERSION);
        return false;
    }
    if (count == 0 || count > (uint32_t)N_PARAMS) {
        ESP_LOGW(TAG, "saved params count %u out of range - defaults",
                 (unsigned)count);
        return false;
    }
    const size_t body = 3 * sizeof(uint32_t) + count * sizeof(float);
    if (len != body + sizeof(uint32_t)) {
        ESP_LOGW(TAG, "saved params length %u does not match count %u "
                 "- defaults", (unsigned)len, (unsigned)count);
        return false;
    }

    uint32_t want_crc;
    memcpy(&want_crc, raw + body, sizeof(want_crc));
    if (esp_rom_crc32_le(0, raw, body) != want_crc) {
        ESP_LOGW(TAG, "saved params CRC mismatch - defaults");
        return false;
    }

    /* The names digest, checked AFTER the CRC so a corrupt blob is
     * reported as corrupt rather than as a layout change. */
    uint32_t saved_names;
    memcpy(&saved_names, raw + 2 * sizeof(uint32_t), sizeof(saved_names));
    const uint32_t have_names = names_crc((int)count);
    if (saved_names != have_names) {
        ESP_LOGE(TAG, "saved params LAYOUT MISMATCH over the first %u "
                 "field(s): digest %08x, this firmware %08x. The saved "
                 "slots do not mean what this build thinks they mean - "
                 "refusing to map them. USING DEFAULTS.",
                 (unsigned)count, (unsigned)saved_names,
                 (unsigned)have_names);
        return false;
    }

    /* g_params already holds the defaults, so copying a prefix leaves
     * every newer field at its default automatically. */
    memcpy(&g_params, raw + 3 * sizeof(uint32_t), count * sizeof(float));

    /* Mode enables never survive a migration. The digest above proves
     * the first `count` slots still MEAN the same thing, so this is not
     * about misreading them - it is that "the last person to fly this
     * build left the hold on" is not a fact a NEW build may assume. */
    if ((int)count != N_PARAMS) {
        for (int i = 0; i < (int)count; i++) {
            if (!k_table[i].no_migrate) continue;
            const float def = ((const float *)&k_defaults)[i];
            if (*field_ptr(&k_table[i]) != def) {
                ESP_LOGW(TAG, "  migration resets enable: %-16s -> %.4f",
                         k_table[i].name, (double)def);
            }
            *field_ptr(&k_table[i]) = def;
        }
    }

    /* Now make the loaded values obey k_table, because until this point
     * NOTHING did. The CRC proves the blob is the one that was written;
     * it says nothing about whether the numbers in it are legal. Every
     * bound in k_table was enforced in params_set() only, so any value
     * that arrived by another route - a blob saved by firmware whose
     * bounds were wider, a field whose meaning changed under an
     * append-only ABI, a half-erased NVS page that still CRCs - landed
     * on g_params unchecked and went straight into the flight loop.
     *
     * m*_scale is the sharp example. Its <= 1.0 ceiling is not caution,
     * it is the only thing keeping post-mix scaling from asking for more
     * than the mixer already clamped to 2000 (
     *). That ceiling was enforced in exactly one function,
     * and NVS load was not it.
     *
     * CLAMP per field rather than reject the blob. Rejecting means
     * falling back to defaults wholesale, which throws away level_roll/
     * pitch_deg and every gain that IS valid because one field is not -
     * an aircraft that is untuned and un-levelled because of a single
     * bad float is worse than one flying a clamped version of it. Each
     * repair is logged BY NAME so it is visible in the boot log rather
     * than inferred later from behaviour.
     *
     * A non-finite value cannot be clamped by comparison - every compare
     * against NaN is false - so it is replaced by the default outright. */
    int n_fixed = 0;
    bool enable_repaired = false;
    for (int i = 0; i < (int)count && i < N_PARAMS; i++) {
        float *p = field_ptr(&k_table[i]);
        const float was = *p;
        const bool  was_legal = isfinite(was) &&
                                was >= k_table[i].min && was <= k_table[i].max;
        if (!was_legal && k_table[i].no_migrate) enable_repaired = true;

        if (!isfinite(was)) {
            /* Indexing k_defaults as a flat float array is exactly what
             * the _Static_assert above licenses. */
            *p = ((const float *)&k_defaults)[i];
            ESP_LOGW(TAG, "  NOT FINITE: %-16s -> default %.4f",
                     k_table[i].name, (double)*p);
            n_fixed++;
        } else if (was < k_table[i].min) {
            *p = k_table[i].min;
            ESP_LOGW(TAG, "  below min:  %-16s %.4f -> %.4f",
                     k_table[i].name, (double)was, (double)*p);
            n_fixed++;
        } else if (was > k_table[i].max) {
            *p = k_table[i].max;
            ESP_LOGW(TAG, "  above max:  %-16s %.4f -> %.4f",
                     k_table[i].name, (double)was, (double)*p);
            n_fixed++;
        }
    }
    /* Per-field clamping is right for GENUINE single-field corruption -
     * one bad float should not cost the level trim and every gain that
     * is fine. It is wrong as a response to a whole blob that means
     * something else, and that is the case it silently handled on
     * vel_max arrived as 200 (it was tof_budget_ms), got
     * clamped to a perfectly legal-looking 10, and flew.
     *
     * So bound what clamping is allowed to claim. One or two repairs is
     * corruption; more than that, or any repair at all on a mode enable,
     * is a layout the digest failed to catch, and the project's own rule
     * applies - reject, do not clamp.
     *
     * The digest above should make this unreachable. It stays because a
     * second, independent check on the same failure is cheap, and
     * because the first one was believed to be enough too. */
    if (enable_repaired || n_fixed > 2) {
        ESP_LOGE(TAG, "%d saved param(s) out of range%s - that is a LAYOUT "
                 "mismatch, not corruption. Refusing the blob. USING "
                 "DEFAULTS.",
                 n_fixed, enable_repaired ? " including a mode enable" : "");
        return false;
    }
    if (n_fixed) {
        ESP_LOGW(TAG, "%d saved param(s) were out of range and have been "
                 "clamped - SAVE to make the clamped values permanent",
                 n_fixed);
    }

    if ((int)count == N_PARAMS) {
        ESP_LOGI(TAG, "params loaded from NVS (v%u, %d values)",
                 (unsigned)version, N_PARAMS);
    } else {
        ESP_LOGW(TAG, "params MIGRATED v%u -> v%d: %u values kept, %d new "
                 "fields left at defaults (SAVE to make it permanent)",
                 (unsigned)version, PARAMS_VERSION, (unsigned)count,
                 N_PARAMS - (int)count);
        for (int i = (int)count; i < N_PARAMS; i++) {
            ESP_LOGW(TAG, "  default: %-16s %.4f",
                     k_table[i].name, (double)*field_ptr(&k_table[i]));
        }
    }
    return true;
}

void params_init(void)
{
    g_params = k_defaults;

    /* The migration indexes params_t as a flat float array in k_table
     * order. The _Static_assert above catches a size mismatch; this
     * catches a REORDER, which has the same effect and no compile-time
     * tell. Checked once at boot rather than trusted. */
    for (int i = 0; i < N_PARAMS; i++) {
        if (k_table[i].off != (size_t)i * sizeof(float)) {
            ESP_LOGE(TAG, "params table order does not match params_t at "
                     "'%s' (offset %u, expected %u) - NOT loading NVS",
                     k_table[i].name, (unsigned)k_table[i].off,
                     (unsigned)(i * sizeof(float)));
            return;
        }
    }

    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGI(TAG, "no saved params, using defaults");
        return;
    }

    size_t len = 0;
    esp_err_t err = nvs_get_blob(h, NVS_KEY, NULL, &len);
    if (err != ESP_OK || len == 0 || len > sizeof(blob_t)) {
        nvs_close(h);
        ESP_LOGI(TAG, "no usable saved params, using defaults");
        return;
    }

    uint8_t raw[sizeof(blob_t)];
    err = nvs_get_blob(h, NVS_KEY, raw, &len);
    nvs_close(h);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "no saved params, using defaults");
        return;
    }

    /* A blob was here and we could not use it. That is the case worth
     * shouting about: this board HAD tuning and is now flying compiled
     * defaults - m*_scale 1.0, no level trim. Reaching this line with a
     * fresh board that never saved anything is normal and returns
     * earlier, without setting the flag. */
    if (!params_load_blob(raw, len)) {
        g_params    = k_defaults;
        s_defaulted = true;
        ESP_LOGE(TAG, "FLYING COMPILED DEFAULTS - the saved tuning was "
                      "refused. Motor trim and level calibration are NOT "
                      "loaded. Arming is blocked until you SAVE.");
    }
}

void params_reset_defaults(void)
{
    g_params = k_defaults;
    ESP_LOGW(TAG, "params reset to defaults IN RAM - SAVE to persist, "
                  "or reboot to get the saved blob back");
}

bool params_were_defaulted(void) { return s_defaulted; }

static const param_desc_t *find(const char *name)
{
    for (int i = 0; i < N_PARAMS; i++) {
        if (strcmp(k_table[i].name, name) == 0) return &k_table[i];
    }
    return NULL;
}

int params_set(const char *name, float value)
{
    const param_desc_t *d = find(name);
    if (!d) return -1;
    /* Written this way round so NaN fails: every comparison with NaN is
     * false, so a NaN can never satisfy the inside-bounds condition. A
     * NaN gain would NaN the mixer - sscanf parses "nan" happily. */
    if (!(value >= d->min && value <= d->max)) return -2;
    *field_ptr(d) = value;     /* 32-bit store: atomic for lock-free readers */
    return 0;
}

int params_get(const char *name, float *out)
{
    const param_desc_t *d = find(name);
    if (!d) return -1;
    *out = *field_ptr(d);
    return 0;
}

int params_save(void)
{
    blob_t b = {
        .version   = PARAMS_VERSION,
        .count     = (uint32_t)N_PARAMS,
        .names_crc = names_crc(N_PARAMS),
        .p         = g_params,
    };
    b.crc = blob_crc(&b);

    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) return -(int)err;
    err = nvs_set_blob(h, NVS_KEY, &b, sizeof(b));
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    if (err != ESP_OK) return -(int)err;
    /* What is on flash now matches what is running, so the "you are on
     * defaults" blocker has been answered by the pilot doing the one
     * thing that resolves it. */
    s_defaulted = false;
    ESP_LOGI(TAG, "params saved (%d values)", N_PARAMS);
    return 0;
}

int params_count(void) { return N_PARAMS; }

int params_describe(int i, const char **name, float *value,
                    float *min, float *max, bool *boot_only)
{
    if (i < 0 || i >= N_PARAMS) return -1;
    const param_desc_t *d = &k_table[i];
    if (name)      *name      = d->name;
    if (value)     *value     = *field_ptr(d);
    if (min)       *min       = d->min;
    if (max)       *max       = d->max;
    if (boot_only) *boot_only = d->boot_only;
    return 0;
}
