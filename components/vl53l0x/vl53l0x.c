/* VL53L0X driver - see vl53l0x.h for the wiring and the contract.
 *
 * The register-level init sequence, the tuning-settings blob and the
 * timing-budget arithmetic below are ST's (STSW-IMG005 VL53L0X_DataInit /
 * VL53L0X_StaticInit / VL53L0X_PerformRefCalibration), transcribed from
 * the widely used Pololu Arduino distillation of that API. The magic
 * numbers are not derivable from the datasheet - ST never published what
 * most of those registers mean - so they are kept verbatim and in order
 * rather than "cleaned up". Do not reorder them.
 *
 * What is ours: the ESP-IDF I2C plumbing, the sticky-error init style,
 * and the non-blocking sample path, because this shares the 100 kHz bus
 * with the mag and runs inside slow_sensor_task, which feeds the ~416 Hz
 * fusion loop. Pololu's read function busy-waits for a measurement;
 * doing that here would stall the sensor snapshot for a whole timing
 * budget (200 ms on this airframe). */

#include "vl53l0x.h"
#include "i2c_all.h"     /* I2C_MASTER_FREQ_HZ - shared bus speed */
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "VL53L0X";

#define VL53L0X_ADDR   0x29

/* Registers (names from ST's vl53l0x_device.h). Only the ones used. */
#define REG_SYSRANGE_START                  0x00
#define REG_SYSTEM_SEQUENCE_CONFIG          0x01
#define REG_SYSTEM_INTERMEASUREMENT_PERIOD  0x04
#define REG_SYSTEM_INTERRUPT_CONFIG_GPIO    0x0A
#define REG_SYSTEM_INTERRUPT_CLEAR          0x0B
#define REG_RESULT_INTERRUPT_STATUS         0x13
#define REG_RESULT_RANGE_STATUS             0x14
#define REG_MSRC_CONFIG_CONTROL             0x60
#define REG_FINAL_RANGE_MIN_COUNT_RATE      0x44
#define REG_PRE_RANGE_CONFIG_VCSEL_PERIOD   0x50
#define REG_PRE_RANGE_TIMEOUT_MACROP_HI     0x51
#define REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD 0x70
#define REG_FINAL_RANGE_TIMEOUT_MACROP_HI   0x71
#define REG_MSRC_CONFIG_TIMEOUT_MACROP      0x46
/* Phase-window and VCSEL-width registers, only touched when the VCSEL
 * pulse period changes. The valid-phase pair bounds the return phase the
 * chip will accept, and it is period-dependent: change the period without
 * changing these and the chip rejects the very returns the longer pulse
 * was meant to collect. */
#define REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW    0x56
#define REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH   0x57
#define REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW  0x47
#define REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH 0x48
#define REG_GLOBAL_CONFIG_VCSEL_WIDTH           0x32
#define REG_ALGO_PHASECAL_CONFIG_TIMEOUT        0x30
#define REG_ALGO_PHASECAL_LIM                   0x30   /* on page 0xFF=0x01 */
#define REG_IDENTIFICATION_MODEL_ID         0xC0
#define REG_IDENTIFICATION_REVISION_ID      0xC2
#define REG_OSC_CALIBRATE_VAL               0xF8
#define REG_GLOBAL_CONFIG_SPAD_ENABLES_REF0 0xB0
#define REG_GLOBAL_CONFIG_REF_EN_START_SEL  0xB6
#define REG_DYNAMIC_SPAD_NUM_REQUESTED      0x4E
#define REG_DYNAMIC_SPAD_REF_EN_START_OFF   0x4F
#define REG_GPIO_HV_MUX_ACTIVE_HIGH         0x84
#define REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP   0x89

#define MODEL_ID_VL53L0X   0xEE
/* ST's "range valid" code. ADVISORY - it is reported and
 * logged but no longer decides whether a measurement is used. See
 * vl53l0x_sample(). */
#define RANGE_STATUS_VALID 11      /* RESULT_RANGE_STATUS[7:3] == 11 */

/* Weak long-range returns are expected; weak close-range returns are not.
 * A false "target" with no real reflector was measured as a 40..198 mm
 * sawtooth, which is exactly the class of value that must not become a
 * ground reference. At 200 mm a genuine floor should clear ST's stock
 * 0.25 MCPS neighbourhood easily, while a faint/crosstalk return should
 * not. The inverse-square shape deliberately relaxes away with distance:
 * it protects the close-range flight path without undoing the long-range
 * profile that made >200 mm readings possible. */
#define CLOSE_SIGNAL_GATE_MM        250u
#define CLOSE_SIGNAL_REF_MM         200.0f
#define CLOSE_SIGNAL_REF_MCPS       0.20f
#define CLOSE_SIGNAL_CAP_MCPS       1.00f

/* Bounded like the other users of this bus: a wedged bus costs a bounded
 * stall, never a hang. */
#define I2C_TIMEOUT_MS     20

/* 0 = BACK-TO-BACK continuous mode: the chip starts the next measurement
 * the instant it finishes the last one, and the measurement itself is as
 * long as the timing budget says.
 *
 * It used to be 50 ms of timed mode, which only made sense against the
 * stock ~33 ms budget. At the 200 ms budget this module needs to see past
 * 24 cm, a fixed 50 ms period is a lie - the chip cannot range four times
 * faster than one measurement takes - so the period is handed back to the
 * chip. Same as the Arduino sketch this profile came from, which calls
 * startContinuous() with no argument. */
#define RANGE_PERIOD_MS    0

/* One I2C hiccup on a bus shared with the baro and mag is noise, not a
 * dead sensor - blanking on it would make the reading look flaky. Blank
 * only after a full second of failures (10 samples at the 10 Hz rate the
 * bus owner polls this at, ~9 Hz net of the INA226 slots). Same rule as
 * the INA226. */
#define TOF_FAIL_BLANK     10

/* A chip that ACKs but has STOPPED RANGING is invisible to the streak
 * above, and that is not hypothetical: it destroyed an aircraft on
 *. The ToF's 3V3 wire popped out of its header on a hard
 * landing while SDA/SCL stayed wired to a live bus. The pin protection
 * diodes kept the chip alive enough to ACK every read and report "no new
 * measurement ready" forever, so no read ever failed, s_fail_streak
 * stayed 0, the cache never blanked, and 211 mm was served as a live
 * distance for two entire flights.
 *
 * The interrupt-status early-out below is correct for a healthy chip
 * polled asynchronously - "nothing new yet" genuinely is not an error -
 * but it must not be free forever.
 *
 * The margin has to be checked against the BUDGET, because the budget is
 * now what sets the measurement rate: at 200 ms back-to-back the chip
 * produces ~5 measurements a second while this is polled at ~9 Hz, so a
 * healthy sensor shows at most one or two empty polls between
 * measurements. Ten in a row is still a dead sensor, not a slow one, with
 * room to spare. That margin is why tof_budget_ms is bounded at 400: at
 * ~9 Hz polling even 400 ms leaves a worst-case streak of ~4. Raise that
 * bound and this constant has to be raised with it, or a perfectly
 * healthy rangefinder starts declaring itself dead.
 *
 * Blanking is what makes this reach the estimator: a blanked cache
 * publishes VL53L0X_MM_NONE, the seq bump below carries it into the
 * fusion loop's fresh-sample branch, and that branch already routes
 * MM_NONE to vert_filter_tof_lost(). */
#define TOF_EMPTY_BLANK    10

/* Polling budgets for the two init waits. Generous: init runs once at
 * boot with no flight tasks alive, and the only thing that matters is
 * that a dead chip cannot wedge the boot. */
#define INIT_POLL_TIMEOUT_MS 500

static i2c_master_dev_handle_t s_dev = NULL;
static i2c_master_bus_handle_t s_bus = NULL;   /* for vl53l0x_reprobe */
static uint8_t  s_stop_variable = 0;
static uint32_t s_budget_us     = 0;
/* Copied from the caller's cfg on the first init and reused by every
 * reprobe. A chip that lost power and came back must come back with the
 * SAME range profile - reverting to ST's stock 24 cm one mid-flight,
 * silently, is the kind of thing this file already has a scar from. */
static vl53l0x_cfg_t s_cfg = {0};

/* Held across init so the sampling task cannot interleave a result read
 * into the middle of the sequence. The init walks the chip through
 * page-select registers (0xFF/0x80); a stray read of 0x13 between two of
 * them would land on the wrong page. The IDF driver serialises single
 * transactions, not sequences of them. */
static volatile bool s_busy = false;

/* Written by the sampling task, read by telemetry on another core. A
 * 16-bit aligned load/store is atomic on this CPU, so no lock: readers
 * see the old or the new value, never a torn one. */
static volatile uint16_t s_cached_mm = VL53L0X_MM_NONE;
/* Bumped on every completed sample, valid or not. The vertical filter
 * runs at ~416 Hz and this chip produces a number every tof_budget_ms -
 * ~5 times a second at the 200 ms this airframe boots with, NOT the
 * ~9 Hz it is polled at - so without a freshness marker the filter would
 * apply the same measurement ~80 times in a row, telling a Kalman update
 * it had 80 independent observations of the same thing, which collapses the
 * covariance and makes the estimator believe a stale reading far more
 * than it should. Comparing values instead of counting them is not
 * enough: two consecutive samples that happen to agree to the
 * millimetre are exactly the still-hovering case that matters most. */
static volatile uint32_t s_sample_seq = 0;
static volatile uint8_t  s_status    = 255;
static uint8_t           s_fail_streak = 0;
/* Consecutive polls that found no completed measurement. See
 * TOF_EMPTY_BLANK - this is the counter that catches a chip which ACKs
 * but has stopped ranging. */
static uint8_t           s_empty_streak = 0;

/* ---- raw I2C ---- */

static esp_err_t rd(uint8_t reg, uint8_t *dst, size_t n)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, dst, n, I2C_TIMEOUT_MS);
}

static esp_err_t wr(uint8_t reg, const uint8_t *src, size_t n)
{
    uint8_t buf[8];
    if (n + 1 > sizeof(buf)) return ESP_ERR_INVALID_SIZE;
    buf[0] = reg;
    memcpy(&buf[1], src, n);
    return i2c_master_transmit(s_dev, buf, n + 1, I2C_TIMEOUT_MS);
}

/* ---- sticky-error helpers, init only ----
 *
 * The init sequence is ~110 transactions. Checking each one at the call
 * site would bury the sequence itself, and the sequence is the thing that
 * has to stay readable and comparable against ST's. So the first failure
 * latches into s_err and every later call is a no-op: nothing is written
 * to a chip that has already stopped answering, and init reports the
 * failure once at the end. */
static esp_err_t s_err = ESP_OK;

static void w8(uint8_t reg, uint8_t v)
{
    if (s_err == ESP_OK) s_err = wr(reg, &v, 1);
}

static uint8_t r8(uint8_t reg)
{
    uint8_t v = 0;
    if (s_err == ESP_OK) s_err = rd(reg, &v, 1);
    return v;
}

static void w16(uint8_t reg, uint16_t v)
{
    uint8_t b[2] = { (uint8_t)(v >> 8), (uint8_t)v };
    if (s_err == ESP_OK) s_err = wr(reg, b, 2);
}

static uint16_t r16(uint8_t reg)
{
    uint8_t b[2] = {0, 0};
    if (s_err == ESP_OK) s_err = rd(reg, b, 2);
    return ((uint16_t)b[0] << 8) | b[1];
}

static void w32(uint8_t reg, uint32_t v)
{
    uint8_t b[4] = { (uint8_t)(v >> 24), (uint8_t)(v >> 16),
                     (uint8_t)(v >> 8),  (uint8_t)v };
    if (s_err == ESP_OK) s_err = wr(reg, b, 4);
}

/* ---- timeout / VCSEL arithmetic (ST's, unit-for-unit) ---- */

/* PLL period 1655 ps, macro period 2304 VCLKs. */
static uint32_t macro_period_ns(uint8_t vcsel_pclks)
{
    return ((2304u * (uint32_t)vcsel_pclks * 1655u) + 500u) / 1000u;
}

static uint8_t decode_vcsel_period(uint8_t reg_val)
{
    return (uint8_t)((reg_val + 1) << 1);
}

/* Register format is "(LSByte * 2^MSByte) + 1". */
static uint16_t decode_timeout(uint16_t reg_val)
{
    return (uint16_t)((reg_val & 0x00FF) << ((reg_val & 0xFF00) >> 8)) + 1;
}

static uint16_t encode_timeout(uint32_t timeout_mclks)
{
    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;

    if (timeout_mclks == 0) return 0;

    ls_byte = timeout_mclks - 1;
    while ((ls_byte & 0xFFFFFF00u) > 0) {
        ls_byte >>= 1;
        ms_byte++;
    }
    return (uint16_t)((ms_byte << 8) | (ls_byte & 0xFF));
}

static uint32_t mclks_to_us(uint16_t mclks, uint8_t vcsel_pclks)
{
    uint32_t ns = macro_period_ns(vcsel_pclks);
    return (((uint32_t)mclks * ns) + 500u) / 1000u;
}

static uint32_t us_to_mclks(uint32_t us, uint8_t vcsel_pclks)
{
    uint32_t ns = macro_period_ns(vcsel_pclks);
    return ((us * 1000u) + (ns / 2u)) / ns;
}

typedef struct {
    bool tcc, dss, msrc, pre_range, final_range;
} seq_enables_t;

typedef struct {
    uint16_t pre_range_vcsel_pclks, final_range_vcsel_pclks;
    uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
    uint32_t msrc_dss_tcc_us,    pre_range_us,    final_range_us;
} seq_timeouts_t;

static void get_seq_enables(seq_enables_t *e)
{
    uint8_t cfg = r8(REG_SYSTEM_SEQUENCE_CONFIG);
    e->tcc         = (cfg >> 4) & 1;
    e->dss         = (cfg >> 3) & 1;
    e->msrc        = (cfg >> 2) & 1;
    e->pre_range   = (cfg >> 6) & 1;
    e->final_range = (cfg >> 7) & 1;
}

static void get_seq_timeouts(const seq_enables_t *e, seq_timeouts_t *t)
{
    t->pre_range_vcsel_pclks =
        decode_vcsel_period(r8(REG_PRE_RANGE_CONFIG_VCSEL_PERIOD));

    t->msrc_dss_tcc_mclks = (uint16_t)(r8(REG_MSRC_CONFIG_TIMEOUT_MACROP) + 1);
    t->msrc_dss_tcc_us    = mclks_to_us(t->msrc_dss_tcc_mclks,
                                        (uint8_t)t->pre_range_vcsel_pclks);

    t->pre_range_mclks = decode_timeout(r16(REG_PRE_RANGE_TIMEOUT_MACROP_HI));
    t->pre_range_us    = mclks_to_us(t->pre_range_mclks,
                                     (uint8_t)t->pre_range_vcsel_pclks);

    t->final_range_vcsel_pclks =
        decode_vcsel_period(r8(REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD));

    t->final_range_mclks = decode_timeout(r16(REG_FINAL_RANGE_TIMEOUT_MACROP_HI));
    if (e->pre_range) t->final_range_mclks -= t->pre_range_mclks;
    t->final_range_us = mclks_to_us(t->final_range_mclks,
                                    (uint8_t)t->final_range_vcsel_pclks);
}

/* Fixed overheads of one measurement, in us (ST's numbers). */
#define OH_START       1910
#define OH_END          960
#define OH_MSRC         660
#define OH_TCC          590
#define OH_DSS          690
#define OH_PRE_RANGE    660
#define OH_FINAL_RANGE  550

static uint32_t get_timing_budget(void)
{
    seq_enables_t  e;
    seq_timeouts_t t;
    uint32_t budget = OH_START + OH_END;

    get_seq_enables(&e);
    get_seq_timeouts(&e, &t);

    if (e.tcc)            budget += t.msrc_dss_tcc_us + OH_TCC;
    if (e.dss)            budget += 2 * (t.msrc_dss_tcc_us + OH_DSS);
    else if (e.msrc)      budget += t.msrc_dss_tcc_us + OH_MSRC;
    if (e.pre_range)      budget += t.pre_range_us + OH_PRE_RANGE;
    if (e.final_range)    budget += t.final_range_us + OH_FINAL_RANGE;

    return budget;
}

/* Everything the sequence does not spend on overhead and the earlier
 * steps goes to the final range step. Called after the sequence config
 * changes, which is the whole point: disabling MSRC/TCC frees time that
 * has to be handed to the final range explicitly or it is simply lost. */
static bool set_timing_budget(uint32_t budget_us)
{
    seq_enables_t  e;
    seq_timeouts_t t;
    uint32_t used = OH_START + OH_END;

    get_seq_enables(&e);
    get_seq_timeouts(&e, &t);

    if (e.tcc)            used += t.msrc_dss_tcc_us + OH_TCC;
    if (e.dss)            used += 2 * (t.msrc_dss_tcc_us + OH_DSS);
    else if (e.msrc)      used += t.msrc_dss_tcc_us + OH_MSRC;
    if (e.pre_range)      used += t.pre_range_us + OH_PRE_RANGE;
    if (!e.final_range)   return true;

    used += OH_FINAL_RANGE;
    if (used > budget_us) return false;      /* requested budget too small */

    uint32_t final_mclks = us_to_mclks(budget_us - used,
                                       (uint8_t)t.final_range_vcsel_pclks);
    /* The final range timeout is stored INCLUDING the pre-range one -
     * they use different VCSEL periods, so the sum only makes sense in
     * macro periods. */
    if (e.pre_range) final_mclks += t.pre_range_mclks;

    w16(REG_FINAL_RANGE_TIMEOUT_MACROP_HI, encode_timeout(final_mclks));
    s_budget_us = budget_us;
    return true;
}

/* ---- VCSEL pulse period (ST's VL53L0X_SetVcselPulsePeriod) ---- */

/* Defined further down with the rest of the init sequence, which is kept
 * in ST's order; changing the VCSEL period needs it, and moving it up
 * here would break that order for no gain. */
static bool single_ref_calibration(uint8_t vhv_init_byte);

/* Longer VCSEL pulses put more light on the target, which is most of what
 * separates the stock ~1.2 m profile from the ~2 m one. The cost is time
 * (each step takes longer, so the timing budget has to be re-divided) and
 * a re-run of the phase calibration, because the phase reference the
 * calibration produced belongs to the OLD pulse period.
 *
 * The per-period magic numbers below are ST's, one row per legal period,
 * and like the tuning blob they are not derivable from the datasheet.
 * Legal periods are the only ones ST published: pre-range 12/14/16/18,
 * final-range 8/10/12/14. Anything else is refused rather than rounded
 * inside here - the caller snaps first, so a refusal here means a bug. */
typedef enum { VCSEL_PRE_RANGE, VCSEL_FINAL_RANGE } vcsel_type_t;

static bool set_vcsel_pulse_period(vcsel_type_t type, uint8_t pclks)
{
    /* Register encoding is (period / 2) - 1. */
    uint8_t reg_val = (uint8_t)((pclks >> 1) - 1);

    /* Read the timeouts in MICROSECONDS before the period changes: they
     * are stored in macro periods, and a macro period is exactly what is
     * about to be redefined. Converting out to us, changing the period,
     * then converting back is what keeps each step's DURATION constant
     * across the change instead of its tick count. */
    seq_enables_t  e;
    seq_timeouts_t t;
    get_seq_enables(&e);
    get_seq_timeouts(&e, &t);

    if (type == VCSEL_PRE_RANGE) {
        switch (pclks) {
        case 12: w8(REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18); break;
        case 14: w8(REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30); break;
        case 16: w8(REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40); break;
        case 18: w8(REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50); break;
        default: return false;
        }
        w8(REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
        w8(REG_PRE_RANGE_CONFIG_VCSEL_PERIOD, reg_val);

        w16(REG_PRE_RANGE_TIMEOUT_MACROP_HI,
            encode_timeout(us_to_mclks(t.pre_range_us, pclks)));

        /* MSRC's timeout is a single byte holding (mclks - 1), so it
         * saturates rather than wrapping. */
        uint32_t msrc_mclks = us_to_mclks(t.msrc_dss_tcc_us, pclks);
        w8(REG_MSRC_CONFIG_TIMEOUT_MACROP,
           (uint8_t)(msrc_mclks > 256u ? 255u : (msrc_mclks - 1u)));
    } else {
        switch (pclks) {
        case 8:
            w8(REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
            w8(REG_GLOBAL_CONFIG_VCSEL_WIDTH,           0x02);
            w8(REG_ALGO_PHASECAL_CONFIG_TIMEOUT,        0x0C);
            w8(0xFF, 0x01); w8(REG_ALGO_PHASECAL_LIM, 0x30); w8(0xFF, 0x00);
            break;
        case 10:
            w8(REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
            w8(REG_GLOBAL_CONFIG_VCSEL_WIDTH,           0x03);
            w8(REG_ALGO_PHASECAL_CONFIG_TIMEOUT,        0x09);
            w8(0xFF, 0x01); w8(REG_ALGO_PHASECAL_LIM, 0x20); w8(0xFF, 0x00);
            break;
        case 12:
            w8(REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
            w8(REG_GLOBAL_CONFIG_VCSEL_WIDTH,           0x03);
            w8(REG_ALGO_PHASECAL_CONFIG_TIMEOUT,        0x08);
            w8(0xFF, 0x01); w8(REG_ALGO_PHASECAL_LIM, 0x20); w8(0xFF, 0x00);
            break;
        case 14:
            w8(REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
            w8(REG_GLOBAL_CONFIG_VCSEL_WIDTH,           0x03);
            w8(REG_ALGO_PHASECAL_CONFIG_TIMEOUT,        0x07);
            w8(0xFF, 0x01); w8(REG_ALGO_PHASECAL_LIM, 0x20); w8(0xFF, 0x00);
            break;
        default: return false;
        }
        w8(REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
        w8(REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD, reg_val);

        /* Same "stored including the pre-range timeout" rule as
         * set_timing_budget - see the comment there. */
        uint32_t fr_mclks = us_to_mclks(t.final_range_us, pclks);
        if (e.pre_range) fr_mclks += t.pre_range_mclks;
        w16(REG_FINAL_RANGE_TIMEOUT_MACROP_HI, encode_timeout(fr_mclks));
    }

    /* Re-divide the budget across the steps at their new durations. A
     * refusal here is not fatal: set_timing_budget writes nothing when
     * the request does not fit, so the per-step timeouts written above
     * simply stand and the measurement takes a little longer. Losing the
     * whole rangefinder over that would be the worse trade. */
    if (!set_timing_budget(s_budget_us))
        ESP_LOGW(TAG, "timing budget %lu us too small for the new VCSEL "
                 "period - keeping the longer sequence",
                 (unsigned long)s_budget_us);

    /* The phase calibration is period-specific, so redo it. Sequence
     * config is saved and restored: this runs mid-init, and clobbering it
     * would silently disable the range steps that follow. */
    uint8_t seq = r8(REG_SYSTEM_SEQUENCE_CONFIG);
    w8(REG_SYSTEM_SEQUENCE_CONFIG, 0x02);
    if (!single_ref_calibration(0x00)) return false;
    w8(REG_SYSTEM_SEQUENCE_CONFIG, seq);

    return s_err == ESP_OK;
}

/* Snap to the nearest period ST published. A SET typo (13) becomes 14
 * with a log line rather than either failing init or being silently
 * ignored - the sensor is optional, but a silently-ignored range setting
 * is how you end up flying the profile you thought you had changed. */
static uint8_t snap_pclks(uint8_t want, const uint8_t *legal, int n)
{
    uint8_t best = legal[0];
    int     best_d = 255;

    for (int i = 0; i < n; i++) {
        int d = (int)want - (int)legal[i];
        if (d < 0) d = -d;
        if (d < best_d) { best_d = d; best = legal[i]; }
    }
    return best;
}

static void apply_range_profile(void)
{
    static const uint8_t PRE_LEGAL[]   = { 12, 14, 16, 18 };
    static const uint8_t FINAL_LEGAL[] = { 8, 10, 12, 14 };

    /* Signal rate limit FIRST, and - the whole point of this function
     * running where it does - AFTER the tuning blob.
     *
     *, the second time: this used to be written inside
     * DataInit, and the tuning blob overwrote it. The blob contains TWO
     * writes to register 0x44. One is on register page 1
     * ({0xFF,0x01} ... {0x44,0xFF}) and is a different register; the
     * other is on page 0:
     *
     *     {0xFF,0x00},{0x44,0x00},{0x45,0x20},
     *
     * and 0x44/0x45 on page 0 IS the 16-bit final-range minimum count
     * rate. 0x0020 = 32 = 0.25 MCPS in Q9.7 - ST's stock value, the one
     * that caps this module at 24 cm. So the driver asked for 0.05,
     * the blob put 0.25 back, and the chip ranged to 24 cm while the
     * boot log said 0.05 because that line echoed the REQUEST.
     *
     * The Arduino sketch this profile is copied from never had the
     * problem: setSignalRateLimit() is called after init() returns,
     * which is after the blob. This is that same order. */
    if (s_cfg.signal_rate_limit_mcps > 0.0f)
        w16(REG_FINAL_RANGE_MIN_COUNT_RATE,
            (uint16_t)(s_cfg.signal_rate_limit_mcps * (1 << 7)));

    if (s_cfg.pre_range_pclks) {
        uint8_t p = snap_pclks(s_cfg.pre_range_pclks, PRE_LEGAL, 4);
        if (p != s_cfg.pre_range_pclks)
            ESP_LOGW(TAG, "pre-range VCSEL %u is not a legal period, using %u",
                     s_cfg.pre_range_pclks, p);
        if (!set_vcsel_pulse_period(VCSEL_PRE_RANGE, p))
            ESP_LOGW(TAG, "pre-range VCSEL period %u refused", p);
    }
    if (s_cfg.final_range_pclks) {
        uint8_t p = snap_pclks(s_cfg.final_range_pclks, FINAL_LEGAL, 4);
        if (p != s_cfg.final_range_pclks)
            ESP_LOGW(TAG, "final-range VCSEL %u is not a legal period, using %u",
                     s_cfg.final_range_pclks, p);
        if (!set_vcsel_pulse_period(VCSEL_FINAL_RANGE, p))
            ESP_LOGW(TAG, "final-range VCSEL period %u refused", p);
    }

    /* Timing budget LAST. set_vcsel_pulse_period re-applies the OLD budget
     * on its way out, so setting this first would be undone by the period
     * changes above - the same order the Arduino sketch this profile came
     * from uses.
     *
     * This is integration time, and it is the biggest single lever on how
     * far the sensor sees: 200 ms against ST's stock ~33 ms is six times
     * the photons per measurement. It is also the biggest cost - see
     * start_continuous() for what it does to the sample rate. */
    if (s_cfg.timing_budget_us && !set_timing_budget(s_cfg.timing_budget_us))
        ESP_LOGW(TAG, "timing budget %lu us refused (too small for the "
                 "sequence) - keeping %lu us",
                 (unsigned long)s_cfg.timing_budget_us,
                 (unsigned long)s_budget_us);
}

/* ---- init ---- */

/* Bounded poll: true if (r8(reg) & mask) became nonzero in time. Any I2C
 * error latches in s_err and ends the wait. */
static bool wait_for_bits(uint8_t reg, uint8_t mask)
{
    int64_t deadline = esp_timer_get_time() + INIT_POLL_TIMEOUT_MS * 1000LL;
    while (esp_timer_get_time() < deadline) {
        uint8_t v = r8(reg);
        if (s_err != ESP_OK) return false;
        if (v & mask) return true;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return false;
}

/* Reference SPAD count and type, out of the chip's NVM. ST reads this
 * through a longer "get info from device" path; this is the short one. */
static bool get_spad_info(uint8_t *count, bool *type_is_aperture)
{
    w8(0x80, 0x01);
    w8(0xFF, 0x01);
    w8(0x00, 0x00);

    w8(0xFF, 0x06);
    w8(0x83, r8(0x83) | 0x04);
    w8(0xFF, 0x07);
    w8(0x81, 0x01);

    w8(0x80, 0x01);

    w8(0x94, 0x6B);
    w8(0x83, 0x00);
    if (!wait_for_bits(0x83, 0xFF)) return false;
    w8(0x83, 0x01);

    uint8_t tmp = r8(0x92);
    *count            = tmp & 0x7F;
    *type_is_aperture = (tmp >> 7) & 0x01;

    w8(0x81, 0x00);
    w8(0xFF, 0x06);
    w8(0x83, r8(0x83) & ~0x04);
    w8(0xFF, 0x01);
    w8(0x00, 0x01);

    w8(0xFF, 0x00);
    w8(0x80, 0x00);

    return s_err == ESP_OK;
}

/* DefaultTuningSettings from ST's vl53l0x_tuning.h, verbatim and in
 * order. Undocumented by design - ST ships them as an opaque blob. */
static const uint8_t TUNING[][2] = {
    {0xFF,0x01},{0x00,0x00},
    {0xFF,0x00},{0x09,0x00},{0x10,0x00},{0x11,0x00},
    {0x24,0x01},{0x25,0xFF},{0x75,0x00},
    {0xFF,0x01},{0x4E,0x2C},{0x48,0x00},{0x30,0x20},
    {0xFF,0x00},{0x30,0x09},{0x54,0x00},{0x31,0x04},{0x32,0x03},
    {0x40,0x83},{0x46,0x25},{0x60,0x00},{0x27,0x00},{0x50,0x06},
    {0x51,0x00},{0x52,0x96},{0x56,0x08},{0x57,0x30},{0x61,0x00},
    {0x62,0x00},{0x64,0x00},{0x65,0x00},{0x66,0xA0},
    {0xFF,0x01},{0x22,0x32},{0x47,0x14},{0x49,0xFF},{0x4A,0x00},
    {0xFF,0x00},{0x7A,0x0A},{0x7B,0x00},{0x78,0x21},
    {0xFF,0x01},{0x23,0x34},{0x42,0x00},{0x44,0xFF},{0x45,0x26},
    {0x46,0x05},{0x40,0x40},{0x0E,0x06},{0x20,0x1A},{0x43,0x40},
    {0xFF,0x00},{0x34,0x03},{0x35,0x44},
    {0xFF,0x01},{0x31,0x04},{0x4B,0x09},{0x4C,0x05},{0x4D,0x04},
    {0xFF,0x00},{0x44,0x00},{0x45,0x20},{0x47,0x08},{0x48,0x28},
    {0x67,0x00},{0x70,0x04},{0x71,0x01},{0x72,0xFE},{0x76,0x00},
    {0x77,0x00},
    {0xFF,0x01},{0x0D,0x01},
    {0xFF,0x00},{0x80,0x01},{0x01,0xF8},
    {0xFF,0x01},{0x8E,0x01},{0x00,0x01},{0xFF,0x00},{0x80,0x00},
};

/* One VHV / phase calibration pass: kick it off, wait for the completion
 * interrupt, clear it. */
static bool single_ref_calibration(uint8_t vhv_init_byte)
{
    w8(REG_SYSRANGE_START, 0x01 | vhv_init_byte);
    if (!wait_for_bits(REG_RESULT_INTERRUPT_STATUS, 0x07)) return false;
    w8(REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
    w8(REG_SYSRANGE_START, 0x00);
    return s_err == ESP_OK;
}

static void start_continuous(uint32_t period_ms)
{
    w8(0x80, 0x01);
    w8(0xFF, 0x01);
    w8(0x00, 0x00);
    w8(0x91, s_stop_variable);
    w8(0x00, 0x01);
    w8(0xFF, 0x00);
    w8(0x80, 0x00);

    if (period_ms == 0) {
        w8(REG_SYSRANGE_START, 0x02);        /* back-to-back continuous */
        return;
    }

    /* The inter-measurement period is counted in the chip's own
     * oscillator ticks, so it has to be scaled by the factory
     * calibration value before it means milliseconds. */
    uint16_t osc_cal = r16(REG_OSC_CALIBRATE_VAL);
    if (osc_cal != 0) period_ms *= osc_cal;

    w32(REG_SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);
    w8(REG_SYSRANGE_START, 0x04);            /* timed continuous mode */
}

bool vl53l0x_init(i2c_master_bus_handle_t bus_handle,
                  const vl53l0x_cfg_t *cfg)
{
    /* The ToF lives on bus B, which i2c_all_init is allowed to fail
     * without stopping the boot. Guard BEFORE s_bus is written, so a
     * dead bus at boot cannot poison the reprobe path with a NULL. */
    if (!bus_handle) {
        ESP_LOGW(TAG, "bus B never came up - ground distance disabled");
        return false;
    }

    s_bus = bus_handle;
    /* NULL means "keep the profile already configured" - that is the
     * reprobe path, which must not quietly drop back to stock. */
    if (cfg) s_cfg = *cfg;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = VL53L0X_ADDR,
        .scl_speed_hz    = I2C_MASTER_FREQ_HZ,
    };
    i2c_master_dev_handle_t dev;
    if (i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev) != ESP_OK) {
        ESP_LOGW(TAG, "bus_add_device failed - ground distance disabled");
        return false;
    }
    s_busy = true;          /* cleared on every exit path below */
    s_dev  = dev;
    s_err  = ESP_OK;
    /* A re-probe (TOF console command) starts the chip's health clean.
     * Leftover streaks from the dead instance would otherwise blank the
     * first poll of a freshly working sensor. */
    s_fail_streak  = 0;
    s_empty_streak = 0;

    uint8_t model = 0;
    if (rd(REG_IDENTIFICATION_MODEL_ID, &model, 1) != ESP_OK ||
        model != MODEL_ID_VL53L0X) {
        ESP_LOGW(TAG, "not detected (model 0x%02X, want 0x%02X) - ground "
                 "distance disabled. On USB-only power the sensor rail is "
                 "unpowered, which looks exactly like this.", model,
                 MODEL_ID_VL53L0X);
        goto fail;
    }

    /* ---- VL53L0X_DataInit ---- */

    /* 2.8 V I/O mode. The breakout regulates VIN down to 2.8 V for the
     * die and level-shifts SDA/SCL, so the pads must be told they are
     * NOT running at the 1.8 V default. */
    w8(REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP,
       r8(REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP) | 0x01);

    w8(0x88, 0x00);                          /* "set I2C standard mode" */

    w8(0x80, 0x01);
    w8(0xFF, 0x01);
    w8(0x00, 0x00);
    s_stop_variable = r8(0x91);              /* needed by start_continuous */
    w8(0x00, 0x01);
    w8(0xFF, 0x00);
    w8(0x80, 0x00);

    /* Disable the SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE
     * (bit 4) limit checks. */
    w8(REG_MSRC_CONFIG_CONTROL, r8(REG_MSRC_CONFIG_CONTROL) | 0x12);

    /* ST's stock final range signal rate limit, 0.25 MCPS in Q9.7.
     *
     * STOCK ON PURPOSE, and it must stay stock: the tuning blob below
     * rewrites this same register to this same 0.25 anyway, so anything
     * written here is decoration. The configured limit is applied in
     * apply_range_profile(), after the blob - see the comment there for
     * what putting it here instead cost. */
    w16(REG_FINAL_RANGE_MIN_COUNT_RATE, (uint16_t)(0.25f * (1 << 7)));

    w8(REG_SYSTEM_SEQUENCE_CONFIG, 0xFF);

    if (s_err != ESP_OK) goto fail_io;

    /* ---- VL53L0X_StaticInit ---- */

    uint8_t spad_count = 0;
    bool    spad_aperture = false;
    if (!get_spad_info(&spad_count, &spad_aperture)) {
        ESP_LOGW(TAG, "SPAD info read failed - ground distance disabled");
        goto fail;
    }

    /* The good-SPAD map lives in NVM; ST reads it the long way, these
     * registers hold the same six bytes. */
    uint8_t spad_map[6];
    if (rd(REG_GLOBAL_CONFIG_SPAD_ENABLES_REF0, spad_map, 6) != ESP_OK)
        goto fail_io;

    w8(0xFF, 0x01);
    w8(REG_DYNAMIC_SPAD_REF_EN_START_OFF, 0x00);
    w8(REG_DYNAMIC_SPAD_NUM_REQUESTED, 0x2C);
    w8(0xFF, 0x00);
    w8(REG_GLOBAL_CONFIG_REF_EN_START_SEL, 0xB4);

    /* Keep exactly spad_count enabled reference SPADs, starting at the
     * first one of the right type (aperture SPADs start at 12). */
    uint8_t first = spad_aperture ? 12 : 0;
    uint8_t on    = 0;
    for (uint8_t i = 0; i < 48; i++) {
        if (i < first || on == spad_count) {
            spad_map[i / 8] &= (uint8_t)~(1 << (i % 8));
        } else if ((spad_map[i / 8] >> (i % 8)) & 1) {
            on++;
        }
    }
    if (s_err == ESP_OK)
        s_err = wr(REG_GLOBAL_CONFIG_SPAD_ENABLES_REF0, spad_map, 6);

    for (size_t i = 0; i < sizeof(TUNING) / sizeof(TUNING[0]); i++)
        w8(TUNING[i][0], TUNING[i][1]);

    /* Interrupt on "new sample ready", active low - polled, not wired to
     * a GPIO on this aircraft, but the status bit is how the sample path
     * knows a measurement finished. */
    w8(REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    w8(REG_GPIO_HV_MUX_ACTIVE_HIGH, r8(REG_GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10);
    w8(REG_SYSTEM_INTERRUPT_CLEAR, 0x01);

    if (s_err != ESP_OK) goto fail_io;

    /* Read the budget the sequence currently implies, disable MSRC and
     * TCC, then write the SAME total back so the time those two steps
     * were using is handed to the final range step instead of lost. */
    uint32_t budget = get_timing_budget();
    w8(REG_SYSTEM_SEQUENCE_CONFIG, 0xE8);
    if (!set_timing_budget(budget) || s_err != ESP_OK) {
        ESP_LOGW(TAG, "timing budget setup failed - ground distance disabled");
        goto fail;
    }

    /* ---- VL53L0X_PerformRefCalibration ---- */

    w8(REG_SYSTEM_SEQUENCE_CONFIG, 0x01);
    if (!single_ref_calibration(0x40)) {
        ESP_LOGW(TAG, "VHV calibration failed - ground distance disabled");
        goto fail;
    }
    w8(REG_SYSTEM_SEQUENCE_CONFIG, 0x02);
    if (!single_ref_calibration(0x00)) {
        ESP_LOGW(TAG, "phase calibration failed - ground distance disabled");
        goto fail;
    }
    w8(REG_SYSTEM_SEQUENCE_CONFIG, 0xE8);    /* restore */

    /* Range profile LAST, after reference calibration: it re-runs the
     * phase calibration itself with the new pulse period, and doing it
     * before the VHV/phase pair above would just have that pair overwrite
     * the result. Same order Pololu's long-range example uses - init(),
     * then the period changes, then startContinuous(). */
    apply_range_profile();

    start_continuous(RANGE_PERIOD_MS);
    if (s_err != ESP_OK) goto fail_io;

    uint8_t rev = 0;
    rd(REG_IDENTIFICATION_REVISION_ID, &rev, 1);
    /* The VCSEL periods AND the signal rate limit are read BACK off the
     * chip rather than echoed from the config: a snapped, refused,
     * clamped or OVERWRITTEN value has to be visible in the log,
     * because "the profile I thought I set" is exactly the thing that is
     * hard to notice going wrong.
     *
     * The signal limit is here because it was echoed, and the echo hid a
     * real fault for a day: the tuning blob was silently putting ST's
     * 0.25 back while this line printed the 0.05 that had been asked
     * for. Everything else on this line was already read back, which is
     * exactly why everything else was true. */
    uint8_t pre_p  = decode_vcsel_period(r8(REG_PRE_RANGE_CONFIG_VCSEL_PERIOD));
    uint8_t fin_p  = decode_vcsel_period(r8(REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD));
    float   sig_lim = (float)r16(REG_FINAL_RANGE_MIN_COUNT_RATE) / (float)(1 << 7);
    ESP_LOGI(TAG, "found (rev 0x%02X), %u %s SPADs, budget %lu us, "
             "signal limit %.2f MCPS, VCSEL %u/%u pclks, continuous @%d ms",
             rev, spad_count,
             spad_aperture ? "aperture" : "non-aperture",
             (unsigned long)s_budget_us, (double)sig_lim, pre_p, fin_p,
             RANGE_PERIOD_MS);
    s_busy = false;
    return true;

fail_io:
    ESP_LOGW(TAG, "I2C failure during init (%s) - ground distance disabled",
             esp_err_to_name(s_err));
fail:
    i2c_master_bus_rm_device(s_dev);
    s_dev  = NULL;
    s_busy = false;
    return false;
}

bool vl53l0x_present(void)
{
    return s_dev != NULL;
}

bool vl53l0x_reprobe(void)
{
    if (!s_bus) return false;

    /* Present AND ranging - nothing to do. NO_TARGET counts as ranging:
     * that is a live chip saying "nothing within 2 m". */
    if (s_dev && s_cached_mm != VL53L0X_MM_NONE) return true;

    /* Present but BLANKED. The old "if (s_dev) return true" made this
     * unrecoverable: a chip that lost power and got it back comes up in
     * its power-on defaults, no longer in continuous ranging mode, so
     * the interrupt never asserts again and the empty-poll counter keeps
     * it blanked forever - while this function cheerfully reported it
     * present and re-initialised nothing. Only a reboot brought it back,
     * which on an aircraft means a wire that re-seats itself in flight
     * stays dead for the rest of the flight.
     *
     * The handle is stale in every way that matters, so tear it down and
     * walk the whole init sequence again. */
    if (s_dev) {
        i2c_master_dev_handle_t old = s_dev;
        s_dev = NULL;              /* the sampler bails on this first */
        i2c_master_bus_rm_device(old);
    }
    return vl53l0x_init(s_bus, NULL);   /* NULL = keep the range profile */
}

uint16_t vl53l0x_cached_mm(void)
{
    return s_cached_mm;
}

uint8_t vl53l0x_last_status(void)
{
    return s_status;
}

uint32_t vl53l0x_sample_seq(void)
{
    return s_sample_seq;
}

static bool close_range_signal_ok(uint16_t mm, float signal_mcps)
{
    if (mm >= CLOSE_SIGNAL_GATE_MM) return true;

    float d_mm = (mm == 0) ? 1.0f : (float)mm;
    float scale = CLOSE_SIGNAL_REF_MM / d_mm;
    float min_mcps = CLOSE_SIGNAL_REF_MCPS * scale * scale;
    if (min_mcps > CLOSE_SIGNAL_CAP_MCPS) min_mcps = CLOSE_SIGNAL_CAP_MCPS;

    return signal_mcps >= min_mcps;
}

esp_err_t vl53l0x_read_raw(uint16_t *mm, uint8_t *status,
                           float *signal_mcps, float *ambient_mcps)
{
    if (!s_dev || s_busy) return ESP_ERR_INVALID_STATE;

    /* Deliberately does NOT clear the interrupt: the result registers
     * hold the last completed measurement either way, so a console query
     * cannot steal a sample from the cached path. */
    uint8_t b[12];
    esp_err_t err = rd(REG_RESULT_RANGE_STATUS, b, sizeof(b));
    if (err != ESP_OK) return err;

    *status = (b[0] >> 3) & 0x0F;
    *mm     = ((uint16_t)b[10] << 8) | b[11];

    /* Two fields out of the SAME 12-byte block the driver has always
     * fetched and thrown away - no extra transaction, which matters on a
     * 100 kHz bus the baro and mag already use 2.4 ms of every 20 ms.
     *
     * The offsets are VL53L0X_GetRangingMeasurementData's, and they are
     * not guesswork: the status at b[0] and the distance at b[10..11]
     * just above come from that same ST layout and are already known
     * good on this module. Both rates are Q9.7 MCPS - the same
     * fixed-point the signal rate LIMIT is written in over in
     * apply_range_profile(). */
    if (signal_mcps)
        *signal_mcps  = (float)(((uint16_t)b[6] << 8) | b[7]) / (float)(1 << 7);
    if (ambient_mcps)
        *ambient_mcps = (float)(((uint16_t)b[8] << 8) | b[9]) / (float)(1 << 7);
    return ESP_OK;
}

static void note_fail(void)
{
    if (s_fail_streak < TOF_FAIL_BLANK) s_fail_streak++;
    if (s_fail_streak >= TOF_FAIL_BLANK) {
        s_cached_mm = VL53L0X_MM_NONE;
        s_status    = 255;
        /* Bump the seq for the same reason the empty-streak path does:
         * the fusion loop only looks at the ToF when the counter moves,
         * so a blank that does not move it leaves the estimator holding
         * an absolute reference it no longer has. Blanking the cache
         * alone fixes the dashboard and nothing else. */
        s_sample_seq++;
    }
}

void vl53l0x_sample(void)
{
    if (!s_dev || s_busy) return;

    /* No new measurement yet is NOT a failure - the chip ranges back-to-
     * back every tof_budget_ms (200 ms here) and this is polled
     * asynchronously to it. Only
     * I2C errors count against the blanking streak.
     *
     * But it is not free forever either: TOF_EMPTY_BLANK explains why a
     * chip that ACKs while no longer ranging has to be caught here, and
     * why the seq is bumped on the way out - that is what carries
     * "the ToF is gone" into the estimator. */
    uint8_t irq = 0;
    if (rd(REG_RESULT_INTERRUPT_STATUS, &irq, 1) != ESP_OK) { note_fail(); return; }
    if ((irq & 0x07) == 0) {
        if (s_empty_streak < TOF_EMPTY_BLANK) s_empty_streak++;
        if (s_empty_streak >= TOF_EMPTY_BLANK) {
            s_cached_mm = VL53L0X_MM_NONE;
            s_status    = 255;
            s_sample_seq++;
        }
        return;
    }

    /* One 12-byte block instead of two reads: the status byte and the
     * distance at offset 10 come from the same measurement, and one
     * transaction costs less bus time than two on a 100 kHz bus that the
     * baro and mag are already using 2.4 ms of every 20 ms. */
    uint8_t b[12];
    if (rd(REG_RESULT_RANGE_STATUS, b, sizeof(b)) != ESP_OK) { note_fail(); return; }

    uint8_t  st  = (b[0] >> 3) & 0x0F;
    uint16_t mm  = ((uint16_t)b[10] << 8) | b[11];
    float    sig = (float)(((uint16_t)b[6] << 8) | b[7]) / (float)(1 << 7);

    /* A completed measurement: the chip is ranging, whatever the range
     * turns out to be. NO_TARGET is a live sensor saying "nothing out
     * there" and must not count as a stopped one. */
    s_empty_streak = 0;

    /* Clearing the interrupt is what releases the next measurement into
     * the result registers. If this write fails the SAME reading would
     * be re-read forever - a frozen distance that looks alive - so a
     * failed clear counts against the streak like any other error. */
    uint8_t one = 0x01;
    bool cleared = (wr(REG_SYSTEM_INTERRUPT_CLEAR, &one, 1) == ESP_OK);

    s_status = st;

    /* The range status is RECORDED but no longer GATES the measurement.
     *
     * It used to require status 11 (ST's "range valid"). Measured on the
     * bench with this module on the long-range profile, the
     * chip marks most returns past ~200 mm as low-confidence, so the gate
     * threw away real, correct distances and the sensor looked like it
     * could only see 20 cm. Pololu's library does not consult the status
     * at all, and that is the configuration that was measured reaching
     * 1700 mm on this exact module.
     *
     * What still gates:
     *
     *   1. the chip's own no-return sentinel. Out of range, the result
     *      register holds 8190/8191, and shipping THAT as a height would
     *      put the aircraft's ground reference 8 m below it.
     *
     *   2. weak close-range returns. Return power falls with distance, so
     *      a real 40..200 mm target should be bright. A weak signal in
     *      that range is not "low confidence long range"; it is the chip
     *      localising stray/crosstalk light as a fake near target.
     *
     * The status remains visible in vl53l0x_last_status() and the TOF
     * console command, so a chip refusing to stand behind its numbers is
     * still diagnosable - it just no longer silently blinds the aircraft. */
    if (mm < VL53L0X_NO_RETURN_MM && close_range_signal_ok(mm, sig)) {
        /* 0 mm is the "no reading at all" sentinel, so a genuine
         * touching-the-floor measurement is floored at 1 mm rather than
         * reported as a dead sensor. */
        s_cached_mm = (mm == 0) ? 1 : mm;
    } else {
        s_cached_mm = VL53L0X_MM_NO_TARGET;
    }
    s_sample_seq++;

    if (cleared) s_fail_streak = 0;
    else         note_fail();
}
