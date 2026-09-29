#include <string.h>
#include <math.h>
#include "lsm6dsox.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "LSM6DSOX";

/* ---- Accelerometer rail watch (see LSM6DSOX_CLIP_RAW in the header) ----
 * Incremented in lsm6dsox_read, which runs in the 416 Hz fusion task, so
 * this has to stay three compares and an add: no logging, no locking, no
 * allocation. Plain uint32_t - the readers are a console command and a
 * human, and a torn read of a monotonic counter is off by one sample. */
static uint32_t s_accel_samples = 0;
static uint32_t s_clip_samples  = 0;   /* ANY axis at the rail */
static uint32_t s_clip_axis[3]  = {0}; /* chip frame x,y,z */

uint32_t lsm6dsox_clip_count(void)         { return s_clip_samples;  }
uint32_t lsm6dsox_accel_sample_count(void) { return s_accel_samples; }

void lsm6dsox_clip_axis_counts(uint32_t out[3])
{
    out[0] = s_clip_axis[0];
    out[1] = s_clip_axis[1];
    out[2] = s_clip_axis[2];
}

/* ---- Liveness counters (see LSM6DSOX_FROZEN_READS in the header) ----
 * Same rules as the rail watch above: bumped in the 416 Hz read path,
 * read by a console command, no locking. */
static uint32_t s_uniform_reads = 0;
static uint32_t s_frozen_reads  = 0;

void lsm6dsox_liveness_counts(uint32_t *uniform, uint32_t *frozen)
{
    if (uniform) *uniform = s_uniform_reads;
    if (frozen)  *frozen  = s_frozen_reads;
}

/* ---- Refusal diagnostics (see LSM6DSOX_STICKY_READS in the header) ----
 * Same single writer as the counters above: the fusion task, through
 * lsm6dsox_read. */
static lsm6dsox_diag_t s_diag;
/* -1 s, so the first capture after boot (t >= 0) always logs. Not
 * INT64_MIN: t - INT64_MIN overflows. */
static int64_t         s_dump_logged_us = -1000000;

void lsm6dsox_diag_get(lsm6dsox_diag_t *out)
{
    if (out) memcpy(out, &s_diag, sizeof(*out));
}

/* ---- SPI helpers (Mode 3: CPOL=1, CPHA=1) ---- */

static esp_err_t write_reg(spi_device_handle_t spi, uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { reg & 0x7F, val };   /* bit 7 = 0 for write */
    spi_transaction_t t = {
        .length    = 16,
        .tx_buffer = tx,
    };
    return spi_device_polling_transmit(spi, &t);
}

static esp_err_t read_reg(spi_device_handle_t spi, uint8_t reg, uint8_t *buf, size_t len)
{
    uint8_t tx[15] = {0};
    uint8_t rx[15] = {0};
    tx[0] = reg | 0x80;                    /* bit 7 = 1 for read */

    spi_transaction_t t = {
        .length    = 8 * (1 + len),
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    esp_err_t err = spi_device_polling_transmit(spi, &t);
    if (err == ESP_OK) {
        memcpy(buf, &rx[1], len);
    }
    return err;
}

/* ---- Refusal dump ----
 * INT1 is sampled FIRST: nothing below reads an output register, so the
 * dump itself cannot clear a pending data-ready. Eight single-byte reads,
 * ~0.1 ms, at most twice per episode. */
static const uint8_t k_dump_regs[8] = {
    LSM6DSOX_WHO_AM_I, LSM6DSOX_INT1_CTRL, LSM6DSOX_CTRL1_XL, LSM6DSOX_CTRL2_G,
    LSM6DSOX_CTRL3_C,  LSM6DSOX_CTRL4_C,   LSM6DSOX_CTRL9_XL, LSM6DSOX_STATUS_REG,
};

static void dump_capture(lsm6dsox_handle_t *dev, lsm6dsox_dump_t *d,
                         uint8_t kind, uint8_t stuck)
{
    d->t_us    = esp_timer_get_time();
    d->episode = s_diag.episodes;
    d->kind    = kind;
    d->stuck   = stuck;
    d->int1    = dev->int1_gpio >= 0
               ? (int8_t)gpio_get_level((gpio_num_t)dev->int1_gpio) : -1;
    d->spi_err = ESP_OK;
    for (int i = 0; i < 8; i++) {
        esp_err_t e = read_reg(dev->spi_dev, k_dump_regs[i], &d->reg[i], 1);
        if (e != ESP_OK) { d->spi_err = e; d->reg[i] = 0; }
    }
}

/* One line on the USB log per capture. Episode starts are limited to one a
 * second - a run of dropouts must not turn the 416 Hz task into a printf
 * loop, and STATS keeps the latest episode whatever this skips. The sticky
 * dump (`always`) is at most one per episode and is the one that matters,
 * so it is never skipped. */
static void dump_log(const char *what, const lsm6dsox_dump_t *d, bool always)
{
    if (!always && d->t_us - s_dump_logged_us < 1000000) return;
    s_dump_logged_us = d->t_us;
    ESP_LOGW(TAG, "%s: episode %lu, %s 0x%02X | WHO_AM_I %02X INT1_CTRL %02X "
             "CTRL1_XL %02X CTRL2_G %02X CTRL3_C %02X CTRL4_C %02X CTRL9_XL %02X "
             "STATUS %02X | INT1 pin %s%s",
             what, (unsigned long)d->episode,
             d->kind == 1 ? "uniform" : "frozen", d->stuck,
             d->reg[0], d->reg[1], d->reg[2], d->reg[3],
             d->reg[4], d->reg[5], d->reg[6], d->reg[7],
             d->int1 < 0 ? "?" : (d->int1 ? "HIGH" : "LOW"),
             d->spi_err == ESP_OK ? "" : " (a dump read FAILED)");
}

/* Called for every refused burst. Counts the episode and takes the dumps;
 * the burst is still refused by the caller exactly as before. */
static void note_refusal(lsm6dsox_handle_t *dev, uint8_t kind, uint8_t stuck)
{
    dev->refused_run++;
    s_diag.current = dev->refused_run;
    if (dev->refused_run > s_diag.longest) s_diag.longest = dev->refused_run;

    if (dev->refused_run == 1) {
        s_diag.episodes++;
        s_diag.have_sticky = false;          /* belongs to the old episode */
        dump_capture(dev, &s_diag.first, kind, stuck);
        s_diag.have_first = true;
        dump_log("IMU refused a burst", &s_diag.first, false);
    } else if (dev->refused_run == LSM6DSOX_STICKY_READS) {
        dump_capture(dev, &s_diag.sticky, kind, stuck);
        s_diag.have_sticky = true;
        dump_log("IMU STILL refusing", &s_diag.sticky, true);
    }
}

/* ---- Public API ---- */

lsm6dsox_handle_t *lsm6dsox_init(void)
{
    /* 1. Init SPI bus */
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = LSM6DSOX_PIN_MOSI,
        .miso_io_num = LSM6DSOX_PIN_MISO,
        .sclk_io_num = LSM6DSOX_PIN_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 32,
    };
    if (spi_bus_initialize(LSM6DSOX_SPI_HOST, &bus_cfg, SPI_DMA_DISABLED) != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed");
        return NULL;
    }

    /* 2. Add device (Mode 3) */
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = LSM6DSOX_SPI_CLK_HZ,
        .mode           = 3,
        .spics_io_num   = LSM6DSOX_PIN_CS,
        .queue_size     = 1,
    };

    lsm6dsox_handle_t *dev = calloc(1, sizeof(lsm6dsox_handle_t));
    if (!dev) return NULL;
    dev->int1_gpio = -1;

    if (spi_bus_add_device(LSM6DSOX_SPI_HOST, &dev_cfg, &dev->spi_dev) != ESP_OK) {
        ESP_LOGE(TAG, "SPI add device failed");
        free(dev);
        return NULL;
    }

    vTaskDelay(pdMS_TO_TICKS(50));

    /* 3. WHO_AM_I */
    uint8_t id = 0;
    read_reg(dev->spi_dev, LSM6DSOX_WHO_AM_I, &id, 1);
    if (id != LSM6DSOX_WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "WHO_AM_I mismatch: 0x%02X (expected 0x6C)", id);
        /* Release what this init acquired, or a later retry/re-init hits
         * "host already in use" instead of the real fault. */
        spi_bus_remove_device(dev->spi_dev);
        spi_bus_free(LSM6DSOX_SPI_HOST);
        free(dev);
        return NULL;
    }
    ESP_LOGI(TAG, "LSM6DSOX found (ID: 0x%02X)", id);

    /* 4. Software reset */
    write_reg(dev->spi_dev, LSM6DSOX_CTRL3_C, 0x01);
    vTaskDelay(pdMS_TO_TICKS(20));

    /* 4b. SPI only, before anything else is configured - I3C first, as in
     * ST's own example. See the SPI-only block in lsm6dsox.h for why. */
    write_reg(dev->spi_dev, LSM6DSOX_CTRL9_XL, LSM6DSOX_CTRL9_XL_VAL);
    write_reg(dev->spi_dev, LSM6DSOX_CTRL4_C,  LSM6DSOX_CTRL4_C_VAL);
    write_reg(dev->spi_dev, LSM6DSOX_CTRL3_C,  LSM6DSOX_CTRL3_C_VAL);

    /* 5. Configure:
     *    CTRL1_XL = 0x6E  -> 416 Hz ODR, 8g FS, LPF2 enable
     *    CTRL2_G  = 0x64  -> 416 Hz ODR, 500 dps FS
     *    CTRL6_C  = 0x00  -> Gyro LPF: see CTRL4_C
     *    CTRL4_C  = 0x06  -> gyro digital LPF1 + I2C off, written in 4b
     *    CTRL8_XL = 0x09  -> Accel LPF: ODR/4 (low-pass for vibration)
     *
     * CTRL1_XL bit layout: ODR_XL[7:4] FS_XL[3:2] LPF2_XL_EN[1].
     * FS_XL is NOT in ascending order - 00=2g, 01=16g, 10=4g, 11=8g -
     * so 8g is 0110 11 1 0 = 0x6E, one bit away from the 0x6A that ran
     * previously.
     *
     * WHY 8g, and not the 4g that was fine for a year: the accel rides
     * on a +1 g DC offset, so at 4g FS only +3 g of headroom is left
     * above gravity, and hover vibration was already peaking near it
     * AFTER the on-chip LPF2. Clipping is one-sided there - the positive
     * tail is cut, the negative tail is not - and a MEMS accel rectifies
     * that asymmetry into a DC offset. Measured: -3.06
     * m/s^2 on earth-frame Z above 1400 us throttle, which the vertical
     * filter has to absorb as a bias it never actually had. 8g doubles
     * the headroom to +7 g at the cost of halving the resolution to
     * 0.244 mg/LSB (0.0024 m/s^2) - far below the at-rest noise floor,
     * so nothing downstream loses anything real. lsm6dsox_clip_count()
     * is what says whether the rail is still being touched. */
    write_reg(dev->spi_dev, LSM6DSOX_CTRL1_XL, 0x6E);
    write_reg(dev->spi_dev, LSM6DSOX_CTRL2_G,  0x64);
    write_reg(dev->spi_dev, LSM6DSOX_CTRL6_C,  0x00);
    write_reg(dev->spi_dev, LSM6DSOX_CTRL8_XL, 0x09);

    /* Route GYRO data-ready to the INT1 pin (INT1_CTRL bit 1,
     * INT1_DRDY_G). Wired to GPIO 15 on this board; fusion_task pins its
     * loop to that edge instead of a software timer.
     *
     * Gyro, not accel: both run at 416 Hz here and the gyro is what the
     * attitude integration is actually paced by.
     *
     * Level mode, not pulsed - the line stays high until the data is
     * read, and reading it clears the line so the next sample makes a
     * fresh rising edge. If the task ever fails to keep up, no new edge
     * appears and the interrupt would stall forever, so fusion_task waits
     * with a timeout and reads anyway when it fires. That read clears
     * DRDY and restarts the edges by itself. */
    write_reg(dev->spi_dev, LSM6DSOX_INT1_CTRL, 0x02);

    /* 5b. Read every configured register back (an echoed setting is
     * not evidence). A register that does not hold its value FAILS init:
     * a wrong CTRL2_G is a 4x gyro scale error, and the fix this block
     * exists to verify - I2C/I3C off - is otherwise unobservable. Read
     * twice, so one garbled read-back is logged as exactly that (itself a
     * symptom worth seeing) instead of costing the boot. Bit i of cfg_bad
     * is k_cfg[i]. */
    static const struct { uint8_t reg, val; const char *name; } k_cfg[8] = {
        { LSM6DSOX_CTRL9_XL,  LSM6DSOX_CTRL9_XL_VAL, "CTRL9_XL"  },
        { LSM6DSOX_CTRL4_C,   LSM6DSOX_CTRL4_C_VAL,  "CTRL4_C"   },
        { LSM6DSOX_CTRL3_C,   LSM6DSOX_CTRL3_C_VAL,  "CTRL3_C"   },
        { LSM6DSOX_CTRL1_XL,  0x6E,                  "CTRL1_XL"  },
        { LSM6DSOX_CTRL2_G,   0x64,                  "CTRL2_G"   },
        { LSM6DSOX_CTRL6_C,   0x00,                  "CTRL6_C"   },
        { LSM6DSOX_CTRL8_XL,  0x09,                  "CTRL8_XL"  },
        { LSM6DSOX_INT1_CTRL, 0x02,                  "INT1_CTRL" },
    };
    uint8_t bad = 0;
    for (int i = 0; i < 8; i++) {
        uint8_t v1 = 0, v2 = 0;
        read_reg(dev->spi_dev, k_cfg[i].reg, &v1, 1);
        if (v1 == k_cfg[i].val) continue;
        read_reg(dev->spi_dev, k_cfg[i].reg, &v2, 1);
        if (v2 == k_cfg[i].val) {
            ESP_LOGW(TAG, "%s read back 0x%02X, then 0x%02X on a re-read - "
                          "one garbled SPI read at init", k_cfg[i].name, v1, v2);
            continue;
        }
        bad |= (uint8_t)(1u << i);
        ESP_LOGE(TAG, "%s holds 0x%02X (then 0x%02X), wrote 0x%02X",
                 k_cfg[i].name, v1, v2, k_cfg[i].val);
    }
    s_diag.cfg_bad = bad;
    if (bad) {
        ESP_LOGE(TAG, "config did not stick (mask 0x%02X) - IMU init FAILED", bad);
        spi_bus_remove_device(dev->spi_dev);
        spi_bus_free(LSM6DSOX_SPI_HOST);
        free(dev);
        return NULL;
    }
    ESP_LOGI(TAG, "config read back OK: SPI only (I2C+I3C off), BDU, "
                  "416 Hz, 8 g, 500 dps, gyro DRDY on INT1");

    vTaskDelay(pdMS_TO_TICKS(100));

    /* 6. Gyro bias calibration (motion-gated, see lsm6dsox_calibrate_bias).
     * A capture it REFUSES - an axis that never moved, or an offset no real
     * part has - means the chip is not delivering data, and an IMU that is
     * not delivering data fails init: no fusion task, no heartbeat, no arm,
     * and the boot log says why. A capture that was merely noisy is still
     * accepted, loudly, exactly as before. */
    if (lsm6dsox_calibrate_bias(dev) == ESP_ERR_INVALID_RESPONSE) {
        ESP_LOGE(TAG, "no real gyro data at boot - IMU init FAILED");
        spi_bus_remove_device(dev->spi_dev);
        spi_bus_free(LSM6DSOX_SPI_HOST);
        free(dev);
        return NULL;
    }

    return dev;
}

esp_err_t lsm6dsox_calibrate_bias(lsm6dsox_handle_t *dev)
{
    /* Average 1000 samples, MOTION-GATED. A bias captured while the
     * board is handled/vibrating poisons the attitude until the next
     * calibration ("drift that comes and goes"). Gate on per-axis
     * standard deviation: a still board measures ~0.04 dps std;
     * handling shows up as 10x that. Retry up to 3 times before
     * accepting a suspect result (loudly). */
    const int   N = 1000;
    const float STILL_STD_DPS = 0.30f;   /* reject attempt above this */
    uint8_t raw[6];
    float bx = 0, by = 0, bz = 0, worst_std = 0;
    /* Raw range per chip axis over the attempt being kept - the frozen
     * test below. Exact integer compares, deliberately: a float std can
     * come out of E[x^2]-E[x]^2 cancellation looking like noise when the
     * input never changed at all. */
    int16_t lo[3] = {0}, hi[3] = {0};

    for (int attempt = 1; attempt <= 3; attempt++) {
        ESP_LOGI(TAG, "Calibrating gyro bias (keep board still), attempt %d...",
                 attempt);
        float sum[3] = {0}, sumsq[3] = {0};
        for (int a = 0; a < 3; a++) { lo[a] = INT16_MAX; hi[a] = INT16_MIN; }

        for (int i = 0; i < N; i++) {
            read_reg(dev->spi_dev, LSM6DSOX_OUTX_L_G, raw, 6);
            int16_t gx = (int16_t)((raw[1] << 8) | raw[0]);
            int16_t gy = (int16_t)((raw[3] << 8) | raw[2]);
            int16_t gz = (int16_t)((raw[5] << 8) | raw[4]);
            const int16_t r3[3] = { gx, gy, gz };
            for (int a = 0; a < 3; a++) {
                if (r3[a] < lo[a]) lo[a] = r3[a];
                if (r3[a] > hi[a]) hi[a] = r3[a];
            }
            /* Remap into the body frame BEFORE accumulating, so the
             * stored bias is in the same frame lsm6dsox_read subtracts
             * it in. */
            float cgx = (float)gx * LSM6DSOX_GYRO_SCALE;
            float cgy = (float)gy * LSM6DSOX_GYRO_SCALE;
            float cgz = (float)gz * LSM6DSOX_GYRO_SCALE;
            float s[3];
            IMU_REMAP(s[0], s[1], s[2], cgx, cgy, cgz);
            for (int a = 0; a < 3; a++) {
                sum[a]   += s[a];
                sumsq[a] += s[a] * s[a];
            }
            vTaskDelay(pdMS_TO_TICKS(2));  /* ~500 Hz sample rate */
        }

        bx = sum[0] / N; by = sum[1] / N; bz = sum[2] / N;
        worst_std = 0;
        float mean[3] = { bx, by, bz };
        for (int a = 0; a < 3; a++) {
            float var = sumsq[a] / N - mean[a] * mean[a];
            float sd  = var > 0 ? sqrtf(var) : 0;
            if (sd > worst_std) worst_std = sd;
        }

        if (worst_std <= STILL_STD_DPS) {
            break;                          /* clean capture */
        }
        ESP_LOGW(TAG, "Board moving during bias cal (std %.2f dps > %.2f)%s",
                 (double)worst_std, (double)STILL_STD_DPS,
                 attempt < 3 ? " - retrying" : " - ACCEPTING SUSPECT BIAS");
    }

    /* REFUSE, don't store. The still-board gate above only catches TOO
     * MUCH motion; a frozen bus shows NONE and sailed through it (incident 55:
     * std 0.000, stored as a 67 dps bias). A still board still has noise,
     * so an axis that never moved one LSB in 1000 reads is not a sensor
     * reading - and no real part has a zero-rate offset past
     * LSM6DSOX_BIAS_MAX_DPS. Either way the previous bias stays. */
    const bool frozen = (lo[0] == hi[0]) || (lo[1] == hi[1]) ||
                        (lo[2] == hi[2]);
    const bool implausible = fabsf(bx) > LSM6DSOX_BIAS_MAX_DPS ||
                             fabsf(by) > LSM6DSOX_BIAS_MAX_DPS ||
                             fabsf(bz) > LSM6DSOX_BIAS_MAX_DPS;
    if (frozen || implausible) {
        ESP_LOGE(TAG, "Gyro bias %.3f, %.3f, %.3f dps (std %.3f) REFUSED: %s. "
                      "Bias left at %.3f, %.3f, %.3f",
                 (double)bx, (double)by, (double)bz, (double)worst_std,
                 frozen ? "an axis never changed - frozen or garbage bus"
                        : "bigger than any real zero-rate offset",
                 (double)dev->gyro_bias.bx, (double)dev->gyro_bias.by,
                 (double)dev->gyro_bias.bz);
        return ESP_ERR_INVALID_RESPONSE;
    }

    dev->gyro_bias.bx = bx;
    dev->gyro_bias.by = by;
    dev->gyro_bias.bz = bz;
    ESP_LOGI(TAG, "Gyro bias: %.3f, %.3f, %.3f dps (std %.3f)",
             (double)dev->gyro_bias.bx, (double)dev->gyro_bias.by,
             (double)dev->gyro_bias.bz, (double)worst_std);

    return worst_std <= STILL_STD_DPS ? ESP_OK : ESP_ERR_INVALID_STATE;
}

esp_err_t lsm6dsox_read(lsm6dsox_handle_t *dev, lsm6dsox_data_t *out)
{
    /* Burst read 12 bytes: gyro (6) + accel (6) from contiguous registers */
    uint8_t raw[12];
    esp_err_t err = read_reg(dev->spi_dev, LSM6DSOX_OUTX_L_G, raw, 12);
    if (err != ESP_OK) return err;

    /* Liveness, on the raw burst and BEFORE anything is counted or
     * converted - a burst that fails here is not a sample. See
     * LSM6DSOX_FROZEN_READS in the header for why both are impossible for
     * a working chip. *out is left untouched, like any failed read. */
    bool uniform = true;
    for (int i = 1; i < 12; i++) {
        if (raw[i] != raw[0]) { uniform = false; break; }
    }
    if (uniform) {
        s_uniform_reads++;
        note_refusal(dev, 1, raw[0]);
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (memcmp(raw, dev->last_raw, sizeof(raw)) == 0) {
        if (dev->same_reads < UINT16_MAX) dev->same_reads++;
    } else {
        memcpy(dev->last_raw, raw, sizeof(raw));
        dev->same_reads = 0;
    }
    if (dev->same_reads > LSM6DSOX_FROZEN_READS) {
        s_frozen_reads++;
        note_refusal(dev, 2, raw[0]);
        return ESP_ERR_INVALID_STATE;
    }
    if (dev->refused_run) {                  /* the episode is over */
        dev->refused_run = 0;
        s_diag.current = 0;
    }

    /* Gyro: registers 0x22-0x27 */
    int16_t gx = (int16_t)((raw[1]  << 8) | raw[0]);
    int16_t gy = (int16_t)((raw[3]  << 8) | raw[2]);
    int16_t gz = (int16_t)((raw[5]  << 8) | raw[4]);

    /* Accel: registers 0x28-0x2D */
    int16_t ax = (int16_t)((raw[7]  << 8) | raw[6]);
    int16_t ay = (int16_t)((raw[9]  << 8) | raw[8]);
    int16_t az = (int16_t)((raw[11] << 8) | raw[10]);

    /* Rail watch, on the RAW counts and before the remap: clipping is a
     * property of one ADC channel, and the remap mixes channels. Two
     * explicit compares per axis rather than abs() - abs(-32768) is
     * undefined for int16 and would be exactly the case we are hunting. */
    s_accel_samples++;
    uint32_t railed = 0;
    if (ax >=  LSM6DSOX_CLIP_RAW || ax <= -LSM6DSOX_CLIP_RAW) { s_clip_axis[0]++; railed = 1; }
    if (ay >=  LSM6DSOX_CLIP_RAW || ay <= -LSM6DSOX_CLIP_RAW) { s_clip_axis[1]++; railed = 1; }
    if (az >=  LSM6DSOX_CLIP_RAW || az <= -LSM6DSOX_CLIP_RAW) { s_clip_axis[2]++; railed = 1; }
    s_clip_samples += railed;

    /* Chip frame -> body frame (see IMU_REMAP in lsm6dsox.h), then
     * subtract the bias, which is stored in the body frame. */
    float cgx = (float)gx * LSM6DSOX_GYRO_SCALE;
    float cgy = (float)gy * LSM6DSOX_GYRO_SCALE;
    float cgz = (float)gz * LSM6DSOX_GYRO_SCALE;
    float cax = (float)ax * LSM6DSOX_ACCEL_SCALE;
    float cay = (float)ay * LSM6DSOX_ACCEL_SCALE;
    float caz = (float)az * LSM6DSOX_ACCEL_SCALE;

    float bgx, bgy, bgz, bax, bay, baz;
    IMU_REMAP(bgx, bgy, bgz, cgx, cgy, cgz);
    IMU_REMAP(bax, bay, baz, cax, cay, caz);

    out->gx = bgx - dev->gyro_bias.bx;
    out->gy = bgy - dev->gyro_bias.by;
    out->gz = bgz - dev->gyro_bias.bz;

    out->ax = bax;
    out->ay = bay;
    out->az = baz;

    return ESP_OK;
}
