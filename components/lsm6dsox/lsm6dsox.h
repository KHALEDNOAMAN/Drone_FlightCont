#ifndef LSM6DSOX_H
#define LSM6DSOX_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/spi_master.h"

/* ---- Pin Configuration ---- */
/* Rewired (whole harness rebuilt). The module's own labels map
 * as SCX=SCK, SDX=MOSI, DO=MISO:
 *   SCK  17 -> 5     MOSI 18 -> 4
 *   MISO 15 -> 6     CS   16 -> 7      INT1 14 -> 15 (fusion.c)
 * 4-7 were bus A and bus B before this; both I2C buses moved to 13/14 and
 * 17/18. All four SPI lines route through the GPIO matrix either way -
 * SPI2's IOMUX pins are 9-13, which are the motors - and 10 MHz is far
 * below the matrix's limit. */
#define LSM6DSOX_SPI_HOST    SPI2_HOST
#define LSM6DSOX_PIN_MOSI    4
#define LSM6DSOX_PIN_MISO    6
#define LSM6DSOX_PIN_SCK     5
#define LSM6DSOX_PIN_CS      7
#define LSM6DSOX_SPI_CLK_HZ (10 * 1000 * 1000)  /* 10 MHz */

/* ---- Registers ---- */
#define LSM6DSOX_WHO_AM_I    0x0F
#define LSM6DSOX_WHO_AM_I_VAL 0x6C

#define LSM6DSOX_INT1_CTRL   0x0D  /* Interrupt routing -> INT1 pin */
#define LSM6DSOX_CTRL1_XL    0x10  /* Accel ODR + FS */
#define LSM6DSOX_CTRL2_G     0x11  /* Gyro  ODR + FS */
#define LSM6DSOX_CTRL3_C     0x12  /* Control reg 3 */
#define LSM6DSOX_CTRL4_C     0x13
#define LSM6DSOX_CTRL6_C     0x15  /* Gyro LPF */
#define LSM6DSOX_CTRL8_XL    0x17  /* Accel LPF */
#define LSM6DSOX_CTRL9_XL    0x18  /* DEN bits + I3C_disable */
#define LSM6DSOX_STATUS_REG  0x1E  /* TDA / GDA / XLDA */

/* ---- SPI-only interface  ----
 * CS HIGH does not mean "idle" to this chip: the datasheet pin table reads
 * "1: SPI idle mode / I2C / MIPI I3C communication enabled". So between
 * every SPI transfer an I2C slave AND an I3C slave listen on SCK and MOSI,
 * and the I3C one has no anti-spike filter (datasheet 5.3: the I2C filters
 * "are not compatible with I3C timing"). The S3 holds INT1 low through its
 * pull-down when the IMU powers up, which per 5.3 leaves BOTH enabled.
 *
 * ST's own example (lsm6dsox_read_data_polling.c) and Adafruit's driver do
 * the same two things right after the software reset: I3C_disable, then
 * BDU. This driver did neither previously, and datasheet 5.1.1 says
 * "(I2C_disable) = 1 must be written in CTRL4_C" to disable the I2C block.
 * Written right after the reset, read back before init returns.
 *
 *   CTRL9_XL = 0xE2  DEN_X|DEN_Y|DEN_Z (the reset default) | I3C_disable
 *   CTRL4_C  = 0x06  I2C_disable | LPF1_SEL_G (the gyro LPF1 it always had)
 *   CTRL3_C  = 0x44  BDU | IF_INC. BDU holds an axis's two bytes together,
 *                    so a burst cannot pair the LSB of one sample with the
 *                    MSB of the next. IF_INC must stay set: the 12-byte
 *                    burst depends on it. */
#define LSM6DSOX_CTRL9_XL_VAL 0xE2
#define LSM6DSOX_CTRL4_C_VAL  0x06
#define LSM6DSOX_CTRL3_C_VAL  0x44

#define LSM6DSOX_OUTX_L_G    0x22  /* Gyro data start */
#define LSM6DSOX_OUTX_L_A    0x28  /* Accel data start */

/* ---- Scaling ---- */
/* 8g  range: 0.244 mg/LSB  = 0.000244 g/LSB  (was 4g / 0.000122 until
 *            see the CTRL1_XL block in lsm6dsox.c for why) */
/* 500 dps:   17.50 mdps/LSB = 0.01750 dps/LSB */
#define LSM6DSOX_ACCEL_SCALE 0.000244f
#define LSM6DSOX_GYRO_SCALE  0.01750f

/* ---- Accelerometer rail watch ----
 * A raw axis at or near +-32767 is the ADC pinned at full scale, not a
 * measurement. Vibration that clips one tail rectifies into a DC offset
 * on the surviving tail's side, which is how a clean 1 g reading turns
 * into a fake sustained acceleration. 32000 of 32767 (97.7 % of FS)
 * catches samples that are effectively at the rail without waiting for
 * the exact code, which the on-chip LPF2 would round off anyway. */
#define LSM6DSOX_CLIP_RAW 32000

/* ---- Liveness (review, item 2) ----
 * SPI has no ACK. A read "succeeds" whatever the bus hands back, so an
 * ESP_OK from lsm6dsox_read used to say nothing about the DATA. incident 55
 *  is the proof: gyro bias -67.480, 67.480, -67.480 dps at
 * std 0.000 is raw 0xF0F0 on all three axes - every byte of the burst
 * read back as 0xF0 - and it was accepted as a calibration.
 *
 * Two checks on the raw burst, both impossible for a working chip:
 *   UNIFORM  all 12 bytes identical. Real axes at rest carry gravity on
 *            one of them and noise on all of them.
 *   FROZEN   the same 12 bytes more than LSM6DSOX_FROZEN_READS reads in a
 *            row. At-rest noise is a few LSB per axis, so even TWO equal
 *            bursts in a row are ~1e-6; a missed DRDY edge can repeat one
 *            sample, never sixteen.
 * Either one returns an error, which clears SENSOR_IMU_OK. fusion advances
 * the heartbeat only on a good sample, so the flight task's stale check
 * (MAX_STALE_LOOPS, 200 ms) then disarms instead of flying on the last
 * gyro reading forever. */
#define LSM6DSOX_FROZEN_READS 16

/* ---- Refusal diagnostics  ----
 * The freeze and the 2-3 frame IMU dropouts beside it
 * (s31: 39 in 14 min, every one self-recovered) are one signature at two
 * lengths: whole bursts of a single byte. What was missing was the chip's
 * state AT that moment. On the first refused burst of an episode, and again
 * LSM6DSOX_STICKY_READS refusals into the same episode, the driver reads
 * WHO_AM_I, INT1_CTRL, CTRL1_XL, CTRL2_G, CTRL3_C, CTRL4_C, CTRL9_XL and
 * STATUS_REG, plus the INT1 pin level, and keeps them for STATS.
 *
 * How to read one:
 *   every register = the stuck byte   the chip is not driving MISO at all
 *   WHO_AM_I 0x6C, config changed     it answers but was reset/rewritten
 *   WHO_AM_I 0x6C, config intact      it answers and the DATA path is bad
 *   INT1 pin HIGH                     it is still sampling; data is waiting
 *   INT1 pin LOW                      it stopped producing data
 * 100 refusals is ~1 s at the 10 ms DRDY-timeout pace a freeze runs at, so
 * a dropout that recovers never gets a second dump. */
#define LSM6DSOX_STICKY_READS 100

typedef struct {
    int64_t  t_us;       /* esp_timer time of the capture */
    uint32_t episode;    /* 1-based count since boot */
    uint8_t  kind;       /* 1 = UNIFORM burst, 2 = FROZEN burst */
    uint8_t  stuck;      /* first byte of the refused burst */
    int8_t   int1;       /* INT1 pin level, -1 = pin not known */
    esp_err_t spi_err;   /* ESP_OK unless a dump read itself failed */
    uint8_t  reg[8];     /* WHO_AM_I, INT1_CTRL, CTRL1_XL, CTRL2_G,
                            CTRL3_C, CTRL4_C, CTRL9_XL, STATUS_REG */
} lsm6dsox_dump_t;

typedef struct {
    uint32_t episodes;   /* refusal episodes since boot */
    uint32_t longest;    /* most refused reads in a row, any episode */
    uint32_t current;    /* refused reads in a row right now, 0 = healthy */
    uint8_t  cfg_bad;    /* init read-back: bit i = config register i did
                            not hold its value (see lsm6dsox_init) */
    bool     have_first, have_sticky;
    lsm6dsox_dump_t first;   /* first refusal of the LATEST episode */
    lsm6dsox_dump_t sticky;  /* same episode, STICKY_READS refusals in */
} lsm6dsox_diag_t;

/* A zero-rate offset bigger than this is not an offset. Datasheet G_TyOff
 * is +-1 dps typical (no max is given); 10 is ten times that. incident 55's
 * 67.48 dps is what this refuses. */
#define LSM6DSOX_BIAS_MAX_DPS 10.0f

/* ---- Board-mount axis remap  ----
 * The sensor perfboard was remounted rotated 90 deg in the horizontal
 * plane, so the chip axes stopped matching the airframe. Bench symptom:
 * nose-down tilt raised the LEFT motor pair (a roll response) instead of
 * the front pair - physical pitch was landing on the firmware roll axis.
 *
 * Direction derived from that observation: M2 (FL) rising means the
 * mixer received +roll, which means measured roll < 0, which means
 * ay < 0 during nose-down - so the chip +Y axis points downhill, i.e.
 * toward the NOSE. Right-handed with Z still up gives:
 *     body_x (fwd) = +chip_y
 *     body_y       = -chip_x
 *     body_z (up)  = +chip_z
 * This restores the pre-rebuild frame, so every sign downstream
 * (Madgwick, PID, mixer table - bench-verified) is unchanged.
 *
 * Applied identically to gyro and accel, in lsm6dsox_read AND in
 * lsm6dsox_calibrate_bias, so the stored gyro bias lives in the remapped
 * frame (bias is re-measured every boot - nothing stale survives). The
 * magnetometer gets the same rotation in mmc5603.c: it sits on the same
 * perfboard and rotated with it.
 *
 * RE-VERIFY BEFORE PROPS (armed, props off, hand-tilt):
 *   nose down      -> M1+M2 (front pair) rise; stream shows pitch move,
 *                     roll ~0
 *   left side down -> the left pair rises
 *   rotate flat 90 -> yaw tracks by ~90 deg (mag alignment check)
 * If pairs come up rotated the other way, the package rotation was the
 * opposite sense: flip both signs (body_x = -chip_y, body_y = +chip_x)
 * and re-test. */
#define IMU_REMAP(bx, by, bz, cx, cy, cz) \
    do { (bx) = (cy); (by) = -(cx); (bz) = (cz); } while (0)

/* ---- Data Container ---- */
typedef struct {
    float ax, ay, az;   /* g */
    float gx, gy, gz;   /* dps */
} lsm6dsox_data_t;

/* ---- Gyro Bias (from startup calibration) ---- */
typedef struct {
    float bx, by, bz;   /* dps */
} lsm6dsox_bias_t;

/* ---- Device Handle ---- */
typedef struct {
    spi_device_handle_t spi_dev;
    lsm6dsox_bias_t     gyro_bias;
    uint8_t             last_raw[12];  /* previous burst, for FROZEN */
    uint16_t            same_reads;    /* bursts in a row equal to it */
    uint32_t            refused_run;   /* refused bursts in a row */
    int                 int1_gpio;     /* for the dump; -1 until the caller
                                          sets it (fusion.c owns the pin) */
} lsm6dsox_handle_t;

/**
 * @brief Init SPI bus + LSM6DSOX. Performs WHO_AM_I check and gyro bias calibration.
 * @return Heap-allocated handle, or NULL on failure.
 */
lsm6dsox_handle_t *lsm6dsox_init(void);

/**
 * @brief Read accel + gyro with bias subtraction on gyro.
 * @return ESP_OK with *out filled, or an error with *out untouched:
 *         the SPI error, ESP_ERR_INVALID_RESPONSE (UNIFORM burst) or
 *         ESP_ERR_INVALID_STATE (FROZEN) - see LSM6DSOX_FROZEN_READS.
 */
esp_err_t lsm6dsox_read(lsm6dsox_handle_t *dev, lsm6dsox_data_t *out);

/**
 * @brief (Re)calibrate gyro bias: 2 s average, motion-gated with retries.
 * Board must be still. Blocks the calling task ~2-6 s.
 * @return ESP_OK on a clean capture, ESP_ERR_INVALID_STATE if the final
 *         attempt was still noisy (bias stored anyway, logged loudly),
 *         ESP_ERR_INVALID_RESPONSE if the capture was REFUSED - an axis
 *         that never changed (frozen/garbage bus) or an offset above
 *         LSM6DSOX_BIAS_MAX_DPS. A refused capture is NOT stored; the
 *         previous bias stays.
 */
esp_err_t lsm6dsox_calibrate_bias(lsm6dsox_handle_t *dev);

/**
 * @brief Bursts lsm6dsox_read has refused since boot as not data:
 *        UNIFORM and FROZEN (see LSM6DSOX_FROZEN_READS). Both stay 0 on
 *        a healthy IMU. Either pointer may be NULL.
 */
void lsm6dsox_liveness_counts(uint32_t *uniform, uint32_t *frozen);

/**
 * @brief Copy of the refusal diagnostics (see LSM6DSOX_STICKY_READS).
 *        Written by the fusion task, read by a console command, unlocked:
 *        a copy taken during a capture can mix two dumps. It is for a
 *        human, and a capture happens at most twice per episode.
 */
void lsm6dsox_diag_get(lsm6dsox_diag_t *out);

/**
 * @brief Accel samples since boot/reset whose raw count reached
 *        LSM6DSOX_CLIP_RAW on ANY axis. Counted once per sample.
 *        Meaningless on its own - divide by lsm6dsox_accel_sample_count().
 */
uint32_t lsm6dsox_clip_count(void);

/**
 * @brief Accel samples read since boot/reset: the denominator for
 *        lsm6dsox_clip_count(). A bare clip count is not a rate.
 */
uint32_t lsm6dsox_accel_sample_count(void);

/**
 * @brief Per-axis rail hits, CHIP frame x,y,z (not the IMU_REMAP body
 *        frame - clipping happens in the ADC, before the remap).
 */
void lsm6dsox_clip_axis_counts(uint32_t out[3]);

#endif
