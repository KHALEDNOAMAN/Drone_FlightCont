#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "mmc5603.h"
#include "i2c_all.h"     /* I2C_MASTER_FREQ_HZ - shared bus speed */
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MMC5603";

/* ---- Register Map ---- */
#define REG_XOUT0      0x00  /* X[19:12] */
#define REG_XOUT1      0x01  /* X[11:4]  */
#define REG_YOUT0      0x02  /* Y[19:12] */
#define REG_YOUT1      0x03  /* Y[11:4]  */
#define REG_ZOUT0      0x04  /* Z[19:12] */
#define REG_ZOUT1      0x05  /* Z[11:4]  */
#define REG_XOUT2      0x06  /* X[3:0] in bits [7:4] */
#define REG_YOUT2      0x07  /* Y[3:0] in bits [7:4] */
#define REG_ZOUT2      0x08  /* Z[3:0] in bits [7:4] */
#define REG_STATUS1    0x18
#define REG_ODR        0x1A
#define REG_CTRL0      0x1B
#define REG_CTRL1      0x1C
#define REG_CTRL2      0x1D
#define REG_PRODUCT_ID 0x39

/* CTRL0 bits */
#define CTRL0_TM_M        0x01  /* trigger one-shot magnetic measurement */
#define CTRL0_SET         0x08  /* one-shot SET pulse */
#define CTRL0_RESET       0x10  /* one-shot RESET pulse */
#define CTRL0_AUTO_SR     0x20  /* automatic SET/RESET around each measurement */
#define CTRL0_CMM_FREQ_EN 0x80  /* compute continuous-mode period from ODR (required) */

/* STATUS1 bits */
#define STATUS1_MEAS_M_DONE 0x40

/* CTRL2 bits */
#define CTRL2_CMM_EN   0x10  /* continuous measurement mode enable */

/* Scale: 20-bit output, 0.00625 uT/LSB (= 0.0625 mG/LSB, 16384 LSB/Gauss).
 * Zero-field offset = 2^19 = 524288 counts. */
#define MMC5603_SCALE     0.00625f
#define MMC5603_OFFSET    524288.0f

/* NVS keys */
#define NVS_NAMESPACE  "mmc5603"
/* blob: mmc5603_cal_t (off + soft-iron S).
 * Key BUMPED cal2 -> cal3 because the frame the calibration
 * is expressed in changed (see the rotation note in mmc5603_read). A cal2
 * blob applied in the cal3 frame would permute the axes while |B| still
 * looked perfect, so it must not silently load. Old key is left on flash;
 * a board with only cal2 falls back to identity and says so at boot. */
#define NVS_KEY_CAL    "cal3"

/* ---- I2C helpers ----
 * Bounded timeout: a wedged bus (slave holding SDA low) must cost a
 * bounded stall the fusion loop can ride through (falls back to 6-DOF),
 * not hang it forever. ~1 ms nominal (9-byte read at the 100 kHz bus);
 * 20 ms is generous. */
#define I2C_TIMEOUT_MS 20

static uint32_t s_i2c_errs = 0;

uint32_t mmc5603_i2c_err_count(void) { return s_i2c_errs; }

static void note_i2c_err(esp_err_t err)
{
    s_i2c_errs++;
    if (s_i2c_errs % 100 == 1) {   /* rate-limited: 1st, 101st, ... */
        ESP_LOGW(TAG, "I2C error #%lu: %s",
                 (unsigned long)s_i2c_errs, esp_err_to_name(err));
    }
}

static esp_err_t write_reg(mmc5603_handle_t *dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    esp_err_t err = i2c_master_transmit(dev->i2c_dev, buf, 2, I2C_TIMEOUT_MS);
    if (err != ESP_OK) note_i2c_err(err);
    return err;
}

static esp_err_t read_regs(mmc5603_handle_t *dev, uint8_t reg, uint8_t *data, size_t len)
{
    esp_err_t err = i2c_master_transmit_receive(dev->i2c_dev, &reg, 1,
                                                data, len, I2C_TIMEOUT_MS);
    if (err != ESP_OK) note_i2c_err(err);
    return err;
}

/* ---- SET/RESET degaussing (init-time only) ----
 * NOTE: writes to CTRL0 overwrite its config bits (AUTO_SR, CMM_FREQ_EN),
 * so this must only run BEFORE continuous mode is configured. While the
 * sensor runs, AUTO_SR handles degaussing automatically. */
static void do_set_reset(mmc5603_handle_t *dev)
{
    write_reg(dev, REG_CTRL0, CTRL0_SET);
    vTaskDelay(pdMS_TO_TICKS(1));
    write_reg(dev, REG_CTRL0, CTRL0_RESET);
    vTaskDelay(pdMS_TO_TICKS(1));
}

/* ---- Public API ---- */

mmc5603_handle_t *mmc5603_init(i2c_master_bus_handle_t bus_handle)
{
    mmc5603_handle_t *dev = calloc(1, sizeof(mmc5603_handle_t));
    if (!dev) return NULL;

    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = MMC5603_I2C_ADDR,
        .scl_speed_hz    = I2C_MASTER_FREQ_HZ,
    };
    if (i2c_master_bus_add_device(bus_handle, &cfg, &dev->i2c_dev) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add MMC5603 to I2C bus");
        free(dev);
        return NULL;
    }

    vTaskDelay(pdMS_TO_TICKS(20));

    /* Product ID check — informational only. Some MMC5603 units answer on
     * the bus and measure correctly but return 0x00 from the ID register
     * (seen on this board,). Health is decided by the functional
     * measurement check below, not the ID. */
    uint8_t id = 0;
    read_regs(dev, REG_PRODUCT_ID, &id, 1);
    if (id != MMC5603_PRODUCT_ID) {
        ESP_LOGW(TAG, "Product ID 0x%02X (expected 0x10) - checking functionally", id);
    } else {
        ESP_LOGI(TAG, "MMC5603 found (ID: 0x%02X)", id);
    }

    /* Software reset */
    write_reg(dev, REG_CTRL1, 0x80);
    vTaskDelay(pdMS_TO_TICKS(20));

    /* Functional check: trigger a one-shot measurement, require the
     * done flag. Catches a dead/absent chip regardless of ID quirks. */
    bool alive = false;
    if (write_reg(dev, REG_CTRL0, CTRL0_TM_M) == ESP_OK) {
        for (int i = 0; i < 25; i++) {
            vTaskDelay(pdMS_TO_TICKS(2));
            uint8_t st = 0;
            if (read_regs(dev, REG_STATUS1, &st, 1) == ESP_OK &&
                (st & STATUS1_MEAS_M_DONE)) {
                alive = true;
                break;
            }
        }
    }
    if (!alive) {
        ESP_LOGE(TAG, "Functional check failed: measurement never completed");
        i2c_master_bus_rm_device(dev->i2c_dev);
        free(dev);
        return NULL;
    }
    ESP_LOGI(TAG, "MMC5603 functional check passed");

    /* Initial SET/RESET (must happen before continuous mode config —
     * CTRL0 writes clobber config bits) */
    do_set_reset(dev);

    /* Configure continuous mode at 100 Hz:
     *   ODR   = 100
     *   CTRL0 = CMM_FREQ_EN | AUTO_SR — compute period from ODR (mandatory
     *           for continuous mode) + automatic SET/RESET degaussing
     *   CTRL2 = CMM_EN — enter continuous measurement mode
     */
    write_reg(dev, REG_ODR, 100);
    write_reg(dev, REG_CTRL0, CTRL0_CMM_FREQ_EN | CTRL0_AUTO_SR);
    write_reg(dev, REG_CTRL2, CTRL2_CMM_EN);

    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_LOGI(TAG, "MMC5603 configured: 100 Hz continuous");

    return dev;
}

esp_err_t mmc5603_read(mmc5603_handle_t *dev, mmc5603_data_t *out)
{
    /* Read 9 bytes: Xout0,Xout1,Yout0,Yout1,Zout0,Zout1,Xout2,Yout2,Zout2.
     * The low 4 bits of each 20-bit axis live in bits [7:4] of the
     * corresponding OUTn2 register. */
    uint8_t buf[9];
    esp_err_t err = read_regs(dev, REG_XOUT0, buf, 9);
    if (err != ESP_OK) return err;

    /* 20-bit output assembly */
    int32_t raw_x = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | (buf[6] >> 4);
    int32_t raw_y = ((int32_t)buf[2] << 12) | ((int32_t)buf[3] << 4) | (buf[7] >> 4);
    int32_t raw_z = ((int32_t)buf[4] << 12) | ((int32_t)buf[5] << 4) | (buf[8] >> 4);

    /* Convert to microtesla, rotate chip axes -> FC body axes (x-fwd,
     * z-up), THEN apply the full cal: out = S * (m_body - off).
     *
     * ORDER MATTERS AND IT WAS WRONG UNTIL. The rotation used
     * to sit AFTER the cal, which made the published vector
     * R*S*(m - off): the calibration was defined in the pre-rotation
     * frame while the stream everyone fits against is post-rotation. Any
     * recalibration captured from the D,/M, stream therefore produced
     * off and S that the driver then applied in the WRONG frame - and it
     * would have passed every check, because |B| is rotation-invariant,
     * so the sphere looks perfect while the axes are permuted and yaw
     * points somewhere else. Rotating first makes the calibration frame
     * and the published frame the same thing, which is what tools/mag_cal.py
     * has always assumed.
     *
     * The rotation is the composite of two separate findings:
     *
     * 1. Driver-level Y negation (bench/mag_frame_check.m):
     *    the rigid-body constraint dm/dt = -omega x m over a tumble ranks
     *    +/-diag(-1,+1,-1) first (residual 0.206 vs 1.538 for identity);
     *    the m/-m ambiguity is resolved by field inclination (Istanbul:
     *    field points into the ground, so mz < 0 when flat)
     *    -> diag(+1,-1,+1). det=-1: an axis sign convention, not mounting.
     *
     * 2. Board-mount 90-deg horizontal rotation, the same one
     *    as lsm6dsox.h IMU_REMAP - the mag is on that perfboard and turned
     *    with it: body_x = +m1_y, body_y = -m1_x, body_z = +m1_z.
     *
     * Composing (2) over (1) gives body = (-chip_y, -chip_x, +chip_z).
     * det is still -1, so nothing downstream of here changes handedness.
     *
     * If yaw stops tracking flat rotations, the mag did NOT rotate with
     * the IMU - re-run bench/mag_frame_check.m before touching anything. */
    float sx = ((float)raw_x - MMC5603_OFFSET) * MMC5603_SCALE;
    float sy = ((float)raw_y - MMC5603_OFFSET) * MMC5603_SCALE;
    float sz = ((float)raw_z - MMC5603_OFFSET) * MMC5603_SCALE;

    float rx = -sy - dev->cal.off[0];
    float ry = -sx - dev->cal.off[1];
    float rz =  sz - dev->cal.off[2];

    const float (*S)[3] = dev->cal.S;
    out->mx = S[0][0]*rx + S[0][1]*ry + S[0][2]*rz;
    out->my = S[1][0]*rx + S[1][1]*ry + S[1][2]*rz;
    out->mz = S[2][0]*rx + S[2][1]*ry + S[2][2]*rz;

    return ESP_OK;
}

/* Identity S, zero offset = raw pass-through */
static void cal_identity(mmc5603_cal_t *cal)
{
    memset(cal, 0, sizeof(*cal));
    cal->S[0][0] = cal->S[1][1] = cal->S[2][2] = 1.0f;
}

esp_err_t mmc5603_load_cal(mmc5603_handle_t *dev)
{
    cal_identity(&dev->cal);

    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "No mag calibration in NVS (raw pass-through)");
        return ESP_ERR_NOT_FOUND;
    }

    mmc5603_cal_t cal;
    size_t len = sizeof(cal);
    if (nvs_get_blob(nvs, NVS_KEY_CAL, &cal, &len) == ESP_OK &&
        len == sizeof(cal)) {
        dev->cal = cal;
        ESP_LOGI(TAG, "Loaded cal: off [%.2f %.2f %.2f] uT, "
                 "S diag [%.3f %.3f %.3f]",
                 (double)cal.off[0], (double)cal.off[1], (double)cal.off[2],
                 (double)cal.S[0][0], (double)cal.S[1][1], (double)cal.S[2][2]);
    } else {
        ESP_LOGW(TAG, "No mag calibration in NVS (raw pass-through)");
    }
    nvs_close(nvs);
    return ESP_OK;
}

esp_err_t mmc5603_save_cal(mmc5603_handle_t *dev, const float off[3], const float S[9])
{
    memcpy(dev->cal.off, off, sizeof(dev->cal.off));
    memcpy(dev->cal.S,   S,   sizeof(dev->cal.S));

    nvs_handle_t nvs;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;

    err = nvs_set_blob(nvs, NVS_KEY_CAL, &dev->cal, sizeof(dev->cal));
    if (err == ESP_OK) err = nvs_commit(nvs);
    nvs_close(nvs);

    ESP_LOGI(TAG, "Saved cal: off [%.2f %.2f %.2f] uT, S diag [%.3f %.3f %.3f]",
             (double)off[0], (double)off[1], (double)off[2],
             (double)S[0], (double)S[4], (double)S[8]);
    return err;
}
