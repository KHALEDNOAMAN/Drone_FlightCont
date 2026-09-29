#include <math.h>
#include <stdlib.h>
#include "dps310.h"
#include "i2c_all.h"     /* I2C_MASTER_FREQ_HZ - shared bus speed */
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "DPS310";

/* ---- Register Map ---- */
#define REG_PRS_B2     0x00
#define REG_TMP_B2     0x03
#define REG_PRS_CFG    0x06
#define REG_TMP_CFG    0x07
#define REG_MEAS_CFG   0x08
#define REG_CFG_REG    0x09
#define REG_RESET      0x0C
#define REG_PRODUCT_ID 0x0D
#define REG_COEF       0x10  /* 18 bytes of calibration coefficients */
#define REG_TMP_COEF_SRC 0x28

/* Oversampling scale factors (from datasheet Table 5) */
static const float kp_table[] = {
    524288, 1572864, 3670016, 7864320,   /* 1x, 2x, 4x, 8x */
    253952, 516096,  1040384, 2088960    /* 16x, 32x, 64x, 128x */
};

/* ---- I2C helpers ----
 * Bounded timeout: a wedged bus (slave holding SDA low) must cost a
 * bounded stall the fusion loop can ride through, not hang it forever.
 * Largest transfer here (18-byte cal read) is ~1.8 ms at the 100 kHz
 * bus; 20 ms is still generous. */
#define I2C_TIMEOUT_MS 20

static uint32_t s_i2c_errs = 0;

uint32_t dps310_i2c_err_count(void) { return s_i2c_errs; }

static void note_i2c_err(esp_err_t err)
{
    s_i2c_errs++;
    if (s_i2c_errs % 100 == 1) {   /* rate-limited: 1st, 101st, ... */
        ESP_LOGW(TAG, "I2C error #%lu: %s",
                 (unsigned long)s_i2c_errs, esp_err_to_name(err));
    }
}

static esp_err_t write_reg(dps310_handle_t *dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    esp_err_t err = i2c_master_transmit(dev->i2c_dev, buf, 2, I2C_TIMEOUT_MS);
    if (err != ESP_OK) note_i2c_err(err);
    return err;
}

static esp_err_t read_regs(dps310_handle_t *dev, uint8_t reg, uint8_t *data, size_t len)
{
    esp_err_t err = i2c_master_transmit_receive(dev->i2c_dev, &reg, 1,
                                                data, len, I2C_TIMEOUT_MS);
    if (err != ESP_OK) note_i2c_err(err);
    return err;
}

/* ---- Twos-complement helper ---- */
static int32_t twos_complement(int32_t val, uint8_t bits)
{
    if (val & ((int32_t)1 << (bits - 1)))
        val -= (int32_t)1 << bits;
    return val;
}

/* ---- Read calibration coefficients ---- */
static esp_err_t read_calibration(dps310_handle_t *dev)
{
    uint8_t buf[18];
    esp_err_t err = read_regs(dev, REG_COEF, buf, 18);
    if (err != ESP_OK) return err;

    /* The transaction succeeding says nothing about the DATA. There is no
     * CRC on this block, so the only defence is rejecting the patterns a
     * failed read actually produces: a bus that returns nothing (all
     * 0x00) or one left floating high (all 0xFF). Neither is a legal
     * calibration - c00 and c10 are the dominant pressure terms and no
     * real part ships them at zero or at full scale.
     *
     * A net, not the fix. The fix is the MEAS_CFG check in dps310_init:
     * a chip that has not finished loading OTP returns coefficients that
     * are garbage WITHOUT being all one value, and this test cannot see
     * those. */
    bool all_zero = true, all_ones = true;
    for (int i = 0; i < 18; i++) {
        if (buf[i] != 0x00) all_zero = false;
        if (buf[i] != 0xFF) all_ones = false;
    }
    if (all_zero || all_ones) {
        ESP_LOGE(TAG, "coefficient block is all 0x%02X - not a calibration",
                 all_zero ? 0x00 : 0xFF);
        return ESP_ERR_INVALID_RESPONSE;
    }

    /* Parse coefficients per datasheet Section 8.11 */
    dev->c0  = (float)twos_complement(((int32_t)buf[0] << 4) | (buf[1] >> 4), 12);
    dev->c1  = (float)twos_complement(((int32_t)(buf[1] & 0x0F) << 8) | buf[2], 12);
    dev->c00 = (float)twos_complement(((int32_t)buf[3] << 12) | ((int32_t)buf[4] << 4) | (buf[5] >> 4), 20);
    dev->c10 = (float)twos_complement(((int32_t)(buf[5] & 0x0F) << 16) | ((int32_t)buf[6] << 8) | buf[7], 20);
    dev->c01 = (float)twos_complement(((int32_t)buf[8] << 8) | buf[9], 16);
    dev->c11 = (float)twos_complement(((int32_t)buf[10] << 8) | buf[11], 16);
    dev->c20 = (float)twos_complement(((int32_t)buf[12] << 8) | buf[13], 16);
    dev->c21 = (float)twos_complement(((int32_t)buf[14] << 8) | buf[15], 16);
    dev->c30 = (float)twos_complement(((int32_t)buf[16] << 8) | buf[17], 16);

    return ESP_OK;
}

/* ---- Public API ---- */

dps310_handle_t *dps310_init(i2c_master_bus_handle_t bus_handle)
{
    dps310_handle_t *dev = calloc(1, sizeof(dps310_handle_t));
    if (!dev) return NULL;

    /* Add to existing I2C bus */
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = DPS310_I2C_ADDR,
        .scl_speed_hz    = I2C_MASTER_FREQ_HZ,
    };
    if (i2c_master_bus_add_device(bus_handle, &cfg, &dev->i2c_dev) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add DPS310 to I2C bus");
        free(dev);
        return NULL;
    }

    vTaskDelay(pdMS_TO_TICKS(50));

    /* Product ID check */
    uint8_t id = 0;
    read_regs(dev, REG_PRODUCT_ID, &id, 1);
    if ((id & 0xF0) != DPS310_PRODUCT_ID) {
        ESP_LOGW(TAG, "Product ID: 0x%02X (expected 0x1x)", id);
    }
    ESP_LOGI(TAG, "DPS310 found (ID: 0x%02X)", id);

    /* Soft reset */
    write_reg(dev, REG_RESET, 0x89);
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Wait for coefficients and sensor ready */
    uint8_t meas_cfg = 0;
    for (int i = 0; i < 20; i++) {
        read_regs(dev, REG_MEAS_CFG, &meas_cfg, 1);
        if ((meas_cfg & 0xC0) == 0xC0) break;  /* COEF_RDY + SENSOR_RDY */
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    /* That loop can EXPIRE, and previously nothing checked whether
     * it had. A timed-out wait fell straight through into
     * read_calibration() and read the coefficient registers before the
     * chip had loaded them from OTP. read_calibration then returned
     * ESP_OK - because the I2C TRANSACTION had succeeded, and there is no
     * CRC on the block - so garbage passed as a healthy sensor and stayed
     * that way: 1578 C, 1157 hPa, -7126 m.
     *
     * The second half of the bug is why it was permanent. fusion arms its
     * 10 s baro_retry_task ONLY when dps310_init returns NULL. This path
     * "succeeded", so the retry never fired, and only a battery pull ever
     * revived the sensor. Returning NULL here is what lets the retry that
     * already exists self-heal the warm-reset case in seconds.
     *
     * read_regs's own return is ignored inside the loop on purpose: a
     * failed read leaves meas_cfg at its initialiser, which fails this
     * test too. Fails safe either way. */
    if ((meas_cfg & 0xC0) != 0xC0) {
        ESP_LOGE(TAG, "COEF_RDY/SENSOR_RDY never asserted (MEAS_CFG 0x%02X) "
                      "- refusing to read coefficients the chip has not "
                      "loaded", meas_cfg);
        i2c_master_bus_rm_device(dev->i2c_dev);
        free(dev);
        return NULL;
    }

    /* Read calibration */
    if (read_calibration(dev) != ESP_OK) {
        ESP_LOGE(TAG, "Calibration read failed");
        /* Hand the device slot back to the bus before freeing our own
         * struct. Without this, every failed init leaks both the heap
         * allocation inside i2c_master_bus_add_device and an entry on the
         * bus's device list - harmless when init ran once at boot, fatal
         * once fusion started retrying this every 10 s (fusion.c). */
        i2c_master_bus_rm_device(dev->i2c_dev);
        free(dev);
        return NULL;
    }

    /* Undocumented "temperature fix" (Infineon errata): some DPS310 units
     * report temperature ~60 C too high, which also corrupts the
     * temperature-compensated pressure. Magic sequence per Infineon app
     * support / Adafruit_DPS310 correctTemp(). */
    write_reg(dev, 0x0E, 0xA5);
    write_reg(dev, 0x0F, 0x96);
    write_reg(dev, 0x62, 0x02);
    write_reg(dev, 0x0E, 0x00);
    write_reg(dev, 0x0F, 0x00);

    /* Check temp coefficient source */
    uint8_t tmp_coef_src = 0;
    read_regs(dev, REG_TMP_COEF_SRC, &tmp_coef_src, 1);

    /* Configure (rate = bits [6:4] as 2^n Hz, oversampling = bits [3:0] as 2^n x):
     *   PRS_CFG  = 0x53  -> 32 Hz rate (101), 8x oversampling (0011) -> kP = kp_table[3]
     *   TMP_CFG  = 0x50 | ext bit -> 32 Hz, 1x oversample, coefficient source
     *   CFG_REG  = 0x00  -> no P_SHIFT/T_SHIFT (only required for >8x oversampling)
     *   MEAS_CFG = 0x07  -> Continuous pressure + temperature
     * Budget check: 32 * 14.8 ms (P, 8x) + 32 * 3.6 ms (T, 1x) = 589 ms/s < 1 s. OK.
     */
    write_reg(dev, REG_PRS_CFG, 0x53);
    write_reg(dev, REG_TMP_CFG, 0x50 | (tmp_coef_src & 0x80));
    write_reg(dev, REG_CFG_REG, 0x00);

    dev->kP = kp_table[3];  /* 8x oversampling */
    dev->kT = 524288.0f;    /* 1x oversampling */

    /* Start continuous measurement */
    write_reg(dev, REG_MEAS_CFG, 0x07);

    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_LOGI(TAG, "DPS310 configured: 32 Hz continuous");

    return dev;
}

esp_err_t dps310_read(dps310_handle_t *dev, float *temp_c, float *press_pa)
{
    uint8_t buf[6];
    esp_err_t err = read_regs(dev, REG_PRS_B2, buf, 6);
    if (err != ESP_OK) return err;

    /* Raw pressure (24-bit, two's complement) */
    int32_t raw_p = twos_complement(
        ((int32_t)buf[0] << 16) | ((int32_t)buf[1] << 8) | buf[2], 24);

    /* Raw temperature (24-bit, two's complement) */
    int32_t raw_t = twos_complement(
        ((int32_t)buf[3] << 16) | ((int32_t)buf[4] << 8) | buf[5], 24);

    /* Scaled values */
    float Traw_sc = (float)raw_t / dev->kT;
    float Praw_sc = (float)raw_p / dev->kP;

    /* Compensated temperature */
    *temp_c = dev->c0 * 0.5f + dev->c1 * Traw_sc;

    /* Compensated pressure */
    *press_pa = dev->c00
              + Praw_sc * (dev->c10 + Praw_sc * (dev->c20 + Praw_sc * dev->c30))
              + Traw_sc * dev->c01
              + Traw_sc * Praw_sc * (dev->c11 + Praw_sc * dev->c21);

    return ESP_OK;
}
