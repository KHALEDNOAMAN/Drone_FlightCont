/* MS5611-01BA03 driver. See ms5611.h for why this part is on the board and
 * for the address warning. The arithmetic lives in ms5611_calc.c, which is
 * host-tested by tools/ms5611_test.py. */

#include <stdlib.h>
#include "ms5611.h"
#include "i2c_all.h"     /* I2C_MASTER_FREQ_HZ - shared bus speed */
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MS5611";

/* ---- Command set ---- */
#define CMD_RESET      0x1E
#define CMD_CONV_D1    0x40   /* pressure,    OR in the OSR bits */
#define CMD_CONV_D2    0x50   /* temperature, OR in the OSR bits */
#define CMD_ADC_READ   0x00
#define CMD_PROM_READ  0xA0   /* 0xA0 + (word << 1) */

/* Bounded timeout, same reasoning as the DPS310 driver: a wedged bus must
 * cost a stall the caller can ride through, not a hang. The longest
 * transfer here is 3 bytes. */
#define I2C_TIMEOUT_MS 20

/* Conversion time by OSR, from the datasheet's max column, rounded up to
 * whole milliseconds and then given one extra. Reading the ADC before the
 * conversion finishes returns 0 - a silent, plausible-looking zero rather
 * than an error, which is exactly the failure mode this project keeps
 * getting bitten by. The extra millisecond is deliberate. */
static uint32_t conv_ms(ms5611_osr_t osr)
{
    switch (osr) {
    case MS5611_OSR_256:  return 2;
    case MS5611_OSR_512:  return 3;
    case MS5611_OSR_1024: return 4;
    case MS5611_OSR_2048: return 6;
    case MS5611_OSR_4096: default: return 10;
    }
}

static uint32_t s_i2c_errs = 0;
static uint32_t s_rejects = 0;

uint32_t ms5611_i2c_err_count(void) { return s_i2c_errs; }
uint32_t ms5611_reject_count(void) { return s_rejects; }

static void note_i2c_err(esp_err_t err)
{
    s_i2c_errs++;
    if (s_i2c_errs % 100 == 1) {   /* rate-limited: 1st, 101st, ... */
        ESP_LOGW(TAG, "I2C error #%lu: %s",
                 (unsigned long)s_i2c_errs, esp_err_to_name(err));
    }
}

static esp_err_t send_cmd(ms5611_handle_t *dev, uint8_t cmd)
{
    esp_err_t err = i2c_master_transmit(dev->i2c_dev, &cmd, 1, I2C_TIMEOUT_MS);
    if (err != ESP_OK) note_i2c_err(err);
    return err;
}

static esp_err_t read_prom_word(ms5611_handle_t *dev, uint8_t word,
                                uint16_t *out)
{
    uint8_t cmd = (uint8_t)(CMD_PROM_READ + (word << 1));
    uint8_t buf[2] = { 0, 0 };
    esp_err_t err = i2c_master_transmit_receive(dev->i2c_dev, &cmd, 1,
                                                buf, 2, I2C_TIMEOUT_MS);
    if (err != ESP_OK) { note_i2c_err(err); return err; }
    *out = (uint16_t)(((uint16_t)buf[0] << 8) | buf[1]);
    return ESP_OK;
}

/* Kick off a conversion and record when it will be ready. Does not wait. */
static esp_err_t start_conv(ms5611_handle_t *dev, uint8_t base)
{
    esp_err_t err = send_cmd(dev, (uint8_t)(base | (uint8_t)dev->osr));
    if (err != ESP_OK) return err;
    dev->conv_due_us = esp_timer_get_time() +
                       (int64_t)conv_ms(dev->osr) * 1000;
    return ESP_OK;
}

/* Read the 24-bit ADC result of a conversion already known to be done. */
static esp_err_t read_adc(ms5611_handle_t *dev, uint32_t *out)
{
    uint8_t cmd = CMD_ADC_READ;
    uint8_t buf[3] = { 0, 0, 0 };
    esp_err_t err = i2c_master_transmit_receive(dev->i2c_dev, &cmd, 1, buf, 3,
                                                I2C_TIMEOUT_MS);
    if (err != ESP_OK) { note_i2c_err(err); return err; }

    const uint32_t v = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) |
                       (uint32_t)buf[2];
    /* A zero ADC result means the conversion had not finished - the part
     * answers the read regardless. Treat it as a failed read rather than
     * feeding 0 into the compensation, where it produces a large, smooth,
     * entirely fictional pressure. */
    if (v == 0) return ESP_ERR_INVALID_RESPONSE;

    *out = v;
    return ESP_OK;
}

/* Blocking convert, used ONLY at init where there is no loop to starve. */
static esp_err_t convert_blocking(ms5611_handle_t *dev, uint8_t base,
                                  uint32_t *out)
{
    esp_err_t err = start_conv(dev, base);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(conv_ms(dev->osr)));
    return read_adc(dev, out);
}

/* One address attempt. Returns a ready handle or NULL, and is quiet about
 * a plain no-ACK: with CSB free to be either level (see ms5611.h) exactly
 * one of the two addresses is EXPECTED to stay silent, so logging that at
 * ERROR would print a scary line on every healthy boot. */
static ms5611_handle_t *try_addr(i2c_master_bus_handle_t bus_handle,
                                 ms5611_osr_t osr, uint8_t addr)
{
    ms5611_handle_t *dev = calloc(1, sizeof(ms5611_handle_t));
    if (!dev) {
        ESP_LOGE(TAG, "alloc failed");
        return NULL;
    }
    dev->osr = osr;
    dev->t_divider = 10;   /* refresh temperature every 10th read */
    dev->t_count = 0;
    dev->ready = false;
    dev->addr = addr;

    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = I2C_MASTER_FREQ_HZ,
    };
    if (i2c_master_bus_add_device(bus_handle, &cfg, &dev->i2c_dev) != ESP_OK) {
        ESP_LOGE(TAG, "bus_add_device failed for 0x%02X", addr);
        free(dev);
        return NULL;
    }

    /* Reset, then the datasheet's 2.8 ms reload time, generously rounded. */
    if (send_cmd(dev, CMD_RESET) != ESP_OK) {
        ESP_LOGD(TAG, "no ACK at 0x%02X", addr);
        goto fail;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    uint16_t prom[MS5611_PROM_WORDS];
    for (int i = 0; i < MS5611_PROM_WORDS; i++) {
        if (read_prom_word(dev, (uint8_t)i, &prom[i]) != ESP_OK) {
            ESP_LOGW(TAG, "0x%02X ACKed but PROM word %d is unreadable", addr, i);
            goto fail;
        }
    }

    /* THE CHECK THE DPS310 COULD NOT OFFER. incident 18: the DPS310 read its
     * coefficients over a healthy transfer and they were garbage, and
     * nothing caught it because a successful transaction says nothing
     * about the data. This part carries a CRC over its calibration, so
     * refuse to run without it.
     *
     * It is also what makes probing two addresses safe: whatever else
     * might answer at 0x76 or 0x77 cannot forge a valid MS5611 PROM. */
    if (!ms5611_prom_crc_ok(prom)) {
        ESP_LOGW(TAG, "0x%02X PROM CRC FAILED - not an MS5611, or its "
                      "calibration is corrupt. Raw: %04X %04X %04X %04X "
                      "%04X %04X %04X %04X", addr,
                 prom[0], prom[1], prom[2], prom[3],
                 prom[4], prom[5], prom[6], prom[7]);
        goto fail;
    }

    ms5611_cal_from_prom(&dev->cal, prom);

    /* An all-zero PROM has a valid CRC of zero, so the CRC alone does not
     * prove the part is calibrated - only that we read it correctly. */
    if (!ms5611_cal_plausible(&dev->cal)) {
        ESP_LOGW(TAG, "0x%02X passed CRC but the coefficients are not "
                      "calibration data (C1..C6 = %u %u %u %u %u %u)", addr,
                 dev->cal.c1, dev->cal.c2, dev->cal.c3,
                 dev->cal.c4, dev->cal.c5, dev->cal.c6);
        goto fail;
    }

    /* Prime the temperature cache so the first pressure read is compensated
     * with a real D2 rather than a zero. */
    if (convert_blocking(dev, CMD_CONV_D2, &dev->last_d2) != ESP_OK) {
        ESP_LOGW(TAG, "0x%02X first temperature conversion failed", addr);
        goto fail;
    }

    dev->ready = true;
    return dev;

fail:
    i2c_master_bus_rm_device(dev->i2c_dev);
    free(dev);
    return NULL;
}

ms5611_handle_t *ms5611_init(i2c_master_bus_handle_t bus_handle,
                             ms5611_osr_t osr)
{
    /* Bus B is allowed to fail to come up (i2c_all.c), and a control baro
     * on a dead bus is "not fitted", not a fault. */
    if (!bus_handle) return NULL;

    /* CSB HIGH first only because 0x76 is what a deliberately-modified
     * board reads as; an unmodified breakout answers at 0x77 and costs one
     * extra no-ACK. Both are free on bus B - see ms5611.h. */
    static const uint8_t addrs[] = { MS5611_ADDR_CSB_HIGH, MS5611_ADDR_CSB_LOW };

    for (size_t i = 0; i < sizeof(addrs) / sizeof(addrs[0]); i++) {
        ms5611_handle_t *dev = try_addr(bus_handle, osr, addrs[i]);
        if (dev) {
            ESP_LOGI(TAG, "ready at 0x%02X (CSB %s), PROM CRC ok, "
                          "C1..C6 = %u %u %u %u %u %u",
                     dev->addr,
                     dev->addr == MS5611_ADDR_CSB_HIGH ? "high" : "low",
                     dev->cal.c1, dev->cal.c2, dev->cal.c3,
                     dev->cal.c4, dev->cal.c5, dev->cal.c6);
            return dev;
        }
    }

    ESP_LOGI(TAG, "no MS5611 on bus B at 0x%02X or 0x%02X",
             MS5611_ADDR_CSB_HIGH, MS5611_ADDR_CSB_LOW);
    return NULL;
}

esp_err_t ms5611_poll(ms5611_handle_t *dev, float *temp_c, float *press_pa)
{
    if (!dev || !dev->ready) return ESP_ERR_INVALID_STATE;

    /* Nothing in flight: start one. Temperature on the divider, pressure
     * otherwise - the phase we start decides what the next completion
     * means. */
    if (dev->phase == MS5611_PH_IDLE) {
        const bool want_temp = (dev->t_count == 0);
        if (++dev->t_count >= dev->t_divider) dev->t_count = 0;

        esp_err_t err = start_conv(dev, want_temp ? CMD_CONV_D2 : CMD_CONV_D1);
        if (err != ESP_OK) return err;
        dev->phase = want_temp ? MS5611_PH_WAIT_D2 : MS5611_PH_WAIT_D1;
        return ESP_ERR_NOT_FINISHED;
    }

    /* Still converting. Reading the ADC early returns 0, which the
     * compensation would happily turn into a confident wrong pressure, so
     * the clock is the gate. */
    if (esp_timer_get_time() < dev->conv_due_us) return ESP_ERR_NOT_FINISHED;

    uint32_t raw = 0;
    esp_err_t err = read_adc(dev, &raw);
    if (err != ESP_OK) {
        dev->phase = MS5611_PH_IDLE;   /* resync rather than wedge */
        return err;
    }

    if (dev->phase == MS5611_PH_WAIT_D2) {
        /* Temperature only refreshes the cache; the pressure conversion
         * that pairs with it starts now and completes next call. */
        dev->last_d2 = raw;
        err = start_conv(dev, CMD_CONV_D1);
        if (err != ESP_OK) { dev->phase = MS5611_PH_IDLE; return err; }
        dev->phase = MS5611_PH_WAIT_D1;
        return ESP_ERR_NOT_FINISHED;
    }

    dev->phase = MS5611_PH_IDLE;

    int32_t t_c100 = 0, p_pa = 0;
    ms5611_compensate(&dev->cal, raw, dev->last_d2, &t_c100, &p_pa);

    /* Reject, never clamp. A reading outside the part's own specified
     * range is not a measurement to be squeezed into a legal value - it
     * means something upstream is wrong, and the caller needs to know that
     * rather than receive a tidy lie. */
    if (!ms5611_reading_plausible(t_c100, p_pa)) {
        s_rejects++;
        if (s_rejects % 100 == 1) {
            ESP_LOGW(TAG, "implausible reading #%lu rejected: %ld Pa, "
                          "%ld.%02ld C (D1=%lu D2=%lu)",
                     (unsigned long)s_rejects, (long)p_pa,
                     (long)(t_c100 / 100), (long)(t_c100 % 100 < 0 ?
                         -(t_c100 % 100) : t_c100 % 100),
                     (unsigned long)raw, (unsigned long)dev->last_d2);
        }
        return ESP_ERR_INVALID_RESPONSE;
    }

    if (temp_c)   *temp_c   = (float)t_c100 * 0.01f;
    if (press_pa) *press_pa = (float)p_pa;
    return ESP_OK;
}
