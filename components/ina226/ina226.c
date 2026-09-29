#include "ina226.h"
#include "i2c_all.h"     /* I2C_MASTER_FREQ_HZ - shared bus speed */
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "INA226";

#define INA226_ADDR    0x40
#define REG_CONFIG     0x00
#define REG_BUS_V      0x02
#define REG_MANUF_ID   0xFE
#define REG_DIE_ID     0xFF

#define MANUF_ID_TI    0x5449    /* "TI" */
#define DIE_ID_226     0x226     /* DIE_ID[15:4]; [3:0] is the die rev */

/* AVG=16, VBUSCT=VSHCT=1.1 ms, continuous shunt+bus -> ~35 ms/update
 * (datasheet SBOS547B config register; default 0x4127 with AVG bumped). */
#define CONFIG_VAL     0x4527

/* Bounded like the other bus users: a wedged bus must cost a bounded
 * stall, never a hang. */
#define I2C_TIMEOUT_MS 20

#define BUS_LSB_UV     1250      /* 1.25 mV per LSB */

/* A read that NAKs once on a 100 kHz bus shared with the baro and mag
 * is noise, not a dead pack sensor; blanking the dashboard on it would
 * make the reading look flaky. Blank only after a full second of
 * failures (5 samples at the 5 Hz sample rate). */
#define VBAT_FAIL_BLANK 5

static i2c_master_dev_handle_t s_dev = NULL;
static const char             *s_bus = "-";   /* bus it answered on */

/* Written by the sampling task, read by telemetry on another core. A
 * 16-bit aligned load/store is atomic on this CPU, so no lock: readers
 * see the old or the new value, never a torn one. */
static volatile uint16_t s_cached_mv   = 0;
static uint8_t           s_fail_streak = 0;

static esp_err_t read_reg16(i2c_master_dev_handle_t dev,
                            uint8_t reg, uint16_t *val)
{
    uint8_t buf[2];
    esp_err_t err = i2c_master_transmit_receive(dev, &reg, 1,
                                                buf, 2, I2C_TIMEOUT_MS);
    if (err == ESP_OK) *val = ((uint16_t)buf[0] << 8) | buf[1];
    return err;
}

static esp_err_t write_reg16(i2c_master_dev_handle_t dev,
                             uint8_t reg, uint16_t val)
{
    uint8_t buf[3] = { reg, (uint8_t)(val >> 8), (uint8_t)val };
    return i2c_master_transmit(dev, buf, 3, I2C_TIMEOUT_MS);
}

/* Identify and configure the chip on one bus. Quiet: the caller reports,
 * because a miss on the first bus is not a failure while the second one
 * is still to be tried. manuf/die are left as read, for that report. */
static bool try_bus(i2c_master_bus_handle_t bus, uint16_t *manuf, uint16_t *die)
{
    *manuf = *die = 0;
    if (!bus) return false;       /* bus B may fail without stopping boot */

    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = INA226_ADDR,
        .scl_speed_hz    = I2C_MASTER_FREQ_HZ,
    };
    i2c_master_dev_handle_t dev;
    if (i2c_master_bus_add_device(bus, &cfg, &dev) != ESP_OK) return false;

    if (read_reg16(dev, REG_MANUF_ID, manuf) != ESP_OK ||
        read_reg16(dev, REG_DIE_ID, die)     != ESP_OK ||
        *manuf != MANUF_ID_TI || (*die >> 4) != DIE_ID_226) {
        i2c_master_bus_rm_device(dev);
        return false;
    }
    if (write_reg16(dev, REG_CONFIG, CONFIG_VAL) != ESP_OK) {
        ESP_LOGW(TAG, "answered, but the config write failed");
        i2c_master_bus_rm_device(dev);
        return false;
    }
    s_dev = dev;
    return true;
}

bool ina226_init(i2c_master_bus_handle_t bus_b, i2c_master_bus_handle_t bus_a)
{
    uint16_t mb = 0, db = 0, ma = 0, da = 0;
    if (try_bus(bus_b, &mb, &db)) {
        s_bus = "B";
    } else if (try_bus(bus_a, &ma, &da)) {
        s_bus = "A";
    } else {
        ESP_LOGW(TAG, "not detected on bus B (manuf 0x%04X die 0x%04X) or "
                 "bus A (manuf 0x%04X die 0x%04X) - VBAT disabled",
                 mb, db, ma, da);
        return false;
    }

    /* First averaged conversion needs ~35 ms; wait so the boot line
     * shows a real number when the pack is in. */
    vTaskDelay(pdMS_TO_TICKS(50));
    uint32_t mv = 0;
    ina226_read_bus_mv(&mv);
    ESP_LOGI(TAG, "found on bus %s (die 0x%04X), vbus %u.%03u V", s_bus,
             s_bus[0] == 'B' ? db : da,
             (unsigned)(mv / 1000), (unsigned)(mv % 1000));
    return true;
}

const char *ina226_bus(void)
{
    return s_bus;
}

bool ina226_present(void)
{
    return s_dev != NULL;
}

esp_err_t ina226_read_bus_mv(uint32_t *mv)
{
    if (!s_dev) return ESP_ERR_INVALID_STATE;
    uint16_t raw = 0;
    esp_err_t err = read_reg16(s_dev, REG_BUS_V, &raw);
    if (err == ESP_OK) *mv = ((uint32_t)raw * BUS_LSB_UV) / 1000;
    return err;
}

void ina226_sample(void)
{
    if (!s_dev) return;

    uint32_t mv = 0;
    if (ina226_read_bus_mv(&mv) == ESP_OK) {
        s_fail_streak = 0;
        /* 36 V ceiling on the VBUS pin, so this cannot legitimately
         * exceed 36000. Clamped at 40000 (a little above that ceiling,
         * still inside uint16) rather than wrapping into a plausible
         * looking voltage. */
        s_cached_mv = (mv > 40000u) ? 40000u : (uint16_t)mv;
        return;
    }

    if (s_fail_streak < VBAT_FAIL_BLANK) s_fail_streak++;
    if (s_fail_streak >= VBAT_FAIL_BLANK) s_cached_mv = 0;
}

uint16_t ina226_cached_mv(void)
{
    return s_cached_mv;
}
