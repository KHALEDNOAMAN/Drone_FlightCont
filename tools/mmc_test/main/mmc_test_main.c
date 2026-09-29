/*
 * MMC5603 standalone bring-up / health test — ESP32-S3, ESP-IDF
 *
 * Same wiring as flight firmware: SCL = GPIO4, SDA = GPIO5.
 *
 * What it does, in order:
 *   1. I2C bus scan            — is 0x30 (and DPS310 0x77) ACKing at all?
 *   2. Product ID read         — 0x39 must return 0x10
 *   3. SET/RESET diagnostic    — 10 rounds of paired measurements:
 *        after a SET pulse the sensing bridge measures  +B + offset
 *        after a RESET pulse it measures                -B + offset
 *        so  field  = (SET - RESET) / 2   (true external field)
 *            offset = (SET + RESET) / 2   (bridge/sensor offset)
 *      A healthy sensor: |field| = 25..65 uT, offset small and stable.
 *      Magnetized/saturated film or nearby magnet: huge field reading.
 *      Broken die: offset huge or values stuck / not flipping with SET/RESET.
 *   4. Continuous stream at 5 Hz with AUTO_SR, printing X Y Z |B|.
 *      Rotate the board slowly — X/Y should swing, |B| should stay constant.
 */

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

static const char *TAG = "MMC_TEST";

#define I2C_SCL_IO      4
#define I2C_SDA_IO      5
#define MMC_ADDR        0x30
#define MMC_PRODUCT_ID  0x10

/* Register map (same as components/mmc5603/mmc5603.c) */
#define REG_XOUT0       0x00
#define REG_STATUS1     0x18
#define REG_ODR         0x1A
#define REG_CTRL0       0x1B
#define REG_CTRL1       0x1C
#define REG_CTRL2       0x1D
#define REG_PRODUCT_ID  0x39

#define CTRL0_TM_M         0x01  /* trigger one-shot magnetic measurement */
#define CTRL0_SET          0x08
#define CTRL0_RESET        0x10
#define CTRL0_AUTO_SR      0x20
#define CTRL0_CMM_FREQ_EN  0x80
#define CTRL1_SW_RESET     0x80
#define CTRL2_CMM_EN       0x10
#define STATUS1_MEAS_M_DONE 0x40

#define SCALE_UT        0.00625f    /* 20-bit mode, uT per LSB */
#define ZERO_OFFSET     524288.0f   /* 2^19 counts = zero field */

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;

static esp_err_t wr(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(s_dev, buf, 2, 100);
}

static esp_err_t rd(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, data, len, 100);
}

/* ---------- 1. bus scan ---------- */
static void bus_scan(void)
{
    printf("\nI2C scan (SCL=%d SDA=%d):\n", I2C_SCL_IO, I2C_SDA_IO);
    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
    int found = 0;
    for (int i = 0; i < 128; i += 16) {
        printf("%02x: ", i);
        for (int j = 0; j < 16; j++) {
            int addr = i + j;
            if (addr < 3 || addr > 119) { printf("   "); continue; }
            if (i2c_master_probe(s_bus, addr, 50) == ESP_OK) {
                printf("%02x ", addr);
                found++;
            } else {
                printf("-- ");
            }
        }
        printf("\n");
    }
    printf("Found %d device(s). Expect 0x30 (MMC5603) and 0x77 (DPS310).\n\n", found);
}

/* ---------- one-shot measurement, raw counts ---------- */
static esp_err_t measure_raw(float *x, float *y, float *z)
{
    esp_err_t err = wr(REG_CTRL0, CTRL0_TM_M);
    if (err != ESP_OK) return err;

    /* poll Meas_m_done, ~10 ms max at default bandwidth */
    uint8_t st = 0;
    for (int i = 0; i < 50; i++) {
        vTaskDelay(pdMS_TO_TICKS(2));
        if (rd(REG_STATUS1, &st, 1) == ESP_OK && (st & STATUS1_MEAS_M_DONE)) break;
    }
    if (!(st & STATUS1_MEAS_M_DONE)) {
        ESP_LOGW(TAG, "measurement never completed (STATUS1=0x%02X)", st);
        return ESP_ERR_TIMEOUT;
    }

    uint8_t b[9];
    err = rd(REG_XOUT0, b, 9);
    if (err != ESP_OK) return err;

    int32_t rx = ((int32_t)b[0] << 12) | ((int32_t)b[1] << 4) | (b[6] >> 4);
    int32_t ry = ((int32_t)b[2] << 12) | ((int32_t)b[3] << 4) | (b[7] >> 4);
    int32_t rz = ((int32_t)b[4] << 12) | ((int32_t)b[5] << 4) | (b[8] >> 4);

    *x = ((float)rx - ZERO_OFFSET) * SCALE_UT;
    *y = ((float)ry - ZERO_OFFSET) * SCALE_UT;
    *z = ((float)rz - ZERO_OFFSET) * SCALE_UT;
    return ESP_OK;
}

/* ---------- 3. SET/RESET diagnostic ---------- */
static void set_reset_diagnostic(void)
{
    printf("=== SET/RESET diagnostic (10 rounds) ===\n");
    printf("field  = true external field (should be 25..65 uT total, stable)\n");
    printf("offset = sensor bridge offset (should be near 0, a few uT)\n\n");

    for (int round = 0; round < 10; round++) {
        float sx, sy, sz, rx, ry, rz;

        wr(REG_CTRL0, CTRL0_SET);
        vTaskDelay(pdMS_TO_TICKS(2));
        if (measure_raw(&sx, &sy, &sz) != ESP_OK) { printf("round %d: SET measure failed\n", round); continue; }

        wr(REG_CTRL0, CTRL0_RESET);
        vTaskDelay(pdMS_TO_TICKS(2));
        if (measure_raw(&rx, &ry, &rz) != ESP_OK) { printf("round %d: RESET measure failed\n", round); continue; }

        /* RESET reverses film polarity: field flips, offset doesn't */
        float fx = (sx - rx) * 0.5f, fy = (sy - ry) * 0.5f, fz = (sz - rz) * 0.5f;
        float ox = (sx + rx) * 0.5f, oy = (sy + ry) * 0.5f, oz = (sz + rz) * 0.5f;
        float fmag = sqrtf(fx*fx + fy*fy + fz*fz);
        float omag = sqrtf(ox*ox + oy*oy + oz*oz);

        printf("round %d: field [%7.2f %7.2f %7.2f] |%6.2f| uT   offset [%7.2f %7.2f %7.2f] |%6.2f| uT\n",
               round, fx, fy, fz, fmag, ox, oy, oz, omag);
    }

    printf("\nInterpretation:\n");
    printf("  |field| 25..65 uT, |offset| < ~10 uT  -> sensor healthy, environment clean\n");
    printf("  |field| >> 65 uT                       -> real external field: magnet/steel nearby, move the board\n");
    printf("  |offset| large but |field| sane        -> sensor magnetized; SET/RESET is compensating, cal will fix\n");
    printf("  field ~= 0 or values stuck             -> sensing film damaged, replace sensor\n\n");
}

void app_main(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = 0,
        .scl_io_num = I2C_SCL_IO,
        .sda_io_num = I2C_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_bus));

    bus_scan();

    /* 100 kHz — flight firmware uses 400 kHz, but with only internal pullups
     * (~45 kOhm) a marginal bus can ACK the address yet corrupt data reads.
     * Slow bus first; if everything passes here but fails at 400 kHz, the
     * fix is real pullup resistors (2.2-10 kOhm to 3V3), not a new sensor. */
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = MMC_ADDR,
        .scl_speed_hz    = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev));

    /* 2. product ID, read 5x — intermittent bus shows up as varying values */
    printf("Product ID (reg 0x39), 5 reads at 100 kHz: ");
    uint8_t id = 0;
    int id_ok = 0;
    for (int i = 0; i < 5; i++) {
        id = 0;
        esp_err_t err = rd(REG_PRODUCT_ID, &id, 1);
        printf("%s0x%02X ", (err != ESP_OK) ? "ERR/" : "", id);
        if (err == ESP_OK && id == MMC_PRODUCT_ID) id_ok++;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    printf(" [%d/5 correct, expected 0x10]\n\n", id_ok);

    /* register dump — what does the chip think its state is? */
    printf("Register dump:\n");
    const uint8_t dump_regs[] = { 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x39 };
    for (size_t i = 0; i < sizeof(dump_regs); i++) {
        uint8_t v = 0xEE;
        esp_err_t err = rd(dump_regs[i], &v, 1);
        printf("  reg 0x%02X = %s0x%02X\n", dump_regs[i], (err != ESP_OK) ? "ERR " : "", v);
    }
    printf("\n");

    if (id_ok == 0) {
        printf("ID never read 0x10 — continuing to SET/RESET diagnostic anyway for more data.\n\n");
    }

    /* software reset, then settle */
    wr(REG_CTRL1, CTRL1_SW_RESET);
    vTaskDelay(pdMS_TO_TICKS(30));

    set_reset_diagnostic();

    /* 4. continuous stream with AUTO_SR, 100 Hz internally, printed at 5 Hz */
    wr(REG_CTRL0, CTRL0_SET);
    vTaskDelay(pdMS_TO_TICKS(2));
    wr(REG_CTRL0, CTRL0_RESET);
    vTaskDelay(pdMS_TO_TICKS(2));
    wr(REG_ODR, 100);
    wr(REG_CTRL0, CTRL0_CMM_FREQ_EN | CTRL0_AUTO_SR);
    wr(REG_CTRL2, CTRL2_CMM_EN);
    vTaskDelay(pdMS_TO_TICKS(20));

    printf("=== Continuous stream (rotate board slowly, watch |B|) ===\n");
    while (1) {
        uint8_t b[9];
        if (rd(REG_XOUT0, b, 9) == ESP_OK) {
            int32_t rx = ((int32_t)b[0] << 12) | ((int32_t)b[1] << 4) | (b[6] >> 4);
            int32_t ry = ((int32_t)b[2] << 12) | ((int32_t)b[3] << 4) | (b[7] >> 4);
            int32_t rz = ((int32_t)b[4] << 12) | ((int32_t)b[5] << 4) | (b[8] >> 4);
            float x = ((float)rx - ZERO_OFFSET) * SCALE_UT;
            float y = ((float)ry - ZERO_OFFSET) * SCALE_UT;
            float z = ((float)rz - ZERO_OFFSET) * SCALE_UT;
            float mag = sqrtf(x*x + y*y + z*z);
            /* Heading in horizontal plane, board held flat. No tilt comp,
             * no hard-iron cal — expect a consistent-but-shifted heading:
             * rotating the board 90 deg should move it ~90 deg. */
            float heading = atan2f(y, x) * 180.0f / (float)M_PI;
            if (heading < 0) heading += 360.0f;
            printf("X %8.2f  Y %8.2f  Z %8.2f  |B| %7.2f uT  hdg %6.1f deg\n",
                   x, y, z, mag, heading);
        } else {
            printf("read failed\n");
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
