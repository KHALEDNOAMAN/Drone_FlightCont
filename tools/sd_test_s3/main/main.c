/* Bench test for a microSD module on its own, before it goes on the
 * aircraft - above all, whether the pin-13 rework (74VHCT125A OE4, pin 13,
 * lifted off ground and tied to the buffered CS, pin 8) really releases
 * MISO, which is why the module needs its MISO-buffer rework.
 *
 * WHAT IT CHECKS, in order:
 *   1. MISO released with CS high, before any SPI traffic. Tested with the
 *      pad's own ~45k pulls: a line nothing drives follows the pull-down AND
 *      the pull-up; a driven line follows neither.
 *   2. MISO driven with CS low - the buffer must still switch ON when the
 *      card is selected, or the card can never answer.
 *   3. Card init through ESP-IDF's sdspi driver at 10 MHz: the same driver,
 *      clock and pull-up as the blackbox (blackbox.c card_probe).
 *   4. MISO still released once the card is in SPI mode.
 *   5. Write, read back, restore - on the card's LAST 64 sectors, whose
 *      contents are read first and written back after, so the card ends up
 *      unchanged. (The blackbox ring starts at the front of the card.)
 *   6. The pull-up. IDF reads MISO with CS HIGH before every command
 *      (sdspi_host.c:477, poll_busy: up to 40 ms waiting for non-zero
 *      bytes). On a released line that reads whatever the pad is pulled to,
 *      so with the pull-DOWN each command must stall ~40 ms. That proves MISO
 *      really floats between commands, and that blackbox.c's pull-up is what
 *      keeps the card fast.
 *
 * WIRING, S3 alone on a breadboard:
 *   module CS -> GPIO4, SCK -> GPIO5, MOSI -> GPIO6, MISO -> GPIO7,
 *   module VCC -> the S3's 5V pin, GND -> GND.
 *   On a DevKitC-1 v1.1 the 5V pin is USB VBUS through a 1N5819 Schottky
 *   (Espressif schematic, sheet 2, D1/D7), so on USB power it OUTPUTS about
 *   4.6-4.7 V - enough for the module's AMS1117-3.3. Meter it on your board.
 *   With the module powered, meter the buffer's pin 14 (VCC): ~3.3 V. At
 *   5 V it would drive 5 V into GPIO7 - do not connect MISO in that case.
 *
 * RUN:  idf.py -p COMx flash monitor      (the EN button runs it again)
 *       Same partition table as the flight firmware, so only the app slot is
 *       rewritten and NVS (params, level and mag cal) survives. */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_rom_sys.h"

#define PIN_CS        4
#define PIN_SCK       5
#define PIN_MOSI      6
#define PIN_MISO      7
#define SD_HOST       SPI2_HOST
#define SD_FREQ_KHZ   10000   /* BB_SD_FREQ_KHZ, blackbox.h */
#define SECTOR        512
#define TEST_SECTORS  64      /* write / verify / restore block at the card's end */
#define CHUNK         8       /* 8 x 512 B = one 4 KB blackbox write */
#define STALL_READS   20

static int s_pass, s_fail;

static void result(bool ok, const char *what, const char *detail)
{
    printf("  [%s] %s%s%s\n", ok ? "PASS" : "FAIL", what,
           (detail && *detail) ? "  - " : "", detail ? detail : "");
    if (ok) s_pass++;
    else    s_fail++;
}

typedef enum { M_RELEASED, M_HELD_HIGH, M_HELD_LOW, M_UNSTABLE } miso_t;
static const char *MISO_NAME[] = { "released", "HELD HIGH", "HELD LOW",
                                   "unstable" };

/* Pull the pad down, then up, eight times each. Leaves the pull-up on,
 * which is the aircraft's configuration (blackbox.c). */
static miso_t miso_probe(char *detail, size_t n)
{
    int lo = 0, hi = 0;
    for (int i = 0; i < 8; i++) {
        gpio_pullup_dis(PIN_MISO);
        gpio_pulldown_en(PIN_MISO);
        esp_rom_delay_us(500);
        lo += gpio_get_level(PIN_MISO);
        gpio_pulldown_dis(PIN_MISO);
        gpio_pullup_en(PIN_MISO);
        esp_rom_delay_us(500);
        hi += gpio_get_level(PIN_MISO);
    }
    miso_t m = (lo == 0 && hi == 8) ? M_RELEASED
             : (lo == 8 && hi == 8) ? M_HELD_HIGH
             : (lo == 0 && hi == 0) ? M_HELD_LOW : M_UNSTABLE;
    snprintf(detail, n, "%s (reads high: %d/8 pulled down, %d/8 pulled up)",
             MISO_NAME[m], lo, hi);
    return m;
}

static void out_pin(int pin, int level)
{
    gpio_set_level(pin, level);                 /* level before direction */
    const gpio_config_t io = { .pin_bit_mask = 1ULL << pin,
                               .mode = GPIO_MODE_OUTPUT };
    gpio_config(&io);
    gpio_set_level(pin, level);
}

static double ms(int64_t us) { return us / 1000.0; }

static void run(void)
{
    char d[160];

    printf("\n=== SD module bench test ===\n");
    printf("CS %d  SCK %d  MOSI %d  MISO %d  (SPI2, %d kHz)\n",
           PIN_CS, PIN_SCK, PIN_MOSI, PIN_MISO, SD_FREQ_KHZ);
    printf("Before trusting any of this: module VCC ~4.7 V from the 5V pin, "
           "buffer pin 14 ~3.3 V.\n\n");

    /* ---- 1-2: the buffer, before the card sees a single clock ----
     * SCK low and MOSI high are the SPI mode-0 idle levels; driving them
     * keeps a card still in SD mode from seeing noise as clocks. */
    out_pin(PIN_CS, 1);
    out_pin(PIN_SCK, 0);
    out_pin(PIN_MOSI, 1);
    const gpio_config_t mi = { .pin_bit_mask = 1ULL << PIN_MISO,
                               .mode = GPIO_MODE_INPUT,
                               .pull_up_en = GPIO_PULLUP_ENABLE };
    gpio_config(&mi);
    vTaskDelay(pdMS_TO_TICKS(50));

    printf("1-2. The MISO buffer follows CS\n");
    miso_t m = miso_probe(d, sizeof d);
    result(m == M_RELEASED, "MISO released with CS high", d);
    if (m == M_HELD_HIGH || m == M_HELD_LOW)
        printf("       -> the module still drives MISO: pin 13 is still on "
               "ground, or the lifted pin touches its pad.\n");
    else if (m == M_UNSTABLE)
        printf("       -> pin 13 may be floating: check the wire to pin 8.\n");

    gpio_set_level(PIN_CS, 0);
    esp_rom_delay_us(50);
    m = miso_probe(d, sizeof d);
    gpio_set_level(PIN_CS, 1);
    result(m == M_HELD_HIGH || m == M_HELD_LOW,
           "MISO driven with CS low (buffer switches on)", d);
    if (m == M_RELEASED)
        printf("       -> the buffer never turns on: pin 13 is not reaching "
               "pin 8 (the buffered CS).\n");

    /* ---- 3: card init, exactly as blackbox.c does it ---- */
    printf("\n3. Card init\n");
    const spi_bus_config_t bus = {
        .mosi_io_num = PIN_MOSI, .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_SCK,
        .quadwp_io_num = -1, .quadhd_io_num = -1,
        .max_transfer_sz = TEST_SECTORS * SECTOR + 64,
    };
    esp_err_t err = spi_bus_initialize(SD_HOST, &bus, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        result(false, "SPI bus init", esp_err_to_name(err));
        return;
    }
    gpio_pullup_en(PIN_MISO);                   /* blackbox.c:189 */
    err = sdspi_host_init();
    if (err != ESP_OK) {
        result(false, "sdspi host init", esp_err_to_name(err));
        return;
    }
    sdspi_device_config_t dev = SDSPI_DEVICE_CONFIG_DEFAULT();
    dev.host_id = SD_HOST;
    dev.gpio_cs = PIN_CS;
    sdspi_dev_handle_t handle;
    err = sdspi_host_init_device(&dev, &handle);
    if (err != ESP_OK) {
        result(false, "sdspi device init", esp_err_to_name(err));
        return;
    }
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = handle;
    host.max_freq_khz = SD_FREQ_KHZ;
    static sdmmc_card_t card;
    int64_t t0 = esp_timer_get_time();
    err = sdmmc_card_init(&host, &card);
    snprintf(d, sizeof d, "%s, %.0f ms", esp_err_to_name(err),
             ms(esp_timer_get_time() - t0));
    result(err == ESP_OK, "card answers and initialises", d);
    if (err != ESP_OK) {
        printf("       -> no card, VCC below ~4.5 V, a swapped SPI line, or "
               "MISO never driven (see 1-2).\n");
        return;
    }
    sdmmc_card_print_info(stdout, &card);
    snprintf(d, sizeof d, "%d B", card.csd.sector_size);
    result(card.csd.sector_size == SECTOR, "512 B sectors", d);

    /* ---- 4: released again, now that the card is in SPI mode ---- */
    printf("\n4. MISO after init\n");
    m = miso_probe(d, sizeof d);
    result(m == M_RELEASED, "MISO released with CS high, card in SPI mode", d);

    /* ---- 5: write / verify / restore at the card's end ---- */
    printf("\n5. Write, read back, restore (last %d sectors)\n", TEST_SECTORS);
    const size_t bytes = TEST_SECTORS * SECTOR;
    const uint32_t start = (uint32_t)card.csd.capacity - TEST_SECTORS;
    uint8_t *orig = heap_caps_malloc(bytes, MALLOC_CAP_DMA);
    uint8_t *pat  = heap_caps_malloc(bytes, MALLOC_CAP_DMA);
    uint8_t *back = heap_caps_malloc(bytes, MALLOC_CAP_DMA);
    if (!orig || !pat || !back) {
        result(false, "buffers", "out of DMA memory");
        return;
    }
    t0 = esp_timer_get_time();
    err = sdmmc_read_sectors(&card, orig, start, TEST_SECTORS);
    int64_t dt = esp_timer_get_time() - t0;
    snprintf(d, sizeof d, "sector %" PRIu32 "..%" PRIu32 ", %.0f KB/s",
             start, start + TEST_SECTORS - 1, bytes / 1024.0 / (dt / 1e6));
    result(err == ESP_OK, "read the originals", d);
    if (err != ESP_OK) {
        printf("       -> not writing anything: the originals could not be "
               "saved first.\n");
        return;
    }

    for (size_t i = 0; i < bytes; i++)
        pat[i] = (uint8_t)(0xA5 ^ ((i / SECTOR) * 31) ^ (i * 7) ^ (i >> 8));
    int64_t worst = 0, total = 0;
    esp_err_t werr = ESP_OK;
    for (int c = 0; c < TEST_SECTORS; c += CHUNK) {
        t0 = esp_timer_get_time();
        esp_err_t e = sdmmc_write_sectors(&card, pat + c * SECTOR, start + c,
                                          CHUNK);
        dt = esp_timer_get_time() - t0;
        if (e != ESP_OK) werr = e;
        total += dt;
        if (dt > worst) worst = dt;
    }
    snprintf(d, sizeof d, "avg %.1f ms, worst %.1f ms per 4 KB (%s)",
             ms(total) / (TEST_SECTORS / CHUNK), ms(worst),
             esp_err_to_name(werr));
    result(werr == ESP_OK, "write in 4 KB chunks", d);

    err = sdmmc_read_sectors(&card, back, start, TEST_SECTORS);
    int bad = -1;
    for (size_t i = 0; err == ESP_OK && i < bytes; i++)
        if (back[i] != pat[i]) { bad = (int)i; break; }
    if (bad >= 0)
        snprintf(d, sizeof d, "first mismatch at byte %d: wrote 0x%02X, "
                 "read 0x%02X", bad, pat[bad], back[bad]);
    else
        snprintf(d, sizeof d, "%s", esp_err_to_name(err));
    result(err == ESP_OK && bad < 0, "read back matches what was written", d);

    werr = sdmmc_write_sectors(&card, orig, start, TEST_SECTORS);
    err = sdmmc_read_sectors(&card, back, start, TEST_SECTORS);
    const bool restored = werr == ESP_OK && err == ESP_OK &&
                          memcmp(back, orig, bytes) == 0;
    result(restored, "original contents restored", restored ? "" :
           "the last 64 sectors may hold the test pattern now");

    /* ---- 6: does MISO really float between commands? ---- */
    printf("\n6. Pull-up check (%d single-sector reads each)\n", STALL_READS);
    t0 = esp_timer_get_time();
    for (int i = 0; i < STALL_READS; i++)
        sdmmc_read_sectors(&card, back, start, 1);
    const double per_up = ms(esp_timer_get_time() - t0) / STALL_READS;
    gpio_pullup_dis(PIN_MISO);
    gpio_pulldown_en(PIN_MISO);
    t0 = esp_timer_get_time();
    for (int i = 0; i < STALL_READS; i++)
        sdmmc_read_sectors(&card, back, start, 1);
    const double per_dn = ms(esp_timer_get_time() - t0) / STALL_READS;
    gpio_pulldown_dis(PIN_MISO);
    gpio_pullup_en(PIN_MISO);
    snprintf(d, sizeof d, "%.1f ms per read pulled up, %.1f ms pulled down",
             per_up, per_dn);
    result(per_dn > per_up + 20.0,
           "MISO floats between commands (pull-down stalls IDF's busy poll)",
           d);
    if (per_dn <= per_up + 20.0)
        printf("       -> no stall: something still drives MISO while CS "
               "is high.\n");
    else
        printf("       -> so the pull-up is required, and blackbox.c:189 "
               "already enables it.\n");

    heap_caps_free(orig);
    heap_caps_free(pat);
    heap_caps_free(back);
}

void app_main(void)
{
    vTaskDelay(pdMS_TO_TICKS(1500));      /* let the USB console attach */
    run();
    printf("\n=== %d passed, %d failed ===\n", s_pass, s_fail);
    if (s_fail == 0)
        printf("Module is fit for a shared SPI3: it releases MISO when not "
               "selected, and the card works at the blackbox's 10 MHz.\n");
    printf("Press EN to run again.\n");
}
