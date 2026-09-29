#include "blackbox.h"
#include "params.h"
#include "fc_cores.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "driver/spi_common.h"
#include "driver/sdspi_host.h"
#include "driver/gpio.h"
#include "sdmmc_cmd.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "BLACKBOX";

/* The on-card format has no version field - the TLV len byte IS the
 * frame version, and tools/blackbox_decode.py dispatches on it (34, 35,
 * 43, 48, 50, 54, 66, 74, 75). So a change to bb_frame_t that nobody mirrors in
 * the decoder does not fail loudly; it silently produces logs the tool skips
 * as "not a frame length we know", and the flight you wanted is gone.
 *
 * Fail at compile time instead. If this fires: add the new size to
 * FRAMES in blackbox_decode.py, then update the number here.
 * tools/blackbox_frame_test.py checks the other half of the same
 * agreement - that the decoder's field LAYOUT still matches this
 * struct, which a size check alone cannot see. */
_Static_assert(sizeof(bb_frame_t) == 75,
               "bb_frame_t size changed - update FRAMES in "
               "tools/blackbox_decode.py before changing this assert");

/* ===================== WHY AN SD CARD  =====================
 *
 * Until this wrote a ring on the internal flash, erasing each
 * 4 KB sector just before filling it. An erase on internal flash disables
 * the flash cache CHIP-WIDE - both cores, every task, whatever its priority
 * - for as long as the erase takes. On the original S3 that was ~27 ms,
 * ~5x a second: already a known loop stall.
 *
 * The replacement S3 carries a Zbit flash part (JEDEC 0x5E, device 0x4018)
 * whose sector erase measured ~90 ms. Blackbox s10/s11: the
 * flight loop was blind for 85-95 ms every 212 ms - one gap per sector,
 * 349 gaps over 40 ms in s11 against 0 on the old board - and the aircraft
 * rocked on the ground at ~2 Hz and could not be lifted.
 *
 * An SD card on its own SPI bus has no erase and never touches the flash
 * cache, so the loop cannot be stalled by logging at all. A slow card only
 * delays THIS task (core 0, prio 2); the queue absorbs it.
 *
 * ===================== ON-CARD LAYOUT =====================
 *
 * Raw blocks, no filesystem. A filesystem adds allocation-table writes at
 * unpredictable moments and buys nothing here: logs come off over USB with
 * tools/blackbox_pull.py exactly as before, and the byte stream it receives
 * is unchanged, so neither the pull tool nor the decoder had to move.
 *
 *   LBA 0          superblock (512 B): magic 'BBSD', version, data_lba.
 *   data_lba+8*i   log sector i (4096 B = 8 blocks), header seq == i+1.
 *
 * Writes are strictly linear and the index only advances on a successful
 * write, so "sector i is valid" is true for every i below the head and
 * false from the head on. Boot finds the head by BINARY SEARCH - ~28
 * header reads on a 128 GB card, instead of the old full scan.
 *
 * A card without the superblock is NEVER written. SDFORMAT (or BBERASE)
 * marks it. On an already-marked card, "erase" does not touch the old data:
 * it moves data_lba past it, which is O(1) and leaves the region beyond the
 * new start guaranteed never-written by any previous epoch. */

#define SECTOR_SIZE       4096
#define BLOCK_SIZE        512
#define BLOCKS_PER_SECTOR (SECTOR_SIZE / BLOCK_SIZE)
#define SUPER_LBA         0
#define FIRST_DATA_LBA    8

#define HDR_MAGIC       0x31584242u          /* "BBX1" little-endian */
#define SUPER_MAGIC     0x44534242u          /* "BBSD" little-endian */
#define SUPER_VERSION   1u
#define REC_PARAMS      1
#define REC_FRAME       2
#define REC_SESSION_END 3
#define QUEUE_DEPTH     192                  /* ~770 ms of frames: rides out
                                              * a card's busy period */

/* BBDUMP with no argument sends the newest this-many sectors: the same
 * ~12 MB the flash ring held, so a pull still takes ~90 s. The card holds
 * far more; BBDUMP,<seq> reaches any of it. */
#define DUMP_WINDOW_SECTORS 3000u

/* Card retry cadence while there is no usable card and no session. */
#define RETRY_MS        2000

/* A write slower than this is counted in BBINFO. 20 ms is ~5 flight-loop
 * frames - harmless to the loop (different core, queued), but a card that
 * does it often is a card to replace before it does it for 800 ms. */
#define SLOW_WRITE_MS   20

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t seq;        /* == sector index + 1 within the current epoch */
    uint16_t session;
    uint16_t flags;      /* bit0: sector begins a new session */
} sector_hdr_t;

typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint32_t version;
    uint32_t data_lba;
} super_t;

typedef enum {
    SD_NO_CARD = 0,      /* nothing answered, or not probed yet */
    SD_UNMARKED,         /* a card, but not ours: never written until SDFORMAT */
    SD_READY,
    SD_FAULT,            /* a write failed; logging stopped until re-probe */
    SD_FULL,
} sd_state_t;

static const char *const STATE_NAME[] = {
    "NO CARD", "UNMARKED (run SDFORMAT)", "READY", "WRITE FAULT", "FULL",
};

static QueueHandle_t      s_q = NULL;
static TaskHandle_t       s_writer = NULL;
static SemaphoreHandle_t  s_card_mtx = NULL;   /* every card access */

/* Set once the bus is up and the card has been probed at least once; never
 * cleared. The other SPI3 devices wait on it (blackbox.h, SPI3 IS SHARED). */
static EventGroupHandle_t s_spi3_ev = NULL;
#define SPI3_SETTLED_BIT  BIT0

static bool               s_bus_up = false;
static sdspi_dev_handle_t s_dev = -1;
static sdmmc_host_t       s_host;
static sdmmc_card_t       s_card;
static volatile sd_state_t s_state = SD_NO_CARD;

static uint32_t s_data_lba   = FIRST_DATA_LBA;
static uint32_t s_n_sectors  = 0;
static uint32_t s_next_index = 0;    /* head: sector index written next */
static uint16_t s_session    = 0;

static volatile bool s_session_open   = false;
static volatile bool s_start_pending  = false;
static volatile bool s_stop_pending   = false;
static volatile uint32_t s_dropped    = 0;
static uint32_t s_frames_written      = 0;
static uint32_t s_write_max_ms        = 0;
static uint32_t s_write_slow          = 0;

/* Sector assembly buffer (writer task only) and a read buffer (dump side).
 * DMA-capable so the SPI driver never bounces them. */
static uint8_t *s_buf = NULL;
static uint8_t *s_rd  = NULL;
static size_t   s_fill = 0;          /* 0 = sector not started */
static bool     s_new_session_sector = false;

/* ===================== card access (hold s_card_mtx) ===================== */

static esp_err_t card_probe(void)
{
    if (!s_bus_up) {
        spi_bus_config_t bus = {
            .mosi_io_num   = BB_SD_PIN_MOSI,
            .miso_io_num   = BB_SD_PIN_MISO,
            .sclk_io_num   = BB_SD_PIN_SCK,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = SECTOR_SIZE + 64,
        };
        esp_err_t err = spi_bus_initialize(BB_SD_SPI_HOST, &bus, SPI_DMA_CH_AUTO);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(err));
            return err;
        }
        /* MISO pull-up. The moment the SD module releases MISO - which a
         * shared bus needs, and the Catalex module does not do until it is
         * fixed - nothing drives the line between transactions. sdspi then
         * reads it with the card's CS still HIGH (poll_busy, before every
         * command) and waits up to 40 ms for non-zero bytes, so a line that
         * floats low would add up to 40 ms to every card command. The pad's
         * ~45k is far weaker than either driver on the line: the SD buffer,
         * and the PMW3901's 1 mA output (IDF asks for >700 uA against
         * pull-ups as strong as 10k). */
        gpio_pullup_en(BB_SD_PIN_MISO);
        err = sdspi_host_init();
        if (err != ESP_OK) return err;

        sdspi_device_config_t dev = SDSPI_DEVICE_CONFIG_DEFAULT();
        dev.host_id = BB_SD_SPI_HOST;
        dev.gpio_cs = BB_SD_PIN_CS;
        err = sdspi_host_init_device(&dev, &s_dev);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "SD device init failed: %s", esp_err_to_name(err));
            return err;
        }
        s_bus_up = true;
    }

    s_host = (sdmmc_host_t)SDSPI_HOST_DEFAULT();
    s_host.slot = s_dev;
    s_host.max_freq_khz = BB_SD_FREQ_KHZ;
    return sdmmc_card_init(&s_host, &s_card);
}

static bool sector_valid(uint32_t i, uint16_t *session_out)
{
    const uint8_t *blk = s_rd;
    if (sdmmc_read_sectors(&s_card, s_rd, s_data_lba + i * BLOCKS_PER_SECTOR, 1)
            != ESP_OK) return false;
    sector_hdr_t h;
    memcpy(&h, blk, sizeof(h));
    if (h.magic != HDR_MAGIC || h.seq != i + 1) return false;
    if (session_out) *session_out = h.session;
    return true;
}

/* Probe, read the superblock, find the head. Sets s_state. */
static void card_bringup(void)
{
    esp_err_t err = card_probe();
    if (err != ESP_OK) {
        s_state = SD_NO_CARD;
        return;
    }
    if (s_card.csd.sector_size != BLOCK_SIZE) {
        ESP_LOGE(TAG, "card sector size %d, need %d", s_card.csd.sector_size,
                 BLOCK_SIZE);
        s_state = SD_FAULT;
        return;
    }
    if (sdmmc_read_sectors(&s_card, s_rd, SUPER_LBA, 1) != ESP_OK) {
        s_state = SD_NO_CARD;
        return;
    }
    super_t sb;
    memcpy(&sb, s_rd, sizeof(sb));
    if (sb.magic != SUPER_MAGIC || sb.version != SUPER_VERSION ||
        sb.data_lba < FIRST_DATA_LBA ||
        sb.data_lba >= (uint32_t)s_card.csd.capacity) {
        ESP_LOGW(TAG, "card present (%llu MB) but UNMARKED - nothing will be "
                 "written until SDFORMAT",
                 (unsigned long long)s_card.csd.capacity * BLOCK_SIZE / (1024 * 1024));
        s_state = SD_UNMARKED;
        return;
    }

    s_data_lba  = sb.data_lba;
    s_n_sectors = ((uint32_t)s_card.csd.capacity - s_data_lba) / BLOCKS_PER_SECTOR;

    /* Smallest i with !valid(i). Valid below the head, invalid from it on. */
    uint32_t lo = 0, hi = s_n_sectors;
    while (lo < hi) {
        uint32_t mid = lo + (hi - lo) / 2;
        if (sector_valid(mid, NULL)) lo = mid + 1;
        else                         hi = mid;
    }
    s_next_index = lo;
    s_session = 0;
    if (s_next_index > 0) sector_valid(s_next_index - 1, &s_session);

    s_state = (s_next_index >= s_n_sectors) ? SD_FULL : SD_READY;
    ESP_LOGI(TAG, "SD %s: %llu MB, head sector %u, last session %u",
             STATE_NAME[s_state],
             (unsigned long long)s_card.csd.capacity * BLOCK_SIZE / (1024 * 1024),
             (unsigned)s_next_index, s_session);
}

/* ===================== writer side ===================== */

static void sector_begin(void)
{
    memset(s_buf, 0xFF, SECTOR_SIZE);
    sector_hdr_t h = {
        .magic   = HDR_MAGIC,
        .seq     = s_next_index + 1,
        .session = s_session,
        .flags   = s_new_session_sector ? 1 : 0,
    };
    memcpy(s_buf, &h, sizeof(h));
    s_fill = sizeof(h);
    s_new_session_sector = false;
}

static void sector_commit(void)
{
    s_fill = 0;
    if (s_state != SD_READY) return;
    if (s_next_index >= s_n_sectors) { s_state = SD_FULL; return; }

    xSemaphoreTake(s_card_mtx, portMAX_DELAY);
    int64_t t0 = esp_timer_get_time();
    esp_err_t err = sdmmc_write_sectors(&s_card, s_buf,
                        s_data_lba + s_next_index * BLOCKS_PER_SECTOR,
                        BLOCKS_PER_SECTOR);
    if (err != ESP_OK) {
        /* One retry: an SPI card can refuse a command while busy. */
        err = sdmmc_write_sectors(&s_card, s_buf,
                  s_data_lba + s_next_index * BLOCKS_PER_SECTOR,
                  BLOCKS_PER_SECTOR);
    }
    uint32_t ms = (uint32_t)((esp_timer_get_time() - t0) / 1000);
    xSemaphoreGive(s_card_mtx);

    if (ms > s_write_max_ms) s_write_max_ms = ms;
    if (ms > SLOW_WRITE_MS)  s_write_slow++;

    if (err != ESP_OK) {
        /* Do NOT advance: the head stays on the sector that failed, so the
         * card stays gap-free and the binary search stays true. Logging
         * stops; the arm gate refuses the next flight until a re-probe
         * finds the card healthy again. */
        ESP_LOGE(TAG, "sector %u write failed: %s - logging stopped",
                 (unsigned)s_next_index, esp_err_to_name(err));
        s_state = SD_FAULT;
        return;
    }
    s_next_index++;
}

/* Append one TLV record, committing the sector first if it cannot fit. */
static void rec_append(uint8_t type, const void *payload, uint8_t len)
{
    if (s_fill == 0) sector_begin();
    if (s_fill + 2 + len > SECTOR_SIZE) {
        sector_commit();
        sector_begin();
    }
    s_buf[s_fill]     = type;
    s_buf[s_fill + 1] = len;
    memcpy(&s_buf[s_fill + 2], payload, len);
    s_fill += 2 + (size_t)len;
}

static void writer_task(void *pv)
{
    bb_frame_t f;
    int64_t next_retry_us = 0;

    while (1) {
        /* Card bring-up lives here, not in blackbox_init: probing an empty
         * slot can take a second, and boot must not wait on it. Retried
         * only between sessions - never touch the card's init sequence
         * while a flight is being recorded. */
        if (s_state != SD_READY && !s_session_open &&
            esp_timer_get_time() >= next_retry_us) {
            xSemaphoreTake(s_card_mtx, portMAX_DELAY);
            if (s_state != SD_READY) card_bringup();
            xSemaphoreGive(s_card_mtx);
            next_retry_us = esp_timer_get_time() + (int64_t)RETRY_MS * 1000;
            /* Card in SPI mode, or no card: either way nothing on the bus
             * can mistake another device's traffic for SD commands now.
             * NOT set while s_bus_up is false - the bus does not exist. */
            if (s_bus_up) xEventGroupSetBits(s_spi3_ev, SPI3_SETTLED_BIT);
        }

        if (s_start_pending) {
            s_start_pending = false;
            s_session++;
            s_new_session_sector = true;
            if (s_fill != 0) sector_commit();   /* session starts a sector */
            struct __attribute__((packed)) {
                uint32_t version;
                params_t p;
            } snap = { .version = PARAMS_VERSION, .p = g_params };
            rec_append(REC_PARAMS, &snap, (uint8_t)sizeof(snap));
            s_frames_written = 0;
            s_dropped = 0;
            ESP_LOGI(TAG, "session %u started", s_session);
        }

        if (xQueueReceive(s_q, &f, pdMS_TO_TICKS(50)) == pdTRUE) {
            rec_append(REC_FRAME, &f, (uint8_t)sizeof(f));
            s_frames_written++;
        }

        if (s_stop_pending && uxQueueMessagesWaiting(s_q) == 0) {
            s_stop_pending = false;
            uint8_t z = 0;
            rec_append(REC_SESSION_END, &z, 1);
            sector_commit();                    /* flush the partial sector */
            ESP_LOGI(TAG, "session %u closed: %u frames, %u dropped, "
                     "write max %u ms",
                     s_session, (unsigned)s_frames_written,
                     (unsigned)s_dropped, (unsigned)s_write_max_ms);
        }
    }
}

/* Rule 1 of the shared bus (blackbox.h): every other device's CS HIGH before
 * the card sees its first clock. Level first, then direction, so enabling
 * the output can never produce even a momentary low. First thing in
 * blackbox_init, ahead of anything that can fail and return. */
static void park_guest_cs(void)
{
    static const int pins[] = { BB_FLOW_PIN_CS };
    for (size_t i = 0; i < sizeof(pins) / sizeof(pins[0]); i++) {
        gpio_set_level(pins[i], 1);
        const gpio_config_t io = {
            .pin_bit_mask = 1ULL << pins[i],
            .mode         = GPIO_MODE_OUTPUT,
        };
        gpio_config(&io);
        gpio_set_level(pins[i], 1);
    }
}

void blackbox_init(void)
{
    park_guest_cs();
    s_spi3_ev = xEventGroupCreate();

    s_buf = heap_caps_malloc(SECTOR_SIZE, MALLOC_CAP_DMA);
    s_rd  = heap_caps_malloc(SECTOR_SIZE, MALLOC_CAP_DMA);
    s_card_mtx = xSemaphoreCreateMutex();
    s_q = xQueueCreate(QUEUE_DEPTH, sizeof(bb_frame_t));
    if (!s_buf || !s_rd || !s_card_mtx || !s_q) {
        ESP_LOGE(TAG, "out of memory - blackbox DISABLED (arming will be "
                      "refused)");
        return;
    }
    ESP_LOGI(TAG, "SD on SPI3: CS %d SCK %d MOSI %d MISO %d, %d kHz",
             BB_SD_PIN_CS, BB_SD_PIN_SCK, BB_SD_PIN_MOSI, BB_SD_PIN_MISO,
             BB_SD_FREQ_KHZ);
    xTaskCreatePinnedToCore(writer_task, "bb_writer", 4096, NULL, 2,
                            &s_writer, FC_CORE_SUPPORT);
}

bool blackbox_ready(void)
{
    return s_writer != NULL && s_state == SD_READY;
}

bool blackbox_spi3_wait_settled(uint32_t timeout_ms)
{
    if (!s_spi3_ev) {
        /* blackbox_init has not run (or could not make the event): the bus
         * may not exist, so the answer is no - after the wait, so a caller
         * polling this in a loop does not spin. */
        vTaskDelay(pdMS_TO_TICKS(timeout_ms));
        return false;
    }
    const EventBits_t b = xEventGroupWaitBits(s_spi3_ev, SPI3_SETTLED_BIT,
                                              pdFALSE, pdTRUE,
                                              pdMS_TO_TICKS(timeout_ms));
    return (b & SPI3_SETTLED_BIT) != 0;
}

void blackbox_session_start(void)
{
    if (!s_writer) return;
    s_session_open  = true;
    s_start_pending = true;
}

void blackbox_session_stop(void)
{
    if (!s_writer) return;
    s_session_open = false;
    s_stop_pending = true;
}

void blackbox_log(const bb_frame_t *f)
{
    if (!s_session_open || !s_q) return;
    if (s_state != SD_READY || xQueueSend(s_q, f, 0) != pdTRUE) s_dropped++;
}

void blackbox_info(char *buf, size_t buflen)
{
    if (!s_writer) {
        snprintf(buf, buflen, "blackbox: DISABLED (init failed)");
        return;
    }
    if (s_state == SD_NO_CARD || s_state == SD_UNMARKED) {
        snprintf(buf, buflen, "blackbox: SD %s", STATE_NAME[s_state]);
        return;
    }
    /* frames per sector: header, then 2-byte TLV prefix per frame */
    const uint32_t per_sector = (SECTOR_SIZE - (uint32_t)sizeof(sector_hdr_t))
                                / ((uint32_t)sizeof(bb_frame_t) + 2u);
    const uint32_t free_h = (uint32_t)(((uint64_t)(s_n_sectors - s_next_index)
                              * per_sector) / BB_LOG_RATE_HZ / 3600u);
    snprintf(buf, buflen,
             "blackbox: SD %s, %u sectors (%u MB) used, ~%u h free, "
             "session %u, %u frames, %u dropped, write max %u ms (%u >%d ms)",
             STATE_NAME[s_state], (unsigned)s_next_index,
             (unsigned)(s_next_index / 256u), (unsigned)free_h, s_session,
             (unsigned)s_frames_written, (unsigned)s_dropped,
             (unsigned)s_write_max_ms, (unsigned)s_write_slow, SLOW_WRITE_MS);
}

int blackbox_format(void)
{
    if (!s_writer) return -1;
    if (s_session_open || s_stop_pending) return -2;

    xSemaphoreTake(s_card_mtx, portMAX_DELAY);
    if (s_state == SD_NO_CARD) card_bringup();
    if (s_bus_up) xEventGroupSetBits(s_spi3_ev, SPI3_SETTLED_BIT);
    if (s_state == SD_NO_CARD) {
        xSemaphoreGive(s_card_mtx);
        return -3;
    }

    /* Marked card: skip past everything written in this epoch (+1 spare
     * sector). Unmarked or faulted card: start at the first data LBA - no
     * earlier epoch of ours can be ahead of it. */
    uint32_t new_lba = FIRST_DATA_LBA;
    if (s_state == SD_READY || s_state == SD_FULL) {
        new_lba = s_data_lba + (s_next_index + 1) * BLOCKS_PER_SECTOR;
    }
    if (new_lba + BLOCKS_PER_SECTOR >= (uint32_t)s_card.csd.capacity) {
        new_lba = FIRST_DATA_LBA;   /* card exhausted: wrap to the start */
    }

    memset(s_rd, 0, BLOCK_SIZE);
    super_t sb = { .magic = SUPER_MAGIC, .version = SUPER_VERSION,
                   .data_lba = new_lba };
    memcpy(s_rd, &sb, sizeof(sb));
    esp_err_t err = sdmmc_write_sectors(&s_card, s_rd, SUPER_LBA, 1);
    if (err == ESP_OK && new_lba == FIRST_DATA_LBA) {
        /* Starting at the very beginning: make sure sector 0 does not hold a
         * valid header left by an epoch that wrapped. Blank the header
         * block only - that alone makes the binary search stop at 0. */
        memset(s_rd, 0, BLOCK_SIZE);
        err = sdmmc_write_sectors(&s_card, s_rd, FIRST_DATA_LBA, 1);
    }
    if (err != ESP_OK) {
        s_state = SD_FAULT;
        xSemaphoreGive(s_card_mtx);
        return -(int)err;
    }

    s_data_lba     = new_lba;
    s_n_sectors    = ((uint32_t)s_card.csd.capacity - s_data_lba) / BLOCKS_PER_SECTOR;
    s_next_index   = 0;
    s_session      = 0;
    s_fill         = 0;
    s_write_max_ms = 0;
    s_write_slow   = 0;
    s_state        = SD_READY;
    xSemaphoreGive(s_card_mtx);
    ESP_LOGW(TAG, "SD formatted: data starts at LBA %u, %u sectors",
             (unsigned)s_data_lba, (unsigned)s_n_sectors);
    return 0;
}

/* Standard base64, RFC 4648. */
static const char B64[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static size_t b64_encode(const uint8_t *in, size_t n, char *out)
{
    size_t o = 0;
    for (size_t i = 0; i < n; i += 3) {
        uint32_t v = (uint32_t)in[i] << 16;
        if (i + 1 < n) v |= (uint32_t)in[i + 1] << 8;
        if (i + 2 < n) v |= in[i + 2];
        out[o++] = B64[(v >> 18) & 63];
        out[o++] = B64[(v >> 12) & 63];
        out[o++] = (i + 1 < n) ? B64[(v >> 6) & 63] : '=';
        out[o++] = (i + 2 < n) ? B64[v & 63] : '=';
    }
    out[o] = 0;
    return o;
}

/* Read sector i and emit it as "BB,<seq>,<off>,<b64>" lines, 768 raw bytes
 * per line - byte-identical to what the flash ring produced, so
 * tools/blackbox_pull.py and the decoder are unchanged. Returns true if the
 * sector was valid and emitted. */
static bool dump_one_sector(uint32_t i, void (*emit_line)(const char *line))
{
    static char line[16 + 1024 + 8];

    xSemaphoreTake(s_card_mtx, portMAX_DELAY);
    esp_err_t err = sdmmc_read_sectors(&s_card, s_rd,
                        s_data_lba + i * BLOCKS_PER_SECTOR, BLOCKS_PER_SECTOR);
    xSemaphoreGive(s_card_mtx);
    if (err != ESP_OK) return false;

    sector_hdr_t h;
    memcpy(&h, s_rd, sizeof(h));
    if (h.magic != HDR_MAGIC || h.seq != i + 1) return false;

    for (uint32_t off = 0; off < SECTOR_SIZE; off += 768) {
        uint32_t n = SECTOR_SIZE - off;
        if (n > 768) n = 768;
        int p = snprintf(line, sizeof(line), "BB,%u,%u,",
                         (unsigned)h.seq, (unsigned)off);
        b64_encode(s_rd + off, n, line + p);
        emit_line(line);
    }
    return true;
}

void blackbox_dump(void (*emit_line)(const char *line))
{
    char eof[24];
    uint32_t emitted = 0;

    if (s_writer && (s_state == SD_READY || s_state == SD_FULL ||
                     s_state == SD_FAULT)) {
        uint32_t head  = s_next_index;
        uint32_t start = head > DUMP_WINDOW_SECTORS ? head - DUMP_WINDOW_SECTORS : 0;
        for (uint32_t i = start; i < head; i++) {
            if (dump_one_sector(i, emit_line)) emitted++;
        }
    }
    snprintf(eof, sizeof(eof), "BBEOF,%u", (unsigned)emitted);
    emit_line(eof);
}

void blackbox_dump_seq(uint32_t seq, void (*emit_line)(const char *line))
{
    char eof[24];
    uint32_t found = 0;

    if (s_writer && seq >= 1 && seq <= s_next_index &&
        (s_state == SD_READY || s_state == SD_FULL || s_state == SD_FAULT)) {
        found = dump_one_sector(seq - 1, emit_line) ? 1 : 0;
    }
    snprintf(eof, sizeof(eof), "BBEOF,%u", (unsigned)found);
    emit_line(eof);
}
