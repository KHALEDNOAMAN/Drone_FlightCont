#include "pmw3901.h"
#include "fc_cores.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "PMW3901";

/* ===================== SPI timing =====================
 * Microseconds. Datasheet v1.10 table 5 minimum in brackets. Where the spec
 * is tiny the value here is generous on purpose: this is the flow task's own
 * time, spent in esp_rom_delay_us, and never on the shared slow-sensor path
 * (incident 29). Only the bus-holding parts cost anyone else anything.
 *
 * NOT 50 us everywhere "because Bitcraze does": the one that matters most is
 * tSCLK-NCS for a WRITE - NCS must stay low >= 35 us after the last clock or
 * the write can be aborted. PX4's driver releases CS immediately, which
 * violates it. */
#define T_CS_SETUP_US       10   /* NCS low -> first SCLK      tNCS-SCLK [0.12] */
#define T_SRAD_REG_US      100   /* address -> data, register  tSRAD     [35]   */
#define T_SRAD_BURST_US     50   /* address -> data, burst     tSRAD     [35], Crazyflie's proven 50 */
#define T_CS_HOLD_READ_US   10   /* last SCLK -> NCS high      tSCLK-NCS [0.12] */
#define T_CS_HOLD_WRITE_US  50   /* last SCLK -> NCS high      tSCLK-NCS [35]   */
#define T_GAP_READ_US       50   /* read -> next command       tSRR/tSRW [20]   */
#define T_GAP_WRITE_US     200   /* write -> next command      tSWW/tSWR [45], Bitcraze/Crazyflie */
#define T_GAP_BURST_US      50   /* burst -> next command      tBEXIT [0.5] + tSRR */
#define T_MISO_SETTLE_US    50   /* ~45k pad pull into tens of pF: >15 tau */

#define RESET_SETTLE_MS     10   /* after Power_Up_Reset, before any access (refs use 5) */
#define MOTION_AFTER_RESET_US 60000 /* tMOT-RST: motion valid 50 ms after reset */
#define LIVENESS_EVERY      (1000 / PMW3901_PERIOD_MS)   /* once a second */
#define ID_FAIL_LIMIT       3
#define SENTINEL_FAIL_LIMIT 2
#define OBS_WAIT_BURSTS     5
#define MISMATCH_KEEP       4

/* ===================== state ===================== */

typedef enum { ST_IDLE = 0, ST_WAIT_BUS, ST_INIT, ST_RUN, ST_RETRY } run_state_t;
static const char *const STATE_NAME[] = {
    "not started", "waiting for SPI3", "initialising", "RUNNING", "retrying",
};

typedef enum {
    WHY_NONE = 0, WHY_MISO_HELD, WHY_NO_CHIP, WHY_POST_RESET, WHY_SPI_ERR,
    WHY_ID_LOST, WHY_SENSOR_RESET, WHY_CLOCK,
} why_t;
static const char *const WHY_NAME[] = {
    "-", "MISO held by another device", "no PMW3901 answering",
    "lost after soft reset", "SPI transaction error",
    "identity check failed while running", "chip lost its configuration",
    "SPI clock above datasheet maximum",
};

typedef enum {
    MISO_UNTESTED = 0, MISO_RELEASED, MISO_HELD_HIGH, MISO_HELD_LOW,
    MISO_ERRATIC, MISO_FORCED,
} miso_t;
static const char *const MISO_NAME[] = {
    "untested", "released (floats when idle)", "HELD HIGH", "HELD LOW",
    "erratic", "check SKIPPED (FLOW,FORCE)",
};

static pmw3901_cfg_t       s_cfg;
static spi_device_handle_t s_dev;
static int                 s_clk_khz;

/* Internal RAM and word-aligned, so the DMA-enabled bus never bounce-
 * allocates per transaction. Touched by the flow task only. */
static uint8_t s_tx[16] __attribute__((aligned(4)));
static uint8_t s_rx[16] __attribute__((aligned(4)));

/* Published: written by the flow task, read by anyone, under s_mux. */
static portMUX_TYPE     s_mux = portMUX_INITIALIZER_UNLOCKED;
static pmw3901_track_t  s_track;
static pmw3901_sample_t s_latest;
static bool             s_have_latest;
static uint32_t         s_seq;

/* Diagnostics: single writer (the flow task), read unlocked by the console.
 * A torn read of a counter is off by one - acceptable for a status line. */
static volatile run_state_t s_state;
static volatile why_t       s_why;
static volatile bool        s_force_next;
static volatile bool        s_id_ok_now;
static miso_t   s_miso;
static uint8_t  s_id, s_inv;
static pmw3901_id_verdict_t s_id_verdict;
/* Whether an identity read has EVER happened. Until it has, s_id/s_inv are
 * the zeros they were born as and the verdict is the enum's default - which
 * is "ok". Printing that as "ID 00/00 (ok)" turned "never asked" into a
 * passing answer (seen on the first FLOW). */
static bool     s_id_read;

/* How far the last init attempt got, so FLOW can say where it stopped
 * instead of printing the fields of steps that never ran. */
typedef enum {
    STAGE_NONE = 0, STAGE_MISO, STAGE_ID, STAGE_RESET, STAGE_TABLES,
    STAGE_DONE,
} init_stage_t;
static const char *const STAGE_NAME[] = {
    "not started", "the MISO check", "the identity read", "the soft reset",
    "the register tables", "done",
};
static init_stage_t s_stage;

static uint32_t s_init_tries, s_init_ok, s_init_ms;
static uint8_t  s_pre_67, s_pre_73, s_pre_c1, s_pre_c2;
static bool     s_pre_cal;
static uint32_t s_rb_checked, s_rb_matched;
static struct { uint8_t bank, reg, want, got; } s_mm[MISMATCH_KEEP];

static int      s_sent_n;
static uint8_t  s_sent_reg[2], s_sent_val[2];
static uint32_t s_sent_streak, s_sent_disagree, s_sensor_resets;

static uint32_t s_id_fail_total, s_id_fail_streak, s_spi_errs;
static uint32_t s_obs_pass, s_obs_fail, s_obs_wait;
static uint8_t  s_obs_last;
static bool     s_obs_armed;
static uint32_t s_wait_max_us, s_dt_max_us, s_latch_unc_max_us;

/* ===================== bus access ===================== */

static inline void cs_low(void)  { gpio_set_level(s_cfg.pin_cs, 0); }
static inline void cs_high(void) { gpio_set_level(s_cfg.pin_cs, 1); }

/* Take SPI3 and put SCLK at this chip's idle level. Returns the wait.
 *
 * WHY THE DUMMY BYTE. The SD card runs mode 0, where SCLK idles LOW; this
 * chip runs mode 3, where it idles HIGH. When the bus changes hands the
 * driver writes the new polarity into the peripheral, but on the S3 it only
 * reaches the pin when the next transaction starts (spi_ll_apply_config).
 * If NCS were already low at that moment, the pin's low->high step would be
 * a rising edge - this chip's SAMPLE edge - and it would clock in a bit that
 * was never sent, shifting everything after it by one. One byte clocked
 * with every chip select still high settles SCLK where nobody is listening.
 * (If it is ever skipped, the identity read comes back as 0x24/0x5B or
 * 0x92/0x6C and pmw3901_id_check() says "shifted".)
 *
 * False means the bus was NOT taken, and the caller must not touch CS or
 * call bus_release() - releasing a bus that was never acquired asserts, and
 * a passive sensor must never be able to reboot the flight controller. */
static bool bus_acquire(uint32_t *waited_us)
{
    const int64_t t0 = esp_timer_get_time();
    if (spi_device_acquire_bus(s_dev, portMAX_DELAY) != ESP_OK) {
        s_spi_errs++;
        return false;
    }
    if (waited_us) *waited_us = (uint32_t)(esp_timer_get_time() - t0);

    s_tx[0] = 0xFF;
    spi_transaction_t t = { .length = 8, .tx_buffer = s_tx };
    spi_device_polling_transmit(s_dev, &t);
    return true;
}

static void bus_release(void)
{
    spi_device_release_bus(s_dev);
}

static esp_err_t xfer(size_t n, bool rx)
{
    spi_transaction_t t = {
        .length    = 8 * n,
        .tx_buffer = s_tx,
        .rx_buffer = rx ? s_rx : NULL,
    };
    return spi_device_polling_transmit(s_dev, &t);
}

/* One register write = one CS frame. CS is by hand (spics_io_num = -1)
 * because a READ needs a pause inside a single frame, and the gap after
 * release is spent outside the bus lock - it is a minimum, and if the card
 * takes the bus meanwhile the gap only grows. */
static esp_err_t reg_write(uint8_t reg, uint8_t val)
{
    if (!bus_acquire(NULL)) return ESP_FAIL;
    cs_low();
    esp_rom_delay_us(T_CS_SETUP_US);
    s_tx[0] = reg | 0x80;
    s_tx[1] = val;
    const esp_err_t err = xfer(2, false);
    esp_rom_delay_us(T_CS_HOLD_WRITE_US);
    cs_high();
    bus_release();
    esp_rom_delay_us(T_GAP_WRITE_US);
    if (err != ESP_OK) s_spi_errs++;
    return err;
}

static esp_err_t reg_read(uint8_t reg, uint8_t *val)
{
    *val = 0;
    if (!bus_acquire(NULL)) return ESP_FAIL;
    cs_low();
    esp_rom_delay_us(T_CS_SETUP_US);
    s_tx[0] = reg & 0x7F;
    esp_err_t err = xfer(1, false);
    esp_rom_delay_us(T_SRAD_REG_US);
    s_tx[0] = 0x00;
    s_rx[0] = 0x00;
    if (err == ESP_OK) err = xfer(1, true);
    esp_rom_delay_us(T_CS_HOLD_READ_US);
    cs_high();
    bus_release();
    esp_rom_delay_us(T_GAP_READ_US);
    *val = s_rx[0];
    if (err != ESP_OK) s_spi_errs++;
    return err;
}

/* Motion burst. *latched says whether the address byte went out - from
 * that moment the chip has cleared its counters, so the window is over
 * whether or not the data phase then succeeds. */
static esp_err_t burst_read(uint8_t out[PMW3901_BURST_LEN], int64_t *t_latch,
                            uint32_t *wait_us, bool *latched)
{
    *latched = false;
    *wait_us = 0;
    if (!bus_acquire(wait_us)) return ESP_FAIL;
    cs_low();
    esp_rom_delay_us(T_CS_SETUP_US);
    s_tx[0] = PMW3901_REG_MOTION_BURST;
    const int64_t a = esp_timer_get_time();
    esp_err_t err = xfer(1, false);
    const int64_t b = esp_timer_get_time();
    *latched = (err == ESP_OK);
    esp_rom_delay_us(T_SRAD_BURST_US);
    memset(s_tx, 0, PMW3901_BURST_LEN);
    memset(s_rx, 0, PMW3901_BURST_LEN);
    if (err == ESP_OK) err = xfer(PMW3901_BURST_LEN, true);
    esp_rom_delay_us(T_CS_HOLD_READ_US);
    cs_high();
    bus_release();
    esp_rom_delay_us(T_GAP_BURST_US);

    /* The latch is somewhere inside the address byte's transaction. Stamp
     * the midpoint and keep the width: a preemption between a and b would
     * show up here instead of silently skewing dt. */
    *t_latch = a + (b - a) / 2;
    const uint32_t unc = (uint32_t)((b - a) / 2);
    if (unc > s_latch_unc_max_us) s_latch_unc_max_us = unc;
    memcpy(out, s_rx, PMW3901_BURST_LEN);
    if (err != ESP_OK) s_spi_errs++;
    return err;
}

/* ===================== is MISO actually free? =====================
 * With the bus held (so no card command is in flight) and every chip select
 * high, nothing may drive MISO. Pull the pad down, then up, with its own
 * ~45k resistors: a released line follows both, a line some device is
 * driving follows neither. No clocks, no chip selected - this cannot hurt
 * anything, and it is the test that tells a working shared bus from the
 * Catalex SD module, whose MISO buffer is never disabled. Restores the pad
 * pulls to whatever the bus owner configured. */
static miso_t miso_check(void)
{
    const gpio_num_t p = (gpio_num_t)s_cfg.pin_miso;
    gpio_io_config_t io = {0};
    const bool have_io = gpio_get_io_config(p, &io) == ESP_OK;

    if (!bus_acquire(NULL)) return MISO_UNTESTED;
    gpio_pullup_dis(p);
    gpio_pulldown_en(p);
    esp_rom_delay_us(T_MISO_SETTLE_US);
    const int lo = gpio_get_level(p);
    gpio_pulldown_dis(p);
    gpio_pullup_en(p);
    esp_rom_delay_us(T_MISO_SETTLE_US);
    const int hi = gpio_get_level(p);
    if (have_io) {
        if (io.pd) gpio_pulldown_en(p); else gpio_pulldown_dis(p);
        if (io.pu) gpio_pullup_en(p);   else gpio_pullup_dis(p);
    }
    bus_release();

    if (lo == 0 && hi == 1) return MISO_RELEASED;
    if (lo == 1 && hi == 1) return MISO_HELD_HIGH;
    if (lo == 0 && hi == 0) return MISO_HELD_LOW;
    return MISO_ERRATIC;
}

/* ===================== init ===================== */

static bool read_id(void)
{
    uint8_t id = 0, inv = 0;
    const esp_err_t e1 = reg_read(PMW3901_REG_PRODUCT_ID, &id);
    const esp_err_t e2 = reg_read(PMW3901_REG_INV_PRODUCT_ID, &inv);
    s_id = id;
    s_inv = inv;
    s_id_verdict = pmw3901_id_check(id, inv);
    s_id_read = true;
    return e1 == ESP_OK && e2 == ESP_OK && s_id_verdict == PMW_ID_OK;
}

/* PixArt demo kit V3.20's per-chip calibration, ahead of the tables. Two of
 * its writes depend on what the chip reports, so this is code. */
static void run_prelude(void)
{
    uint8_t v = 0;
    reg_write(0x7F, 0x00);
    reg_write(0x55, 0x01);
    reg_write(0x50, 0x07);
    reg_write(0x7F, 0x0E);
    reg_write(0x43, 0x10);
    reg_read(0x67, &v);
    s_pre_67 = v;
    reg_write(0x48, (v & 0x80) ? 0x04 : 0x02);
    reg_write(0x7F, 0x00);
    reg_write(0x51, 0x7B);
    reg_write(0x50, 0x00);
    reg_write(0x55, 0x00);
    reg_write(0x7F, 0x0E);
    reg_read(0x73, &v);
    s_pre_73 = v;
    s_pre_cal = (v == 0);
    if (s_pre_cal) {
        uint8_t c1 = 0, c2 = 0;
        reg_read(0x70, &c1);
        reg_read(0x71, &c2);
        s_pre_c1 = pmw3901_prelude_c1(c1);
        s_pre_c2 = pmw3901_prelude_c2(c2);
        reg_write(0x7F, 0x00);
        reg_write(0x61, 0xAD);
        reg_write(0x51, 0x70);
        reg_write(0x7F, 0x0E);
        reg_write(0x70, s_pre_c1);
        reg_write(0x71, s_pre_c2);
    }
}

/* Write a table, reading every register BACK (an echoed setting is
 * not evidence). A mismatch is recorded, not retried: some of these
 * proprietary registers may be triggers, and writing a trigger again is
 * not a harmless repeat. Nor is a mismatch fatal - which of them read back
 * cleanly on this part is exactly what the first bench run is for. The
 * gates are the identity reads around the tables. ok[] (may be NULL) gets
 * a per-entry verdict. */
static void run_table(const pmw3901_reg_t *tbl, unsigned n, uint8_t *bank,
                      bool *ok)
{
    for (unsigned i = 0; i < n; i++) {
        uint8_t rb = 0;
        const bool w = reg_write(tbl[i].reg, tbl[i].val) == ESP_OK;
        const bool r = reg_read(tbl[i].reg, &rb) == ESP_OK;
        const bool match = w && r && rb == tbl[i].val;
        if (tbl[i].reg == PMW3901_REG_BANK) *bank = tbl[i].val;
        s_rb_checked++;
        if (match) {
            s_rb_matched++;
        } else {
            const uint32_t k = s_rb_checked - s_rb_matched - 1;
            if (k < MISMATCH_KEEP) {
                s_mm[k].bank = *bank;
                s_mm[k].reg  = tbl[i].reg;
                s_mm[k].want = tbl[i].val;
                s_mm[k].got  = rb;
            }
        }
        if (ok) ok[i] = match;
    }
}

/* Reset detection. The IDs survive a brown-out reset; the configuration
 * does not. So two registers we wrote and read back become sentinels, and
 * a reset is declared only when BOTH read wrong in the same check - one
 * register the chip happens to update on its own can then never force a
 * re-init by itself (co-failure). Picked from the tail of table B,
 * which is bank 0, the bank everything afterwards runs in. */
static void pick_sentinels(const bool ok_b[])
{
    s_sent_n = 0;
    for (int i = (int)pmw3901_init_b_len - 1; i >= 0 && s_sent_n < 2; i--) {
        const pmw3901_reg_t *e = &pmw3901_init_b[i];
        if (e->reg == PMW3901_REG_BANK) break;   /* left bank 0's tail */
        if (!ok_b[i]) continue;
        s_sent_reg[s_sent_n] = e->reg;
        s_sent_val[s_sent_n] = e->val;
        s_sent_n++;
    }
    /* Must hold still: re-read over ~20 ms, drop any that moved. */
    for (int k = 0; k < 3 && s_sent_n > 0; k++) {
        vTaskDelay(pdMS_TO_TICKS(7));
        for (int j = 0; j < s_sent_n; j++) {
            uint8_t v = 0;
            if (reg_read(s_sent_reg[j], &v) != ESP_OK || v != s_sent_val[j]) {
                s_sent_reg[j] = s_sent_reg[s_sent_n - 1];
                s_sent_val[j] = s_sent_val[s_sent_n - 1];
                s_sent_n--;
                j--;
            }
        }
    }
    if (s_sent_n < 2) s_sent_n = 0;   /* one alone would be a single-point test */
    s_sent_streak = 0;
}

static why_t chip_init(void)
{
    s_init_tries++;
    const int64_t t0 = esp_timer_get_time();
    s_rb_checked = s_rb_matched = 0;
    s_sent_n = 0;

    /* 1. Nothing else may be driving MISO, or selecting this chip would put
     *    its 1 mA output against whatever is. */
    s_stage = STAGE_MISO;
    const bool force = s_force_next;
    s_force_next = false;
    const miso_t m = miso_check();
    if (m != MISO_RELEASED) {
        s_miso = m;
        if (!force) return WHY_MISO_HELD;
        ESP_LOGW(TAG, "MISO %s - check SKIPPED by FLOW,FORCE", MISO_NAME[m]);
        s_miso = MISO_FORCED;
    } else {
        s_miso = m;
    }

    /* 2. Who is there? Before touching anything. */
    s_stage = STAGE_ID;
    if (!read_id()) return WHY_NO_CHIP;

    /* 3. Power-up reset: the S3 can reboot without the sensor losing power,
     *    so the chip may be in any state a half-finished transaction left. */
    s_stage = STAGE_RESET;
    reg_write(PMW3901_REG_POWER_UP_RESET, PMW3901_RESET_MAGIC);
    const int64_t t_reset = esp_timer_get_time();
    vTaskDelay(pdMS_TO_TICKS(RESET_SETTLE_MS));
    if (!read_id()) return WHY_POST_RESET;

    /* 4. Read the motion registers once, as the power-up sequence says. */
    uint8_t dummy;
    for (uint8_t r = PMW3901_REG_MOTION; r <= PMW3901_REG_DELTA_Y_H; r++)
        reg_read(r, &dummy);

    /* 5. Vendor configuration. */
    s_stage = STAGE_TABLES;
    uint8_t bank = 0;
    bool ok_b[PMW3901_INIT_B_LEN] = {0};
    run_prelude();
    run_table(pmw3901_init_a, pmw3901_init_a_len, &bank, NULL);
    vTaskDelay(pdMS_TO_TICKS(10));
    run_table(pmw3901_init_b, pmw3901_init_b_len, &bank, ok_b);
    pick_sentinels(ok_b);

    /* 6. Still the chip we started with, and back in bank 0 (the ID
     *    register is only the ID register in bank 0). */
    if (!read_id()) return WHY_POST_RESET;

    /* 7. tMOT-RST. */
    while (esp_timer_get_time() - t_reset < MOTION_AFTER_RESET_US)
        vTaskDelay(pdMS_TO_TICKS(5));

    s_init_ms = (uint32_t)((esp_timer_get_time() - t0) / 1000);
    s_stage = STAGE_DONE;
    return WHY_NONE;
}

/* ===================== running ===================== */

static why_t liveness_check(void)
{
    /* Identity: the proof that the chip is on the bus and answering with
     * its own bytes, not a stuck line. */
    if (read_id()) {
        s_id_fail_streak = 0;
        s_id_ok_now = true;
    } else {
        s_id_fail_total++;
        s_id_ok_now = false;
        if (++s_id_fail_streak >= ID_FAIL_LIMIT) return WHY_ID_LOST;
    }

    if (s_sent_n == 2) {
        int bad = 0;
        for (int j = 0; j < 2; j++) {
            uint8_t v = 0;
            if (reg_read(s_sent_reg[j], &v) != ESP_OK || v != s_sent_val[j]) bad++;
        }
        if (bad == 2) {
            if (++s_sent_streak >= SENTINEL_FAIL_LIMIT) {
                s_sensor_resets++;
                return WHY_SENSOR_RESET;
            }
        } else {
            if (bad == 1) s_sent_disagree++;
            s_sent_streak = 0;
        }
    }

    /* Observation: clear it and see whether the next frames set it. The
     * chip's own "am I running frames" register. DIAGNOSTIC only until a
     * bench run shows how this part uses it - a check that has never been
     * seen to pass must not be allowed to fail anything. */
    if (reg_write(PMW3901_REG_OBSERVATION, 0x00) == ESP_OK) {
        s_obs_armed = true;
        s_obs_wait = 0;
    }
    return WHY_NONE;
}

static why_t run_loop(void)
{
    uint8_t raw[PMW3901_BURST_LEN];
    int64_t prev_latch = 0;
    uint32_t n = 0, err_streak = 0;

    portENTER_CRITICAL(&s_mux);
    pmw3901_track_restart(&s_track);
    portEXIT_CRITICAL(&s_mux);
    s_id_fail_streak = 0;
    s_id_ok_now = true;
    s_obs_armed = false;

    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(PMW3901_PERIOD_MS));

        int64_t t_latch = 0;
        uint32_t wait_us = 0;
        bool latched = false;
        const esp_err_t err = burst_read(raw, &t_latch, &wait_us, &latched);
        if (wait_us > s_wait_max_us) s_wait_max_us = wait_us;

        if (err != ESP_OK) {
            /* Latched but unread: the counts are gone, the window is over. */
            if (latched) {
                prev_latch = t_latch;
                portENTER_CRITICAL(&s_mux);
                pmw3901_track_note_lost(&s_track);
                portEXIT_CRITICAL(&s_mux);
            }
            if (++err_streak >= 10) return WHY_SPI_ERR;
            continue;
        }
        err_streak = 0;

        pmw3901_burst_t b;
        pmw3901_burst_decode(raw, &b);
        const uint32_t dt = prev_latch ? (uint32_t)(t_latch - prev_latch) : 0;
        prev_latch = t_latch;
        if (dt > s_dt_max_us) s_dt_max_us = dt;

        portENTER_CRITICAL(&s_mux);
        const uint16_t f = pmw3901_track_update(&s_track, raw, &b, dt);
        s_seq++;
        s_latest.t_us        = t_latch;
        s_latest.dt_us       = dt;
        s_latest.seq         = s_seq;
        s_latest.b           = b;
        s_latest.flags       = f;
        s_latest.bus_wait_us = wait_us;
        s_have_latest = true;
        portEXIT_CRITICAL(&s_mux);

        if (s_obs_armed) {
            if (b.observation != 0) {
                s_obs_pass++;
                s_obs_last = b.observation;
                s_obs_armed = false;
            } else if (++s_obs_wait >= OBS_WAIT_BURSTS) {
                s_obs_fail++;
                s_obs_armed = false;
            }
        }

        if (++n % LIVENESS_EVERY == 0) {
            const why_t w = liveness_check();
            if (w != WHY_NONE) return w;
        }

        /* After a long bus stall, do not fire a string of back-to-back
         * catch-up reads: the chip accumulated through the stall, and one
         * read collects all of it. */
        const TickType_t now = xTaskGetTickCount();
        if (now - wake > pdMS_TO_TICKS(4 * PMW3901_PERIOD_MS)) wake = now;
    }
}

/* ===================== task ===================== */

static void log_failure(why_t w)
{
    switch (w) {
    case WHY_MISO_HELD:
        ESP_LOGE(TAG, "SPI3 MISO (GPIO%d) is %s with every chip select "
                      "released - another device is driving it. The PMW3901 "
                      "will NOT be selected. On this aircraft that is the SD "
                      "module's 74LVC125 (Catalex design: MISO buffer output "
                      "enable tied to ground). Fix the module or replace it; "
                      "see pmw3901.h. FLOW,FORCE overrides once.",
                 s_cfg.pin_miso, MISO_NAME[s_miso]);
        break;
    case WHY_NO_CHIP:
    case WHY_POST_RESET:
        ESP_LOGW(TAG, "%s: ID 0x%02X/0x%02X - %s%s", WHY_NAME[w], s_id, s_inv,
                 pmw3901_id_verdict_str(s_id_verdict),
                 (s_id_verdict == PMW_ID_NONE_00 || s_id_verdict == PMW_ID_NONE_FF)
                     ? ". Expected on USB-only power: the sensor rail is "
                       "battery-fed." : "");
        break;
    default:
        ESP_LOGW(TAG, "%s", WHY_NAME[w]);
        break;
    }
}

static void retry_wait(why_t w)
{
    static why_t prev = WHY_NONE;
    static uint32_t same = 0;
    same = (w == prev) ? same + 1 : 0;
    prev = w;
    uint32_t ms = 2000;
    if (w == WHY_MISO_HELD) ms = 5000;
    else if (same >= 5)     ms = 10000;   /* e.g. USB-only: no rail, no hurry */
    for (uint32_t t = 0; t < ms && !s_force_next; t += 100)
        vTaskDelay(pdMS_TO_TICKS(100));
}

static void flow_task(void *arg)
{
    (void)arg;

    /* Rule 1 of pmw3901.h: not one clock until the card is in SPI mode. */
    s_state = ST_WAIT_BUS;
    for (uint32_t waited = 0; !s_cfg.bus_settled(5000); ) {
        waited += 5;
        if (waited % 30 == 0)
            ESP_LOGW(TAG, "still waiting for SPI3 after %u s: the blackbox has "
                          "not finished its first SD probe", (unsigned)waited);
    }

    /* Claim CS. The bus owner already drives it high; level before
     * direction, so there is no instant at which it could read low. */
    gpio_set_level(s_cfg.pin_cs, 1);
    const gpio_config_t io = {
        .pin_bit_mask = 1ULL << s_cfg.pin_cs,
        .mode         = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);
    gpio_set_level(s_cfg.pin_cs, 1);

    const spi_device_interface_config_t dc = {
        .clock_speed_hz = PMW3901_SPI_HZ,
        .mode           = 3,
        .spics_io_num   = -1,   /* by hand: see reg_write() */
        .queue_size     = 1,
    };
    while (spi_bus_add_device(s_cfg.host, &dc, &s_dev) != ESP_OK) {
        ESP_LOGE(TAG, "cannot add the flow device to SPI%d - retrying in 5 s",
                 (int)s_cfg.host + 1);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    spi_device_get_actual_freq(s_dev, &s_clk_khz);
    if (s_clk_khz <= 0 || s_clk_khz > PMW3901_SPI_HZ / 1000) {
        s_why = WHY_CLOCK;
        s_state = ST_RETRY;
        ESP_LOGE(TAG, "SPI clock came out at %d kHz, above the %d kHz "
                      "datasheet maximum - flow sensor NOT started",
                 s_clk_khz, PMW3901_SPI_HZ / 1000);
        vTaskDelete(NULL);
    }

    why_t last_logged = WHY_NONE;
    int64_t last_log_us = 0;
    for (;;) {
        s_state = ST_INIT;
        why_t w = chip_init();
        if (w == WHY_NONE) {
            s_init_ok++;
            s_why = WHY_NONE;
            last_logged = WHY_NONE;
            ESP_LOGI(TAG, "up: ID 0x%02X/0x%02X, SPI%d %d kHz mode 3, CS %d, "
                          "init %u ms, read-back %u/%u matched, reset "
                          "sentinels %s%s",
                     s_id, s_inv, (int)s_cfg.host + 1, s_clk_khz, s_cfg.pin_cs,
                     (unsigned)s_init_ms, (unsigned)s_rb_matched,
                     (unsigned)s_rb_checked, s_sent_n ? "ON" : "OFF",
                     s_init_ok > 1 ? " (RE-INIT)" : "");
            s_state = ST_RUN;
            w = run_loop();
            s_id_ok_now = false;
            ESP_LOGW(TAG, "stopped: %s - re-initialising", WHY_NAME[w]);
        }
        s_why = w;
        s_state = ST_RETRY;
        const int64_t now = esp_timer_get_time();
        if (w != last_logged || now - last_log_us > 60000000LL) {
            log_failure(w);
            last_logged = w;
            last_log_us = now;
        }
        retry_wait(w);
    }
}

/* ===================== public ===================== */

void pmw3901_start(const pmw3901_cfg_t *cfg)
{
    if (!cfg || !cfg->bus_settled) {
        /* Without it the SD ordering rule cannot be kept - refuse rather
         * than risk talking over an unprobed card. */
        ESP_LOGE(TAG, "no bus_settled hook - flow sensor NOT started");
        return;
    }
    s_cfg = *cfg;
    pmw3901_track_init(&s_track);
    if (xTaskCreatePinnedToCore(flow_task, "flow", 4096, NULL, 2, NULL,
                                FC_CORE_SUPPORT) != pdPASS) {
        ESP_LOGE(TAG, "task create failed - flow sensor NOT started");
    }
}

bool pmw3901_alive(void)
{
    return s_state == ST_RUN && s_id_ok_now;
}

bool pmw3901_latest(pmw3901_sample_t *out)
{
    portENTER_CRITICAL(&s_mux);
    const bool have = s_have_latest;
    if (have) *out = s_latest;
    portEXIT_CRITICAL(&s_mux);
    return have;
}

void pmw3901_totals(pmw3901_totals_t *out)
{
    portENTER_CRITICAL(&s_mux);
    out->sum_dx  = s_track.sum_dx;
    out->sum_dy  = s_track.sum_dy;
    out->seq     = s_seq;
    out->n_valid = s_track.n_valid;
    out->n_lost  = s_track.n_lost;
    portEXIT_CRITICAL(&s_mux);
    out->reinits = s_init_ok > 1 ? s_init_ok - 1 : 0;
    out->running = s_state == ST_RUN;
}

void pmw3901_force_next_init(void)
{
    s_force_next = true;
}

bool pmw3901_status_line(int part, char *buf, size_t n)
{
    switch (part) {
    case 0:
        if (!s_id_read) {
            snprintf(buf, n, "flow: %s%s%s | SPI%d %d kHz mode 3, CS GPIO%d | "
                             "ID never read | MISO %s",
                     STATE_NAME[s_state],
                     s_state == ST_RETRY ? ": " : "",
                     s_state == ST_RETRY ? WHY_NAME[s_why] : "",
                     (int)s_cfg.host + 1, s_clk_khz, s_cfg.pin_cs,
                     MISO_NAME[s_miso]);
            return true;
        }
        snprintf(buf, n, "flow: %s%s%s | SPI%d %d kHz mode 3, CS GPIO%d | "
                         "ID %02X/%02X (%s) | MISO %s",
                 STATE_NAME[s_state],
                 s_state == ST_RETRY ? ": " : "",
                 s_state == ST_RETRY ? WHY_NAME[s_why] : "",
                 (int)s_cfg.host + 1, s_clk_khz, s_cfg.pin_cs, s_id, s_inv,
                 pmw3901_id_verdict_str(s_id_verdict), MISO_NAME[s_miso]);
        return true;
    case 1:
        if (s_stage < STAGE_TABLES) {
            /* The table fields below would be zeros from steps that never
             * ran - say where it stopped instead. */
            snprintf(buf, n, "flow init: %u ok / %u tries - the last attempt "
                             "stopped at %s",
                     (unsigned)s_init_ok, (unsigned)s_init_tries,
                     STAGE_NAME[s_stage]);
            return true;
        }
        snprintf(buf, n, "flow init: %u ok / %u tries, last %u ms | prelude "
                         "0x67=%02X 0x73=%02X%s c1=%02X c2=%02X | read-back "
                         "%u/%u matched | reset sentinels %s",
                 (unsigned)s_init_ok, (unsigned)s_init_tries,
                 (unsigned)s_init_ms, s_pre_67, s_pre_73,
                 s_pre_cal ? " (cal applied)" : "", s_pre_c1, s_pre_c2,
                 (unsigned)s_rb_matched, (unsigned)s_rb_checked,
                 s_sent_n == 2 ? "ON" : "OFF (fewer than 2 held still)");
        return true;
    case 2: {
        pmw3901_track_t t;
        portENTER_CRITICAL(&s_mux);
        t = s_track;
        portEXIT_CRITICAL(&s_mux);
        snprintf(buf, n, "flow data: %u bursts, %u valid, %u lost (stuck %u, "
                         "torn %u, railed %u, rate %u, dt %u) | identical-run "
                         "max %u | ID fails %u | chip resets %u, sentinel "
                         "disagreements %u",
                 (unsigned)t.n_bursts, (unsigned)t.n_valid, (unsigned)t.n_lost,
                 (unsigned)t.n_stuck, (unsigned)t.n_inconsistent,
                 (unsigned)t.n_saturated, (unsigned)t.n_rate,
                 (unsigned)t.n_bad_dt, (unsigned)t.same_run_max,
                 (unsigned)s_id_fail_total, (unsigned)s_sensor_resets,
                 (unsigned)s_sent_disagree);
        return true;
    }
    case 3:
        snprintf(buf, n, "flow timing: window max %u us, SPI3 wait max %u us, "
                         "latch +-%u us | observation self-test %u pass / %u "
                         "fail (last 0x%02X) | SPI errors %u",
                 (unsigned)s_dt_max_us, (unsigned)s_wait_max_us,
                 (unsigned)s_latch_unc_max_us, (unsigned)s_obs_pass,
                 (unsigned)s_obs_fail, s_obs_last, (unsigned)s_spi_errs);
        return true;
    case 4: {
        pmw3901_sample_t s;
        if (!pmw3901_latest(&s)) {
            snprintf(buf, n, "flow last: no sample yet");
            return true;
        }
        /* The age goes first: a value without its age is how a dead sensor
         * passes for a still one. */
        snprintf(buf, n, "flow last: %u ms ago, seq %u, window %u us, dx %d "
                         "dy %d, squal %u, shutter %u, raw %u..%u sum %u, "
                         "motion 0x%02X, obs 0x%02X, flags 0x%03X%s",
                 (unsigned)((esp_timer_get_time() - s.t_us) / 1000),
                 (unsigned)s.seq, (unsigned)s.dt_us, s.b.dx, s.b.dy,
                 s.b.squal, s.b.shutter, s.b.raw_min, s.b.raw_max,
                 s.b.raw_sum, s.b.motion, s.b.observation, s.flags,
                 (s.flags & PMW_F_VALID) ? " VALID" : " not valid");
        return true;
    }
    case 5: {
        const uint32_t bad = s_rb_checked - s_rb_matched;
        if (bad == 0) return false;
        int k = snprintf(buf, n, "flow read-back mismatches (%u):", (unsigned)bad);
        for (uint32_t i = 0; i < bad && i < MISMATCH_KEEP && k > 0 && (size_t)k < n; i++)
            k += snprintf(buf + k, n - (size_t)k, " bank %02X reg %02X wrote %02X read %02X;",
                          s_mm[i].bank, s_mm[i].reg, s_mm[i].want, s_mm[i].got);
        return true;
    }
    default:
        return false;
    }
}
