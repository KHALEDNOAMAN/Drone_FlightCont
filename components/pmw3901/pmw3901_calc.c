#include "pmw3901_calc.h"

/* FREESTANDING: no libc. See pmw3901_calc.h. */

void pmw3901_burst_decode(const uint8_t b[PMW3901_BURST_LEN],
                          pmw3901_burst_t *o)
{
    o->motion      = b[0];
    o->observation = b[1];
    o->dx          = (int16_t)(uint16_t)(((uint16_t)b[3] << 8) | b[2]);
    o->dy          = (int16_t)(uint16_t)(((uint16_t)b[5] << 8) | b[4]);
    o->squal       = b[6];
    o->raw_sum     = b[7];
    o->raw_max     = b[8];
    o->raw_min     = b[9];
    o->shutter     = (uint16_t)(((uint16_t)b[10] << 8) | b[11]);
}

void pmw3901_track_init(pmw3901_track_t *t)
{
    t->sum_dx = 0;
    t->sum_dy = 0;
    t->n_bursts = t->n_valid = t->n_lost = 0;
    t->n_stuck = t->n_inconsistent = t->n_saturated = 0;
    t->n_rate = t->n_bad_dt = 0;
    t->same_run = t->same_run_max = 0;
    for (int i = 0; i < PMW3901_BURST_LEN; i++) t->prev[i] = 0;
    t->have_prev = 0;
    t->first_pending = 1;
    t->prev_lost = 0;
}

void pmw3901_track_restart(pmw3901_track_t *t)
{
    t->first_pending = 1;
    t->have_prev = 0;
    t->same_run = 0;
}

static uint32_t mag16(int16_t v)
{
    /* via int32: -INT16_MIN does not fit an int16 */
    int32_t w = v;
    return (uint32_t)(w < 0 ? -w : w);
}

uint16_t pmw3901_track_update(pmw3901_track_t *t,
                              const uint8_t raw[PMW3901_BURST_LEN],
                              const pmw3901_burst_t *b, uint32_t dt_us)
{
    uint16_t f = 0;
    t->n_bursts++;
    if (b->motion & PMW3901_MOTION_BIT) f |= PMW_F_MOTION;

    int same = t->have_prev, all00 = 1, allff = 1;
    for (int i = 0; i < PMW3901_BURST_LEN; i++) {
        if (raw[i] != t->prev[i]) same = 0;
        if (raw[i] != 0x00) all00 = 0;
        if (raw[i] != 0xFF) allff = 0;
        t->prev[i] = raw[i];
    }
    t->have_prev = 1;
    t->same_run = same ? t->same_run + 1 : 0;
    if (t->same_run > t->same_run_max) t->same_run_max = t->same_run;

    if (all00 || allff) {
        f |= PMW_F_BUS_STUCK;
        t->n_stuck++;
    }
    if (b->raw_min > b->raw_max) {
        f |= PMW_F_INCONSISTENT;
        t->n_inconsistent++;
    }
    if (b->dx == INT16_MAX || b->dx == INT16_MIN ||
        b->dy == INT16_MAX || b->dy == INT16_MIN) {
        f |= PMW_F_SATURATED;
        t->n_saturated++;
    }

    if (t->first_pending) {
        /* Counts since the reset: an unknown window, discarded whatever
         * they say. The integral starts at this latch, so this is not a
         * hole and the next sample is not AFTER_LOST. */
        t->first_pending = 0;
        t->prev_lost = 0;
        return (uint16_t)(f | PMW_F_FIRST);
    }

    if (dt_us == 0 || dt_us > PMW3901_DT_MAX_US) {
        f |= PMW_F_BAD_DT;
        t->n_bad_dt++;
    } else {
        /* 64-bit: 12000 * dt overflows 32 bits past ~0.36 s, and a window
         * that long is exactly what an SD stall produces. */
        const uint64_t lim =
            (uint64_t)PMW3901_MAX_COUNTS_PER_S * dt_us / 1000000u +
            PMW3901_RATE_SLACK_COUNTS;
        if (mag16(b->dx) > lim || mag16(b->dy) > lim) {
            f |= PMW_F_RATE;
            t->n_rate++;
        }
    }

    if (t->prev_lost) f |= PMW_F_AFTER_LOST;

    if (f & PMW_F_REJECT_MASK) {
        /* The chip cleared these counts when it latched them, so whatever
         * really happened in this window is gone for good. */
        t->n_lost++;
        t->prev_lost = 1;
    } else {
        f |= PMW_F_VALID;
        t->n_valid++;
        t->sum_dx += b->dx;
        t->sum_dy += b->dy;
        t->prev_lost = 0;
    }
    return f;
}

void pmw3901_track_note_lost(pmw3901_track_t *t)
{
    t->n_bursts++;
    if (t->first_pending) {
        t->first_pending = 0;
        t->prev_lost = 0;
        return;
    }
    t->n_lost++;
    t->prev_lost = 1;
}

pmw3901_id_verdict_t pmw3901_id_check(uint8_t id, uint8_t inv)
{
    if (id == PMW3901_PRODUCT_ID && inv == PMW3901_INV_PRODUCT_ID)
        return PMW_ID_OK;
    if (id == 0x00 && inv == 0x00) return PMW_ID_NONE_00;
    if (id == 0xFF && inv == 0xFF) return PMW_ID_NONE_FF;
    /* One bit late: the true byte shifted right, some earlier bit on top.
     * One bit early: shifted left, the next bit at the bottom. Both bytes
     * must show the SAME shift - one alone is just a bad read. */
    const int late  = (id  & 0x7F) == (PMW3901_PRODUCT_ID     >> 1) &&
                      (inv & 0x7F) == (PMW3901_INV_PRODUCT_ID >> 1);
    const int early = (id  & 0xFE) == ((PMW3901_PRODUCT_ID     << 1) & 0xFE) &&
                      (inv & 0xFE) == ((PMW3901_INV_PRODUCT_ID << 1) & 0xFE);
    if (late || early) return PMW_ID_SHIFTED;
    if (id == PMW3901_PRODUCT_ID) return PMW_ID_INV_BAD;
    return PMW_ID_WRONG;
}

const char *pmw3901_id_verdict_str(pmw3901_id_verdict_t v)
{
    switch (v) {
    case PMW_ID_OK:      return "ok";
    case PMW_ID_NONE_00: return "MISO held LOW or chip unpowered";
    case PMW_ID_NONE_FF: return "nothing drove MISO (chip absent/unpowered, "
                                "MISO or CS not connected)";
    case PMW_ID_SHIFTED: return "bytes shifted by one bit - SPI mode or "
                                "clock-idle level, not wiring";
    case PMW_ID_INV_BAD: return "inverse ID wrong - unreliable read";
    case PMW_ID_WRONG:   return "not a PMW3901 (or garbage)";
    }
    return "?";
}

/* PixArt demo kit V3.20 - see pmw3901_calc.h for the source and for how it
 * differs from the older table. Transcribed from PX4's PMW3901.cpp and
 * checked entry by entry against it. */
const pmw3901_reg_t pmw3901_init_a[] = {
    {0x7F, 0x00}, {0x61, 0xAD}, {0x7F, 0x03}, {0x40, 0x00}, {0x7F, 0x05},
    {0x41, 0xB3}, {0x43, 0xF1}, {0x45, 0x14}, {0x5B, 0x32}, {0x5F, 0x34},
    {0x7B, 0x08}, {0x7F, 0x06}, {0x44, 0x1B}, {0x40, 0xBF}, {0x4E, 0x3F},
    {0x7F, 0x08}, {0x65, 0x20}, {0x6A, 0x18}, {0x7F, 0x09}, {0x4F, 0xAF},
    {0x5F, 0x40}, {0x48, 0x80}, {0x49, 0x80}, {0x57, 0x77}, {0x60, 0x78},
    {0x61, 0x78}, {0x62, 0x08}, {0x63, 0x50}, {0x7F, 0x0A}, {0x45, 0x60},
    {0x7F, 0x00}, {0x4D, 0x11}, {0x55, 0x80}, {0x74, 0x21}, {0x75, 0x1F},
    {0x4A, 0x78}, {0x4B, 0x78}, {0x44, 0x08}, {0x45, 0x50}, {0x64, 0xFF},
    {0x65, 0x1F}, {0x7F, 0x14}, {0x65, 0x67}, {0x66, 0x08}, {0x63, 0x70},
    {0x7F, 0x15}, {0x48, 0x48}, {0x7F, 0x07}, {0x41, 0x0D}, {0x43, 0x14},
    {0x4B, 0x0E}, {0x45, 0x0F}, {0x44, 0x42}, {0x4C, 0x80}, {0x7F, 0x10},
    {0x5B, 0x02}, {0x7F, 0x07}, {0x40, 0x41}, {0x70, 0x00},
};
_Static_assert(sizeof(pmw3901_init_a) / sizeof(pmw3901_init_a[0]) ==
               PMW3901_INIT_A_LEN, "table A length != PMW3901_INIT_A_LEN");
const unsigned pmw3901_init_a_len = PMW3901_INIT_A_LEN;

const pmw3901_reg_t pmw3901_init_b[] = {
    {0x32, 0x44}, {0x7F, 0x07}, {0x40, 0x40}, {0x7F, 0x06}, {0x62, 0xF0},
    {0x63, 0x00}, {0x7F, 0x0D}, {0x48, 0xC0}, {0x6F, 0xD5}, {0x7F, 0x00},
    {0x5B, 0xA0}, {0x4E, 0xA8}, {0x5A, 0x50}, {0x40, 0x80},
};
_Static_assert(sizeof(pmw3901_init_b) / sizeof(pmw3901_init_b[0]) ==
               PMW3901_INIT_B_LEN, "table B length != PMW3901_INIT_B_LEN");
const unsigned pmw3901_init_b_len = PMW3901_INIT_B_LEN;

uint8_t pmw3901_bank_after(const pmw3901_reg_t *tbl, unsigned n,
                           uint8_t bank_in)
{
    uint8_t bank = bank_in;
    for (unsigned i = 0; i < n; i++)
        if (tbl[i].reg == PMW3901_REG_BANK) bank = tbl[i].val;
    return bank;
}

uint8_t pmw3901_prelude_c1(uint8_t raw)
{
    unsigned v = raw;
    v += (v <= 28u) ? 14u : 11u;
    return (uint8_t)(v > 0x3Fu ? 0x3Fu : v);
}

uint8_t pmw3901_prelude_c2(uint8_t raw)
{
    return (uint8_t)(((unsigned)raw * 45u) / 100u);
}
