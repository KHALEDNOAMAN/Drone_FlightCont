#ifndef PMW3901_CALC_H
#define PMW3901_CALC_H

/* PMW3901MB optical flow - the pure half: the register map, the vendor init
 * tables, motion-burst decoding, the per-sample checks, and the bookkeeping
 * that turns a stream of bursts into an integral a filter can difference.
 *
 * FREESTANDING - stdint only, no libc, no ESP-IDF - so tools/pmw3901_test.py
 * compiles exactly this file with clang and drives it from Python. The SPI
 * side, the timing and the task live in pmw3901.c.
 *
 * WHAT A SAMPLE IS. The chip accumulates motion counts internally and a
 * motion-burst read LATCHES and CLEARS them. So one burst is the motion over
 * the window since the previous burst - whatever that window's length
 * actually was. The flow task cannot keep a fixed rate: it shares SPI3 with
 * the SD card, and a card write holds the bus for ~3.5 ms, card housekeeping
 * for tens to hundreds. A consumer that assumed 10 ms windows would turn
 * every card stall into a fabricated velocity spike ("a constant that
 * counts SAMPLES lies about TIME"). Every sample therefore carries the
 * length of its own window, and the counts are also kept as ABSOLUTE running
 * sums - the accz_dv_ms rule: differencing an absolute sum is exact across
 * any number of dropped samples, a per-sample delta is not. */

#include <stdint.h>

/* ---- Register map (datasheet v1.10 table 7; the rest is proprietary) ---- */
#define PMW3901_REG_PRODUCT_ID      0x00   /* reads 0x49 */
#define PMW3901_REG_MOTION          0x02
#define PMW3901_REG_DELTA_X_L       0x03
#define PMW3901_REG_DELTA_X_H       0x04
#define PMW3901_REG_DELTA_Y_L       0x05
#define PMW3901_REG_DELTA_Y_H       0x06
#define PMW3901_REG_OBSERVATION     0x15   /* R/W, see pmw3901.c */
#define PMW3901_REG_MOTION_BURST    0x16
#define PMW3901_REG_POWER_UP_RESET  0x3A   /* write 0x5A */
#define PMW3901_REG_INV_PRODUCT_ID  0x5F   /* reads 0xB6 = ~0x49 */
#define PMW3901_REG_BANK            0x7F   /* proprietary bank select */

#define PMW3901_PRODUCT_ID          0x49
#define PMW3901_INV_PRODUCT_ID      0xB6
#define PMW3901_RESET_MAGIC         0x5A

#define PMW3901_BURST_LEN           12

/* ---- Motion burst, in wire order ----
 * Motion, Observation, Delta_X_L, Delta_X_H, Delta_Y_L, Delta_Y_H, Squal,
 * RawData_Sum, Maximum_RawData, Minimum_RawData, Shutter_Upper,
 * Shutter_Lower. The deltas are little-endian and the shutter is BIG-endian
 * - two orders in one 12-byte read, which is exactly the kind of thing a
 * struct overlay gets wrong (Crazyflie reads it into a packed struct and
 * then has to byte-swap the shutter by hand). Decoded field by field here
 * instead. */
typedef struct {
    uint8_t  motion;       /* bit 7 = motion since the last read */
    uint8_t  observation;
    int16_t  dx, dy;       /* counts, SENSOR frame, not remapped */
    uint8_t  squal;        /* surface quality: features the chip is tracking */
    uint8_t  raw_sum;      /* image brightness statistics */
    uint8_t  raw_max;
    uint8_t  raw_min;
    uint16_t shutter;      /* exposure, clock cycles; rises as light falls */
} pmw3901_burst_t;

#define PMW3901_MOTION_BIT          0x80u

void pmw3901_burst_decode(const uint8_t b[PMW3901_BURST_LEN],
                          pmw3901_burst_t *out);

/* ---- Per-sample verdict ----
 * PMW_F_VALID is the only bit a consumer must look at: the counts in this
 * sample are motion over a known window. Everything else says why not, or
 * is information.
 *
 * Deliberately NOT a reason to reject: low squal and high shutter. Those
 * describe the SCENE (dark room, blank floor, too close to focus), not the
 * data path, the thresholds for this airframe are not known yet, and the
 * passive-log stage exists to measure them. They are logged raw. */
#define PMW_F_VALID        (1u << 0)
#define PMW_F_MOTION       (1u << 1)  /* the chip's own motion bit */
/* First burst after a (re)init. Its counts cover an unknown window that
 * began at the chip reset, so they are discarded by design. The integral
 * (re)starts at this sample's latch. */
#define PMW_F_FIRST        (1u << 2)
/* All twelve bytes 0x00 or all 0xFF: nothing drove MISO. A dead bus reads
 * as "perfectly still" (0x00) or as dx=dy=-1 with squal 255 (0xFF) -
 * both plausible-looking, which is why it has its own bit. */
#define PMW_F_BUS_STUCK    (1u << 3)
/* Minimum_RawData > Maximum_RawData. Impossible for a real frame, free to
 * check, and it catches bytes that are merely shifted or torn. */
#define PMW_F_INCONSISTENT (1u << 4)
#define PMW_F_SATURATED    (1u << 5)  /* a delta sitting on an int16 rail */
/* More counts than the optics can produce in this window. REJECTED, never
 * clamped. */
#define PMW_F_RATE         (1u << 6)
/* The window before this one was lost, so the running sum has a hole
 * between the previous sample and this one. This sample itself is fine. */
#define PMW_F_AFTER_LOST   (1u << 7)
/* Window of zero length, or longer than PMW3901_DT_MAX_US. */
#define PMW_F_BAD_DT       (1u << 8)

#define PMW_F_REJECT_MASK  (PMW_F_BUS_STUCK | PMW_F_INCONSISTENT | \
                            PMW_F_SATURATED | PMW_F_RATE | PMW_F_BAD_DT)

/* Rate bound. Datasheet: 7.4 rad/s maximum. Counts per radian are NOT
 * established for this module yet - published scalings run from 1/794 rad
 * per count (ArduPilot, 1.26e-3) to 1/385 (PX4) - so 7.4 rad/s is 2850 to
 * 5870 counts/s. 12000 is ~2x the top of that: a garbage detector, not a
 * physics limit. For scale, Crazyflie drops samples over 100 counts per
 * 10 ms (10000/s) and PX4 over 240 per 10 ms (24000/s). */
#define PMW3901_MAX_COUNTS_PER_S    12000u
#define PMW3901_RATE_SLACK_COUNTS   2u
/* 2 s: past this a window at the rate bound could already have filled an
 * int16 accumulator, and a window that long is useless for rate anyway. */
#define PMW3901_DT_MAX_US           2000000u

typedef struct {
    /* the integral, VALID windows only. int32 lasts ~50 h at the rate bound */
    int32_t  sum_dx, sum_dy;
    uint32_t n_bursts;          /* every burst that reached update() */
    uint32_t n_valid;
    uint32_t n_lost;            /* windows whose counts are gone */
    uint32_t n_stuck, n_inconsistent, n_saturated, n_rate, n_bad_dt;
    /* Consecutive byte-identical bursts. A DIAGNOSTIC, not a fault: a live
     * chip over a still, evenly lit scene can repeat itself. The real
     * liveness checks are in pmw3901.c (ID read-back, reset sentinel). */
    uint32_t same_run, same_run_max;
    uint8_t  prev[PMW3901_BURST_LEN];
    uint8_t  have_prev;
    uint8_t  first_pending;
    uint8_t  prev_lost;
} pmw3901_track_t;

/* Zero everything. Boot only. */
void pmw3901_track_init(pmw3901_track_t *t);

/* After a chip (re)init: the next burst is PMW_F_FIRST. The running sums
 * and counters are KEPT, so a consumer differencing sums across a re-init
 * still gets a correct answer; the lost window shows up in n_lost. */
void pmw3901_track_restart(pmw3901_track_t *t);

/* Classify one burst. raw is the 12 bytes as read, b their decode, dt_us
 * the time since the previous burst's latch (ignored on the FIRST burst).
 * Returns PMW_F_* flags; adds to the sums only when PMW_F_VALID. */
uint16_t pmw3901_track_update(pmw3901_track_t *t,
                              const uint8_t raw[PMW3901_BURST_LEN],
                              const pmw3901_burst_t *b, uint32_t dt_us);

/* A burst whose address byte went out but whose data never arrived: the
 * chip cleared its counters, so that window is gone and the next sample is
 * AFTER_LOST. If it was the FIRST burst, nothing is lost - the integral
 * simply starts at that latch, and the next sample is an ordinary one. */
void pmw3901_track_note_lost(pmw3901_track_t *t);

/* ---- Identity ----
 * Product_ID and Inverse_Product_ID read together, and the verdict says
 * more than yes/no, because the failure pattern names the fault. */
typedef enum {
    PMW_ID_OK = 0,
    PMW_ID_NONE_00,   /* both 0x00: MISO held low, or the chip unpowered */
    PMW_ID_NONE_FF,   /* both 0xFF: nothing drove MISO (the pull-up won) */
    PMW_ID_SHIFTED,   /* both off by exactly one bit position: SPI mode or
                         clock-idle level wrong, not wiring */
    PMW_ID_INV_BAD,   /* ID right, inverse wrong: a flaky read */
    PMW_ID_WRONG,     /* something answered, and it is not a PMW3901 */
} pmw3901_id_verdict_t;

pmw3901_id_verdict_t pmw3901_id_check(uint8_t id, uint8_t inv_id);
const char *pmw3901_id_verdict_str(pmw3901_id_verdict_t v);

/* ---- Vendor init ----
 * "Performance optimisation registers": proprietary, undocumented in the
 * public datasheet, and REQUIRED - PixArt's words are that they must be
 * written after power-up "to achieve optimum performance".
 *
 * SOURCE: the PixArt PMW3901MB demo kit V3.20 (21) sequence, as
 * carried by PX4 (src/drivers/optical_flow/pmw3901/PMW3901.cpp, BSD-3).
 * Chosen over the older datasheet-era table that the Bitcraze Arduino
 * library and ArduPilot still ship because it is PixArt's later revision
 * and it adds a per-chip calibration prelude the old one lacks. The two
 * differ in 0x74 (0x21 vs 0x1F), bank 0x14 0x65/0x63 (0x67/0x70 vs
 * 0x60/0x78), bank 0x15 0x48 (0x48 vs 0x58) and the mid-table pause
 * (10 ms vs 100 ms). If squal is poor on a well-lit textured floor, the old
 * table is the first thing to diff against.
 *
 * Table A, then a 10 ms pause, then table B. The prelude (pmw3901.c) runs
 * first and needs two register reads, so it is code, not a table. */
typedef struct {
    uint8_t reg;
    uint8_t val;
} pmw3901_reg_t;

/* Compile-time lengths, _Static_assert'ed against the tables themselves in
 * pmw3901_calc.c, so an edited table cannot silently outgrow an array
 * sized from these (pmw3901.c keeps a per-entry verdict for table B). */
#define PMW3901_INIT_A_LEN 59
#define PMW3901_INIT_B_LEN 14

extern const pmw3901_reg_t pmw3901_init_a[];
extern const unsigned      pmw3901_init_a_len;
extern const pmw3901_reg_t pmw3901_init_b[];
extern const unsigned      pmw3901_init_b_len;

/* The bank selected after running n entries of a table from bank_in.
 * Table B MUST end in bank 0: the motion burst (0x16), the ID registers and
 * the reset sentinel are all read in bank 0, and nothing re-selects it. */
uint8_t pmw3901_bank_after(const pmw3901_reg_t *tbl, unsigned n,
                           uint8_t bank_in);

/* The prelude's two derived values. PX4 computes c1 in uint8_t and so wraps
 * for a raw value >= 245 before its clamp can see it; the vendor intent is
 * min(raw + k, 0x3F), and that is what this does. */
uint8_t pmw3901_prelude_c1(uint8_t raw);
uint8_t pmw3901_prelude_c2(uint8_t raw);

#endif /* PMW3901_CALC_H */
