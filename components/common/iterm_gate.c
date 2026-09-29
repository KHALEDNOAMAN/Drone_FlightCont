/* Rate-loop I-term ground hold. The rule, the evidence behind each part of
 * it, and what it does when the ToF has nothing to say: iterm_gate.h. */

#include "iterm_gate.h"

void iterm_gate_init(iterm_gate_t *g)
{
    g->was_armed = false;
    g->ref_ok = false;
    g->ref_mm = 0;
    g->lifted = false;
    g->thr_seen = false;
    g->have_seq = false;
    g->seq_moved = false;
    g->seq = 0;
    g->seq_us = 0;
    g->fresh = false;
    g->hold = false;
}

bool iterm_gate_step(iterm_gate_t *g, bool armed, uint16_t throttle_us,
                     uint16_t tof_mm, uint32_t tof_seq, int64_t now_us)
{
    /* FRESH means the chip completed a measurement recently, not that the
     * cached number looks plausible. The cache keeps its last value between
     * samples, so a value alone cannot tell a sensor that stopped from an
     * aircraft sitting still - the counter can. The first sighting is only
     * a baseline: it says nothing about WHEN that sample arrived. */
    if (!g->have_seq) {
        g->have_seq = true;
        g->seq = tof_seq;
    } else if (tof_seq != g->seq) {
        g->seq = tof_seq;
        g->seq_us = now_us;
        g->seq_moved = true;
    }
    g->fresh = tof_mm != ITERM_GATE_TOF_NONE &&
               tof_mm != ITERM_GATE_TOF_NO_TARGET &&
               g->seq_moved && (now_us - g->seq_us) <= ITERM_GATE_FRESH_US;

    if (!armed) {
        g->was_armed = false;
        g->hold = false;
        return false;
    }

    /* Arm edge: the aircraft is on the ground (pre-arm needs the throttle
     * low), so a fresh reading now IS the height of its own legs. */
    if (!g->was_armed) {
        g->was_armed = true;
        g->ref_ok = g->fresh;
        g->ref_mm = tof_mm;
        g->lifted = false;
        g->thr_seen = false;
    }

    /* No reference: Betaflight's airmode latch, the best a throttle stick
     * alone can do. */
    if (!g->ref_ok) {
        if (throttle_us >= ITERM_GATE_THR_US) g->thr_seen = true;
        g->hold = !g->thr_seen;
        return g->hold;
    }

    const int32_t d = (int32_t)tof_mm - (int32_t)g->ref_mm;
    if (g->fresh && d > ITERM_GATE_NEAR_MM) g->lifted = true;
    const bool near = g->fresh && d >= -ITERM_GATE_NEAR_MM &&
                      d <= ITERM_GATE_NEAR_MM;

    g->hold = g->lifted ? (near && throttle_us < ITERM_GATE_THR_US) : near;
    return g->hold;
}

uint8_t iterm_gate_flags(const iterm_gate_t *g)
{
    return (uint8_t)((g->hold     ? ITERM_GATE_F_HELD    : 0u) |
                     (g->ref_ok   ? ITERM_GATE_F_TOF_REF : 0u) |
                     (g->lifted   ? ITERM_GATE_F_LIFTED  : 0u) |
                     (g->fresh    ? ITERM_GATE_F_FRESH   : 0u) |
                     (g->thr_seen ? ITERM_GATE_F_THR     : 0u));
}
