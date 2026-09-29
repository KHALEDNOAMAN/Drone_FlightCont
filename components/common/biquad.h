#ifndef BIQUAD_H
#define BIQUAD_H

/* Second-order IIR sections, RBJ Audio EQ Cookbook coefficients.
 *
 * Direct Form II Transposed. DF2T is the right choice for float: it keeps
 * the two state variables in the same numeric range as the signal, so a
 * gyro reading of a few hundred dps never produces a huge intermediate
 * the way Direct Form I's delay line can.
 *
 * One section is 5 multiplies and 4 adds per sample - about 20 ns on this
 * part at 240 MHz. Cost is irrelevant here; phase lag is what costs you.
 * A 2nd-order Butterworth has group delay ~0.225/fc at DC, so 80 Hz costs
 * ~2.8 ms and 40 Hz costs ~5.6 ms. That lag is a real part of your
 * control latency - do not lower the cutoff casually.
 *
 * NYQUIST: fc must be well under fs/2 to mean anything. The gyro filters
 * run in the fusion task, which is DRDY-paced at the LSM6DSOX's ~416 Hz
 * ODR (P3), so fs/2 is ~208 Hz and nothing aliases on the way in. (Before
 * P3 fusion sampled at 250 Hz and 125-208 Hz folded into the band - no
 * filter can undo that, which is why the fix was the sampling, not this.) */

typedef struct {
    float b0, b1, b2;   /* feed-forward, already normalised by a0 */
    float a1, a2;       /* feedback,     already normalised by a0 */
    float z1, z2;       /* DF2T state */
} biquad_t;

/* Butterworth-ish default: Q = 0.7071 gives maximally flat passband. */
#define BIQUAD_Q_BUTTERWORTH  0.70710678f

/* Low-pass. fc and fs in Hz. Silently clamps fc to 0.45*fs - above that
 * the bilinear transform's frequency warping makes the response
 * meaningless. */
void biquad_init_lpf(biquad_t *f, float fc_hz, float fs_hz, float q);

/* Band-stop, for a known motor/frame resonance once an FFT has located
 * it. Q sets notch width: 10-20 suits a peak that stays put; too low eats
 * useful bandwidth, too high lets normal RPM drift walk the peak out of
 * the notch. This airframe runs a deliberately WIDE Q5 (fusion.c), because
 * its roll mode wanders 77-83 Hz run to run. */
void biquad_init_notch(biquad_t *f, float fc_hz, float fs_hz, float q);

/* Clear history to a steady-state value. Call with the current sample
 * when starting or restarting, otherwise the filter spends its first
 * several samples ramping up from zero - which on a gyro reads as a
 * transient the controller will happily react to. */
void biquad_reset(biquad_t *f, float x0);

/* One sample in, one sample out. */
static inline float biquad_apply(biquad_t *f, float x)
{
    float y = f->b0 * x + f->z1;
    f->z1   = f->b1 * x - f->a1 * y + f->z2;
    f->z2   = f->b2 * x - f->a2 * y;
    return y;
}

#endif /* BIQUAD_H */
