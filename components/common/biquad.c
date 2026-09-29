#include "biquad.h"
#include <math.h>

#define BIQUAD_PI  3.14159265358979f

/* RBJ Audio EQ Cookbook, the standard reference for these forms.
 *
 *   w0    = 2*pi*fc/fs
 *   alpha = sin(w0) / (2*Q)
 *
 * Low-pass:                         Notch:
 *   b0 = (1 - cos w0) / 2             b0 =  1
 *   b1 =  1 - cos w0                  b1 = -2 cos w0
 *   b2 = (1 - cos w0) / 2             b2 =  1
 *   a0 =  1 + alpha                   a0 =  1 + alpha
 *   a1 = -2 cos w0                    a1 = -2 cos w0
 *   a2 =  1 - alpha                   a2 =  1 - alpha
 *
 * All six are divided through by a0 at init so the hot path never does. */

static void normalise(biquad_t *f,
                      float b0, float b1, float b2,
                      float a0, float a1, float a2)
{
    float inv_a0 = 1.0f / a0;
    f->b0 = b0 * inv_a0;
    f->b1 = b1 * inv_a0;
    f->b2 = b2 * inv_a0;
    f->a1 = a1 * inv_a0;
    f->a2 = a2 * inv_a0;
    f->z1 = 0.0f;
    f->z2 = 0.0f;
}

/* Pass-through: b0 = 1, everything else 0. Used when the caller asks for
 * a nonsensical cutoff, so a bad config degrades to "no filtering"
 * instead of to garbage or a divide by zero. */
static void set_passthrough(biquad_t *f)
{
    f->b0 = 1.0f;
    f->b1 = f->b2 = f->a1 = f->a2 = 0.0f;
    f->z1 = f->z2 = 0.0f;
}

void biquad_init_lpf(biquad_t *f, float fc_hz, float fs_hz, float q)
{
    if (!f) return;
    if (fs_hz <= 0.0f || fc_hz <= 0.0f || q <= 0.0f) {
        set_passthrough(f);
        return;
    }

    /* Clamp below Nyquist. Past ~0.45*fs the bilinear transform warps the
     * response badly enough that the "cutoff" stops meaning anything. */
    float fc_max = 0.45f * fs_hz;
    if (fc_hz > fc_max) fc_hz = fc_max;

    float w0    = 2.0f * BIQUAD_PI * fc_hz / fs_hz;
    float cw0   = cosf(w0);
    float alpha = sinf(w0) / (2.0f * q);

    float b0 = (1.0f - cw0) * 0.5f;
    float b1 =  1.0f - cw0;
    float b2 = (1.0f - cw0) * 0.5f;
    float a0 =  1.0f + alpha;
    float a1 = -2.0f * cw0;
    float a2 =  1.0f - alpha;

    normalise(f, b0, b1, b2, a0, a1, a2);
}

void biquad_init_notch(biquad_t *f, float fc_hz, float fs_hz, float q)
{
    if (!f) return;
    if (fs_hz <= 0.0f || fc_hz <= 0.0f || q <= 0.0f) {
        set_passthrough(f);
        return;
    }

    float fc_max = 0.45f * fs_hz;
    if (fc_hz > fc_max) fc_hz = fc_max;

    float w0    = 2.0f * BIQUAD_PI * fc_hz / fs_hz;
    float cw0   = cosf(w0);
    float alpha = sinf(w0) / (2.0f * q);

    float b0 =  1.0f;
    float b1 = -2.0f * cw0;
    float b2 =  1.0f;
    float a0 =  1.0f + alpha;
    float a1 = -2.0f * cw0;
    float a2 =  1.0f - alpha;

    normalise(f, b0, b1, b2, a0, a1, a2);
}

void biquad_reset(biquad_t *f, float x0)
{
    if (!f) return;
    /* DF2T steady state for a constant input x0: with y == x0 held,
     *   z1 = b1*x0 - a1*x0 + z2
     *   z2 = b2*x0 - a2*x0
     * so the filter outputs x0 immediately instead of ramping from 0. */
    f->z2 = (f->b2 - f->a2) * x0;
    f->z1 = (f->b1 - f->a1) * x0 + f->z2;
}
