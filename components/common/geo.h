#ifndef GEO_H
#define GEO_H

/* Local tangent plane - geodetic lat/lon/alt to NED metres from a home
 * point. Stage A of the GPS-fusion work.
 *
 * A filter cannot work in degrees: a degree of longitude is a different
 * number of metres at every latitude, and the units of a covariance
 * matrix have to be metres. This converts once, at the edge, so nothing
 * downstream ever sees an angle.
 *
 * FLAT-EARTH ON PURPOSE. The two scale factors are computed once when
 * home is latched, from the WGS84 meridional and normal radii of
 * curvature at that latitude, and then applied as plain multiplies. Over
 * the few kilometres this aircraft will ever be from home the error is
 * well under a centimetre, which is smaller than the GPS's own hAcc by
 * two orders of magnitude. Full geodesy would be measurably slower and
 * not measurably better.
 *
 * FREESTANDING, like vert_filter.c next door: stdint only, no libc, no
 * math.h, no ESP-IDF. That is what lets tools/geo_test.py compile this
 * exact file on the host. sin and cos are series-evaluated below for the
 * same reason. Host-tested by tools/geo_test.py.
 */

#include <stdint.h>

typedef struct {
    int32_t lat0_1e7;      /* home latitude,  1e-7 deg */
    int32_t lon0_1e7;      /* home longitude, 1e-7 deg */
    int32_t alt0_mm;       /* home height MSL, mm */
    /* Metres per 1e-7 degree at the home latitude. Precomputed so the
     * per-sample path is two multiplies and no trigonometry. */
    double m_per_1e7_lat;
    double m_per_1e7_lon;
    uint8_t valid;         /* 0 until geo_set_home() has run */
} geo_home_t;

void geo_home_clear(geo_home_t *h);

/* Latch the home point and compute the scale factors for its latitude. */
void geo_set_home(geo_home_t *h, int32_t lat_1e7, int32_t lon_1e7,
                  int32_t alt_mm);

/* Convert a fix to NED metres from home. Returns 1 on success, 0 if home
 * has not been set (in which case *n, *e, *d are left untouched).
 * D is POSITIVE DOWN, so climbing above home gives a negative d. */
int geo_to_ned(const geo_home_t *h, int32_t lat_1e7, int32_t lon_1e7,
               int32_t alt_mm, float *n, float *e, float *d);

#endif /* GEO_H */
