#!/usr/bin/env python3
"""Host test for components/common/geo.c - the local tangent plane.

Same trick as tools/host_c.py: geo.c is freestanding, so clang compiles
the exact file that flies and ctypes drives it.

The reference numbers come from three independent directions, never from
geo.c itself:
  * Python's math module re-deriving the WGS84 radii of curvature
  * published lengths of a degree at 40 deg N (111034 m / 85394 m)
  * physical round trips - walk 100 m north, get 100 m north back

Run:  python tools/geo_test.py
"""

import ctypes
import math
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SRC = os.environ.get("GEO_SRC") or os.path.join(ROOT, "components", "common")

SHIM = r"""
int _fltused = 0;
#include "geo.c"
#define EXPORT __declspec(dllexport)
EXPORT void g_clear(geo_home_t *h){ geo_home_clear(h); }
EXPORT void g_home(geo_home_t *h, int a, int b, int c){ geo_set_home(h,a,b,c); }
EXPORT int  g_ned(const geo_home_t *h, int a, int b, int c,
                  float *n, float *e, float *d){ return geo_to_ned(h,a,b,c,n,e,d); }
EXPORT int  g_size(void){ return (int)sizeof(geo_home_t); }
"""

A = 6378137.0
E2 = 0.00669437999014


class CHome(ctypes.Structure):
    _fields_ = [
        ("lat0_1e7", ctypes.c_int32), ("lon0_1e7", ctypes.c_int32),
        ("alt0_mm", ctypes.c_int32),
        ("m_per_1e7_lat", ctypes.c_double), ("m_per_1e7_lon", ctypes.c_double),
        ("valid", ctypes.c_uint8),
    ]


class HostGeo:
    def __init__(self):
        self._tmp = tempfile.TemporaryDirectory()
        shim = os.path.join(self._tmp.name, "shim.c")
        with open(shim, "w") as fh:
            fh.write(SHIM)
        dll = os.path.join(self._tmp.name, "geo.dll")
        cmd = ["clang", "-shared", "-nostdlib", "-Wl,-noentry", "-O1",
               "-fno-math-errno", "-I", SRC, shim, "-o", dll]
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode != 0:
            print(r.stdout + r.stderr)
            raise SystemExit("clang failed - this host test needs clang on PATH")
        self.lib = ctypes.CDLL(dll)
        L, ph = self.lib, ctypes.POINTER(CHome)
        pf = ctypes.POINTER(ctypes.c_float)
        L.g_clear.argtypes = [ph]
        L.g_home.argtypes = [ph, ctypes.c_int32, ctypes.c_int32, ctypes.c_int32]
        L.g_ned.argtypes = [ph, ctypes.c_int32, ctypes.c_int32, ctypes.c_int32,
                            pf, pf, pf]
        L.g_ned.restype = ctypes.c_int
        L.g_size.restype = ctypes.c_int
        if L.g_size() != ctypes.sizeof(CHome):
            raise SystemExit(f"geo_home_t layout mismatch: C {L.g_size()} B, "
                             f"ctypes {ctypes.sizeof(CHome)} B")

    def __enter__(self):
        return self

    def __exit__(self, *a):
        h = self.lib._handle
        del self.lib
        ctypes.windll.kernel32.FreeLibrary(ctypes.c_void_p(h))
        self._tmp.cleanup()

    def home(self, lat_deg, lon_deg, alt_m=0.0):
        h = CHome()
        self.lib.g_home(ctypes.byref(h), int(round(lat_deg * 1e7)),
                        int(round(lon_deg * 1e7)), int(round(alt_m * 1000)))
        return h

    def ned(self, h, lat_deg, lon_deg, alt_m=0.0):
        n, e, d = (ctypes.c_float() for _ in range(3))
        ok = self.lib.g_ned(ctypes.byref(h), int(round(lat_deg * 1e7)),
                            int(round(lon_deg * 1e7)), int(round(alt_m * 1000)),
                            ctypes.byref(n), ctypes.byref(e), ctypes.byref(d))
        return ok, n.value, e.value, d.value


def ref_scales(lat_deg):
    """WGS84 metres per degree, re-derived here with math, not with geo.c."""
    phi = math.radians(lat_deg)
    w2 = 1.0 - E2 * math.sin(phi) ** 2
    m_rad = A * (1.0 - E2) / (w2 * math.sqrt(w2))
    n_rad = A / math.sqrt(w2)
    return (m_rad * math.pi / 180.0,
            n_rad * math.cos(phi) * math.pi / 180.0)


FAIL = 0


def near(name, got, want, tol):
    global FAIL
    ok = abs(got - want) <= tol
    if not ok:
        FAIL += 1
    print(f"  [{'PASS' if ok else 'FAIL'}] {name:<48} {got:+.6f} "
          f"(want {want:+.6f} +-{tol:g})")


def check(name, got, want):
    global FAIL
    ok = got == want
    if not ok:
        FAIL += 1
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}")
    if not ok:
        print(f"         got {got!r}  want {want!r}")


BURSA_LAT, BURSA_LON = 40.1885, 29.0610


def main():
    with HostGeo() as G:
        print("\nscale factors against WGS84 re-derived in Python")
        for lat in (0.0, 40.1885, 60.0, -33.8675, 85.0):
            h = G.home(lat, 0.0)
            wlat, wlon = ref_scales(lat)
            near(f"lat {lat:>9.4f} - m per deg latitude",
                 h.m_per_1e7_lat * 1e7, wlat, 1e-3)
            near(f"lat {lat:>9.4f} - m per deg longitude",
                 h.m_per_1e7_lon * 1e7, wlon, 1e-3)

        print("\nagainst published lengths of a degree at 40 deg N")
        h = G.home(40.0, 0.0)
        near("metre per degree of latitude  ~111034",
             h.m_per_1e7_lat * 1e7, 111034.0, 2.0)
        near("metre per degree of longitude ~85394",
             h.m_per_1e7_lon * 1e7, 85394.0, 2.0)

        print("\nphysical round trips from Bursa")
        h = G.home(BURSA_LAT, BURSA_LON, 155.0)
        mlat, mlon = ref_scales(BURSA_LAT)

        ok, n, e, d = G.ned(h, BURSA_LAT, BURSA_LON, 155.0)
        check("home maps to the origin", (ok, round(n, 6), round(e, 6),
                                          round(d, 6)), (1, 0.0, 0.0, 0.0))

        # 100 m north
        ok, n, e, d = G.ned(h, BURSA_LAT + 100.0 / mlat, BURSA_LON, 155.0)
        near("100 m north  -> N", n, 100.0, 0.01)
        near("             -> E", e, 0.0, 0.01)

        # 100 m east
        ok, n, e, d = G.ned(h, BURSA_LAT, BURSA_LON + 100.0 / mlon, 155.0)
        near("100 m east   -> E", e, 100.0, 0.01)
        near("             -> N", n, 0.0, 0.01)

        # south and west are negative
        ok, n, e, d = G.ned(h, BURSA_LAT - 50.0 / mlat,
                            BURSA_LON - 50.0 / mlon, 155.0)
        near("50 m south   -> N", n, -50.0, 0.01)
        near("50 m west    -> E", e, -50.0, 0.01)

        print("\nvertical sign - D is POSITIVE DOWN")
        ok, n, e, d = G.ned(h, BURSA_LAT, BURSA_LON, 165.0)
        near("10 m higher than home -> D", d, -10.0, 0.001)
        ok, n, e, d = G.ned(h, BURSA_LAT, BURSA_LON, 145.0)
        near("10 m lower  than home -> D", d, +10.0, 0.001)

        print("\nthe precision that motivated int32 1e-7")
        # One unit of 1e-7 deg of longitude at this latitude, ~0.85 cm.
        # A float32 lat/lon could not represent this step at all.
        ok, n, e, d = G.ned(h, BURSA_LAT, BURSA_LON + 1e-7, 155.0)
        near("a single 1e-7 deg step east is visible", e, mlon * 1e-7, 1e-4)

        print("\nlongitude difference must not overflow int32")
        # Home at +179.9, fix at -179.9: the naive int32 subtraction is
        # 3.598e9, which wraps negative and reports ~40000 km the wrong way.
        h2 = G.home(0.0, 179.9)
        ok, n, e, d = G.ned(h2, 0.0, -179.9)
        _, mlon0 = ref_scales(0.0)
        near("across the antimeridian, 0.2 deg east", e, 0.2 * mlon0, 1.0)

        h3 = G.home(0.0, -179.9)
        ok, n, e, d = G.ned(h3, 0.0, 179.9)
        near("and the same the other way", e, -0.2 * mlon0, 1.0)

        print("\nhome not set")
        h4 = CHome()
        G.lib.g_clear(ctypes.byref(h4))
        n, e, d = (ctypes.c_float(-42.0) for _ in range(3))
        ok = G.lib.g_ned(ctypes.byref(h4), 401885000, 290610000, 0,
                         ctypes.byref(n), ctypes.byref(e), ctypes.byref(d))
        check("refuses to convert", ok, 0)
        check("and leaves the outputs untouched",
              (n.value, e.value, d.value), (-42.0, -42.0, -42.0))

    print(f"\n{FAIL} FAILURE(S)\n" if FAIL else "\nALL PASS\n")
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
