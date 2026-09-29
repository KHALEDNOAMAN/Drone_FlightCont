#!/usr/bin/env python3
"""Host test: can a saved params blob be mapped onto the WRONG fields?

    python tools/params_migrate_test.py

WHY THIS EXISTS
---------------
On six fields were inserted into params_t before alt_acc_sigma
without a PARAMS_VERSION bump. params_load_blob maps saved floats onto
fields BY POSITION, so every value from slot 28 on landed on a different
field than the one it was saved from:

    saved alt_acc_sigma 2.0   -> loaded into hold_auto  (clamped to 1.0)
    saved tof_sig_lim   0.05  -> loaded into vel_kp
    saved tof_fin_pclk  14    -> loaded into vel_i_max
    saved tof_budget_ms 200   -> loaded into vel_tilt_max

Nothing detected it. The _Static_assert catches a SIZE change; the boot
order-check catches a REORDER within one build; neither can see that
YESTERDAY's build had different names in those slots. The aircraft then
flew two sessions with both holds automatically enabled, which is a
setting nobody selected, on an estimator that had never flown.

The fix under test is the names digest in blob_t: crc32 over the names of
k_table[0..count-1]. Append leaves those names untouched and a prefix
load proceeds as designed; insert, rename or reorder changes the digest
and the blob is refused instead of silently reinterpreted.

HOW
---
Compiles the REAL components/params/params.c freestanding against stub
ESP headers, the same trick the other host tests use, and drives it
through a ctypes shim. No paraphrase of the logic in Python - the thing
under test is the code that flies.

Set PARAMS_SRC to a doctored copy to mutation-test it.
"""

import ctypes
import os
import struct
import subprocess
import sys
import tempfile
import zlib

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
PARAMS_DIR = os.path.join(ROOT, "components", "params")
PARAMS_SRC = os.environ.get("PARAMS_SRC",
                            os.path.join(PARAMS_DIR, "params.c"))

# ---------------------------------------------------------------- stubs
# params.c is the one component under test that talks to the ESP-IDF, so
# unlike vert_filter it cannot just be included. These stand in for the
# four headers it pulls, and for the libc this machine's clang has no
# headers for. Nothing here reimplements params.c's own logic.
STUB_HEADERS = {
    "esp_log.h": r"""
#ifndef STUB_ESP_LOG_H
#define STUB_ESP_LOG_H
/* Log lines are not what this test asserts on - it asserts on the values
 * that end up in g_params and on whether the blob was refused. Counted
 * anyway so a future test can require that a refusal was noisy. */
extern int g_log_w, g_log_e;
#define ESP_LOGI(tag, ...) do { } while (0)
#define ESP_LOGW(tag, ...) do { g_log_w++; } while (0)
#define ESP_LOGE(tag, ...) do { g_log_e++; } while (0)
#endif
""",
    "esp_rom_crc.h": r"""
#ifndef STUB_ESP_ROM_CRC_H
#define STUB_ESP_ROM_CRC_H
#include <stdint.h>
#include <stddef.h>
uint32_t esp_rom_crc32_le(uint32_t crc, const uint8_t *buf, uint32_t len);
#endif
""",
    "nvs.h": r"""
#ifndef STUB_NVS_H
#define STUB_NVS_H
#include <stdint.h>
#include <stddef.h>
typedef int esp_err_t;
typedef int nvs_handle_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define NVS_READONLY 0
#define NVS_READWRITE 1
esp_err_t nvs_open(const char *ns, int mode, nvs_handle_t *out);
esp_err_t nvs_get_blob(nvs_handle_t h, const char *k, void *o, size_t *l);
esp_err_t nvs_set_blob(nvs_handle_t h, const char *k, const void *v, size_t l);
esp_err_t nvs_commit(nvs_handle_t h);
void      nvs_close(nvs_handle_t h);
#endif
""",
    "nvs_flash.h": '#include "nvs.h"\n',
    "string.h": r"""
#ifndef STUB_STRING_H
#define STUB_STRING_H
#include <stddef.h>
void *memcpy(void *d, const void *s, size_t n);
size_t strlen(const char *s);
int strcmp(const char *a, const char *b);
#endif
""",
    "math.h": r"""
#ifndef STUB_MATH_H
#define STUB_MATH_H
#define isfinite(x) __builtin_isfinite(x)
#endif
""",
}

SHIM = r"""
int _fltused = 0;
int g_log_w = 0, g_log_e = 0;

typedef __SIZE_TYPE__ sz_t;
void *memcpy(void *d, const void *s, sz_t n)
{
    unsigned char *dd = (unsigned char *)d;
    const unsigned char *ss = (const unsigned char *)s;
    while (n--) *dd++ = *ss++;
    return d;
}
sz_t strlen(const char *s) { const char *p = s; while (*p) p++; return (sz_t)(p - s); }
int strcmp(const char *a, const char *b)
{
    while (*a && *a == *b) { a++; b++; }
    return (int)(unsigned char)*a - (int)(unsigned char)*b;
}

/* Bit-exact CRC-32/ISO-HDLC, the same function the ROM provides. The
 * test verifies its own Python mirror against this rather than assuming
 * a convention. */
unsigned int esp_rom_crc32_le(unsigned int crc, const unsigned char *buf,
                              unsigned int len)
{
    crc = ~crc;
    while (len--) {
        crc ^= *buf++;
        for (int k = 0; k < 8; k++)
            crc = (crc >> 1) ^ (0xEDB88320u & (unsigned int)(-(int)(crc & 1)));
    }
    return ~crc;
}

/* NVS is never exercised: the test hands params_load_blob its bytes
 * directly, because that is the function whose mapping is on trial. */
int  nvs_open(const char *ns, int mode, int *out) { (void)ns; (void)mode; *out = 1; return -1; }
int  nvs_get_blob(int h, const char *k, void *o, sz_t *l) { (void)h;(void)k;(void)o;(void)l; return -1; }
int  nvs_set_blob(int h, const char *k, const void *v, sz_t l) { (void)h;(void)k;(void)v;(void)l; return 0; }
int  nvs_commit(int h) { (void)h; return 0; }
void nvs_close(int h) { (void)h; }

#include "params_under_test.c"

#define EXPORT __declspec(dllexport)

/* Exactly what params_init does with the bytes it read from NVS, minus
 * the NVS. Returns 1 if the blob was adopted, 0 if it was refused. */
EXPORT int pt_load(const unsigned char *raw, int len)
{
    g_params    = k_defaults;
    s_defaulted = false;
    g_log_w = g_log_e = 0;
    if (!params_load_blob(raw, (sz_t)len)) {
        g_params    = k_defaults;
        s_defaulted = true;
        return 0;
    }
    return 1;
}
EXPORT int   pt_count(void)          { return N_PARAMS; }
EXPORT const char *pt_name(int i)    { return (i >= 0 && i < N_PARAMS) ? k_table[i].name : 0; }
EXPORT int   pt_no_migrate(int i)    { return (i >= 0 && i < N_PARAMS) ? (k_table[i].no_migrate ? 1 : 0) : 0; }
EXPORT float pt_min(int i)           { return k_table[i].min; }
EXPORT float pt_max(int i)           { return k_table[i].max; }
EXPORT float pt_default(int i)       { return ((const float *)&k_defaults)[i]; }
EXPORT float pt_value(int i)         { return ((const float *)&g_params)[i]; }
EXPORT int   pt_defaulted(void)      { return s_defaulted ? 1 : 0; }
EXPORT int   pt_errors(void)         { return g_log_e; }
EXPORT unsigned int pt_names_crc(int n) { return names_crc(n); }
EXPORT unsigned int pt_crc32(const unsigned char *b, int n)
{
    return esp_rom_crc32_le(0, b, (unsigned int)n);
}
EXPORT int pt_version(void)          { return PARAMS_VERSION; }
"""


def build(tmp):
    inc = os.path.join(tmp, "inc")
    os.makedirs(inc, exist_ok=True)
    for name, text in STUB_HEADERS.items():
        with open(os.path.join(inc, name), "w") as fh:
            fh.write(text)
    # Copied rather than included by path so PARAMS_SRC can point at a
    # mutant without the shim caring where it lives.
    with open(PARAMS_SRC, "r", encoding="utf-8") as fh:
        src = fh.read()
    with open(os.path.join(inc, "params_under_test.c"), "w",
              encoding="utf-8") as fh:
        fh.write(src)
    shim = os.path.join(tmp, "shim.c")
    with open(shim, "w") as fh:
        fh.write(SHIM)
    dll = os.path.join(tmp, "pt.dll")
    cmd = ["clang", "-shared", "-nostdlib", "-Wl,-noentry", "-O1",
           "-fno-math-errno", "-I", inc, "-I", PARAMS_DIR, shim, "-o", dll]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stdout + r.stderr)
        sys.exit("clang failed - see above. This test needs clang on PATH.")
    return dll


class T:
    def __init__(self):
        self.n = 0
        self.bad = 0

    def ok(self, cond, what):
        self.n += 1
        if cond:
            print("  [PASS] %s" % what)
        else:
            self.bad += 1
            print("  [FAIL] %s" % what)


def main():
    with tempfile.TemporaryDirectory(ignore_cleanup_errors=True) as tmp:
        lib = ctypes.CDLL(build(tmp))
        lib.pt_name.restype = ctypes.c_char_p
        for f in ("pt_min", "pt_max", "pt_default", "pt_value"):
            getattr(lib, f).restype = ctypes.c_float
            getattr(lib, f).argtypes = [ctypes.c_int]
        lib.pt_names_crc.restype = ctypes.c_uint32
        lib.pt_crc32.restype = ctypes.c_uint32
        lib.pt_crc32.argtypes = [ctypes.c_char_p, ctypes.c_int]

        N = lib.pt_count()
        VER = lib.pt_version()
        names = [lib.pt_name(i).decode() for i in range(N)]
        defaults = [lib.pt_default(i) for i in range(N)]
        print("params v%d, %d fields, last = %s\n" % (VER, N, names[-1]))

        # Mirror the C CRC in Python, then PROVE the mirror rather than
        # assuming a convention. If this ever stops matching, every blob
        # this test builds would be rejected for the wrong reason and the
        # suite would look green while testing nothing.
        probe = b"the quick brown fox"
        assert lib.pt_crc32(probe, len(probe)) == zlib.crc32(probe), (
            "Python CRC mirror disagrees with the C one - fix the mirror "
            "before trusting any result below")

        def digest(count):
            c = 0
            for nm in names[:count]:
                c = zlib.crc32(nm.encode() + b"\0", c)
            return c

        def blob(values, count=None, names_crc=None, version=None,
                 legacy=False):
            """Build an on-flash blob. legacy=True omits the digest word,
            i.e. the v6 layout that shipped before this fix."""
            count = len(values) if count is None else count
            head = struct.pack("<II", VER if version is None else version,
                               count)
            if not legacy:
                head += struct.pack("<I",
                                    digest(count) if names_crc is None
                                    else names_crc)
            body = head + struct.pack("<%df" % len(values), *values)
            return body + struct.pack("<I", zlib.crc32(body))

        t = T()

        # ---- 1. round trip: a blob this build wrote, loaded back ------
        vals = list(defaults)
        vals[names.index("m1_scale")] = 0.88
        vals[names.index("level_roll_deg")] = -0.524483
        adopted = lib.pt_load(blob(vals), len(blob(vals)))
        t.ok(adopted == 1, "a self-written blob is adopted")
        t.ok(abs(lib.pt_value(names.index("m1_scale")) - 0.88) < 1e-6,
             "m1_scale survives the round trip")
        t.ok(lib.pt_defaulted() == 0, "round trip does not flag defaulted")

        # ---- 2. APPEND still works (the property the digest must keep) -
        short = vals[:N - 1]
        b = blob(short)
        adopted = lib.pt_load(b, len(b))
        t.ok(adopted == 1, "a shorter (appended-since) blob still loads")
        t.ok(abs(lib.pt_value(names.index("m1_scale")) - 0.88) < 1e-6,
             "append: earlier fields keep their saved values")
        t.ok(abs(lib.pt_value(N - 1) - defaults[N - 1]) < 1e-9,
             "append: the new field sits at its default")

        # ---- 3. no_migrate: an enable is never inherited --------------
        enables = [i for i in range(N) if lib.pt_no_migrate(i)]
        t.ok(len(enables) >= 2,
             "the mode enables are marked no_migrate (%s)"
             % ", ".join(names[i] for i in enables))
        e = enables[0]
        mig = vals[:N - 1]
        mig[e] = 1.0                      # "the last build was left ON"
        b = blob(mig)
        lib.pt_load(b, len(b))
        t.ok(lib.pt_value(e) == 0.0,
             "%s is reset to 0 on a migration, not inherited" % names[e])

        # ...but on a same-shape load it must be honoured, or the setting
        # could never be used at all.
        same = list(vals)
        same[e] = 1.0
        b = blob(same)
        lib.pt_load(b, len(b))
        t.ok(lib.pt_value(e) == 1.0,
             "%s IS honoured when the layout matches exactly" % names[e])

        # ---- 4. THE REGRESSION: the insertion --------------
        # A blob saved by the v6 build, whose slot 28 onward were
        # alt_acc_sigma / tof_*. Same float count as a modern prefix, so
        # only the digest can tell them apart.
        old_names = (names[:28] + ["alt_acc_sigma", "tof_sig_lim",
                                   "tof_pre_pclk", "tof_fin_pclk",
                                   "tof_budget_ms"])
        old_vals = list(defaults[:28]) + [2.0, 0.05, 18.0, 14.0, 200.0]
        c = 0
        for nm in old_names:
            c = zlib.crc32(nm.encode() + b"\0", c)
        b = blob(old_vals, count=len(old_vals), names_crc=c)
        adopted = lib.pt_load(b, len(b))
        t.ok(adopted == 0, "the mid-struct insertion is REFUSED")
        t.ok(lib.pt_errors() > 0, "and the refusal is logged at ERROR")
        t.ok(lib.pt_defaulted() == 1, "and it is flagged as defaulted")
        ah = names.index("alt_hold_auto")
        t.ok(lib.pt_value(ah) == 0.0,
             "alt_hold_auto is 0, NOT the 2.0-clamped-to-1.0 that flew")
        vm = names.index("vel_max")
        t.ok(abs(lib.pt_value(vm) - defaults[vm]) < 1e-9,
             "vel_max is its default %.1f, not tof_budget_ms's 200"
             % defaults[vm])

        # ---- 5. the pre-fix on-flash layout is refused ----------------
        b = blob(vals, legacy=True)
        adopted = lib.pt_load(b, len(b))
        t.ok(adopted == 0, "a v6 blob with no digest word is refused")

        # ---- 6. a plain rename is caught ------------------------------
        b = blob(vals, names_crc=digest(N) ^ 0x1)
        adopted = lib.pt_load(b, len(b))
        t.ok(adopted == 0, "a one-bit digest difference is refused")

        # ---- 7. clamping stays bounded -------------------------------
        one_bad = list(vals)
        one_bad[names.index("rate_kp")] = 999.0
        b = blob(one_bad)
        t.ok(lib.pt_load(b, len(b)) == 1,
             "ONE out-of-range field is still clamped and adopted")

        many_bad = list(vals)
        for nm in ("rate_kp", "yaw_kp", "angle_kp"):
            many_bad[names.index(nm)] = 999.0
        b = blob(many_bad)
        t.ok(lib.pt_load(b, len(b)) == 0,
             "THREE out-of-range fields is a layout mismatch, refused")

        enable_bad = list(vals)
        enable_bad[e] = 2.0               # exactly the historical value
        b = blob(enable_bad)
        t.ok(lib.pt_load(b, len(b)) == 0,
             "an out-of-range MODE ENABLE alone is refused, never clamped")

        # ---- 8. corruption is still corruption ------------------------
        b = bytearray(blob(vals))
        b[-1] ^= 0xFF
        t.ok(lib.pt_load(bytes(b), len(b)) == 0, "a bad CRC is refused")

        print("\n%d checks, %d failed" % (t.n, t.bad))
        if t.bad:
            sys.exit(1)
        print("OK")


if __name__ == "__main__":
    main()
