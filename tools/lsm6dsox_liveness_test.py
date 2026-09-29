#!/usr/bin/env python3
"""Host test for the LSM6DSOX liveness checks (review, item 2):
the UNIFORM and FROZEN burst refusals in lsm6dsox_read(), and the frozen /
implausible refusal in lsm6dsox_calibrate_bias() plus the init failure it
causes at boot.

The real components/lsm6dsox/lsm6dsox.c is compiled - not a model of it.
It is not freestanding (SPI driver, FreeRTOS, esp_log), so this file writes
minimal stand-ins for those headers into a temp dir, and the shim fakes the
SPI bus: WHO_AM_I answers 0x6C, and every data read is served from one of
three generators - seeded noise around a base (a healthy chip), one byte
repeated (incident 55: every byte read back as 0xF0), or one fixed 12-byte pattern
(a data path that stopped updating).

The external reference is incident 55: gyro bias -67.480, 67.480,
-67.480 dps at std 0.000 is raw 0xF0F0 on all three axes through
IMU_REMAP, and the driver stored it as a calibration. Both the boot path
and GYROCAL must refuse it now, and a real noisy chip must still pass.

LSM_SRC points the compiler at a doctored copy for mutation runs.

Run:  python tools/lsm6dsox_liveness_test.py
"""

import ctypes
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SRC = os.environ.get("LSM_SRC") or os.path.join(ROOT, "components", "lsm6dsox")

# Stand-ins for the headers lsm6dsox.c pulls in. Only what the file uses.
STUBS = {
    "esp_err.h": r"""
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
typedef int esp_err_t;
#define ESP_OK                    0
#define ESP_FAIL                 -1
#define ESP_ERR_INVALID_STATE     0x103
#define ESP_ERR_INVALID_RESPONSE  0x108
""",
    "esp_log.h": r"""
#pragma once
#define ESP_LOGE(tag, ...) ((void)(tag))
#define ESP_LOGW(tag, ...) ((void)(tag))
#define ESP_LOGI(tag, ...) ((void)(tag))
""",
    "stdlib.h": r"""
#pragma once
#include <stddef.h>
void *calloc(size_t n, size_t sz);
void free(void *p);
""",
    "string.h": r"""
#pragma once
#include <stddef.h>
void *memcpy(void *d, const void *s, size_t n);
void *memset(void *d, int c, size_t n);
int memcmp(const void *a, const void *b, size_t n);
""",
    "math.h": r"""
#pragma once
#define sqrtf(x) __builtin_sqrtf(x)
#define fabsf(x) __builtin_fabsf(x)
""",
    os.path.join("freertos", "FreeRTOS.h"): r"""
#pragma once
#include <stdint.h>
#include <stdlib.h>
typedef uint32_t TickType_t;
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
""",
    os.path.join("freertos", "task.h"): r"""
#pragma once
static inline void vTaskDelay(TickType_t t) { (void)t; }
""",
    "esp_timer.h": r"""
#pragma once
#include <stdint.h>
int64_t esp_timer_get_time(void);
""",
    os.path.join("driver", "gpio.h"): r"""
#pragma once
typedef int gpio_num_t;
int gpio_get_level(gpio_num_t pin);
""",
    os.path.join("driver", "spi_master.h"): r"""
#pragma once
#include "esp_err.h"
typedef int spi_host_device_t;
#define SPI2_HOST 1
#define SPI_DMA_DISABLED 0
typedef struct {
    int mosi_io_num, miso_io_num, sclk_io_num, quadwp_io_num, quadhd_io_num;
    int max_transfer_sz;
} spi_bus_config_t;
typedef struct {
    int clock_speed_hz, mode, spics_io_num, queue_size;
} spi_device_interface_config_t;
typedef struct spi_dev_s *spi_device_handle_t;
typedef struct {
    size_t length;
    const void *tx_buffer;
    void *rx_buffer;
} spi_transaction_t;
esp_err_t spi_bus_initialize(spi_host_device_t h, const spi_bus_config_t *c,
                             int dma);
esp_err_t spi_bus_add_device(spi_host_device_t h,
                             const spi_device_interface_config_t *c,
                             spi_device_handle_t *out);
esp_err_t spi_device_polling_transmit(spi_device_handle_t h,
                                      spi_transaction_t *t);
esp_err_t spi_bus_remove_device(spi_device_handle_t h);
esp_err_t spi_bus_free(spi_host_device_t h);
""",
}

SHIM = r"""
int _fltused = 0;
#include <stddef.h>
#include <stdint.h>

/* -nostdlib: no libc, so the handful of functions the driver uses. */
void *memcpy(void *d, const void *s, size_t n)
{ unsigned char *a = d; const unsigned char *b = s; while (n--) *a++ = *b++; return d; }
void *memset(void *d, int c, size_t n)
{ unsigned char *a = d; while (n--) *a++ = (unsigned char)c; return d; }
int memcmp(const void *x, const void *y, size_t n)
{
    const unsigned char *a = x, *b = y;
    for (; n; n--, a++, b++) if (*a != *b) return *a < *b ? -1 : 1;
    return 0;
}
static _Alignas(16) unsigned char pool[1024];
static size_t pool_used;
void *calloc(size_t n, size_t sz)
{
    size_t b = (n * sz + 15) & ~(size_t)15;
    if (pool_used + b > sizeof(pool)) return 0;
    void *p = pool + pool_used;
    pool_used += b;
    memset(p, 0, b);
    return p;
}
void free(void *p) { (void)p; }

/* ---- the fake SPI bus ---- */
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"
static int g_mode;                    /* 0 noise, 1 one byte, 2 fixed pattern */
static unsigned char g_byte, g_pat[12];
static int g_base[6], g_amp[6];       /* chip-frame raw gx gy gz ax ay az */
static unsigned int g_seed;
static int g_removed, g_freed;

/* A register file with the reset defaults that matter here - CTRL3_C
 * IF_INC and CTRL9_XL DEN_X|Y|Z - and SW_RESET semantics, plus three
 * faults: g_drop_reg ignores writes, g_glitch_reg returns the complement
 * on its NEXT read only, and g_dead is a chip that does not drive MISO at
 * all: every byte of every read, WHO_AM_I included, is g_byte (the
 * freeze: 0xE9 everywhere). */
static unsigned char g_regs[128];
static int g_drop_reg = -1, g_glitch_reg = -1, g_dead, g_int1;
static long long g_now;

static void regs_default(void)
{
    for (int i = 0; i < 128; i++) g_regs[i] = 0;
    g_regs[0x12] = 0x04;
    g_regs[0x18] = 0xE0;
}
int64_t esp_timer_get_time(void) { return g_now; }
int gpio_get_level(gpio_num_t pin) { (void)pin; return g_int1; }

static unsigned int lcg(void)
{ g_seed = g_seed * 1103515245u + 12345u; return (g_seed >> 16) & 0x7FFFu; }

esp_err_t spi_bus_initialize(spi_host_device_t h, const spi_bus_config_t *c,
                             int dma)
{ (void)h; (void)c; (void)dma; return ESP_OK; }
esp_err_t spi_bus_add_device(spi_host_device_t h,
                             const spi_device_interface_config_t *c,
                             spi_device_handle_t *out)
{ (void)h; (void)c; *out = (spi_device_handle_t)(void *)&g_mode; return ESP_OK; }
esp_err_t spi_bus_remove_device(spi_device_handle_t h)
{ (void)h; g_removed++; return ESP_OK; }
esp_err_t spi_bus_free(spi_host_device_t h)
{ (void)h; g_freed++; return ESP_OK; }

esp_err_t spi_device_polling_transmit(spi_device_handle_t h,
                                      spi_transaction_t *t)
{
    (void)h;
    const unsigned char *tx = (const unsigned char *)t->tx_buffer;
    const int reg = tx[0] & 0x7F;
    if (!t->rx_buffer) {                               /* a register write */
        if (reg == g_drop_reg) return ESP_OK;
        if (reg == 0x12 && (tx[1] & 0x01)) { regs_default(); return ESP_OK; }
        g_regs[reg] = tx[1];
        return ESP_OK;
    }
    unsigned char *rx = (unsigned char *)t->rx_buffer;
    const int n = (int)(t->length / 8) - 1;            /* after the address */
    rx[0] = 0;
    if (g_dead) { for (int i = 0; i < n; i++) rx[1 + i] = g_byte; return ESP_OK; }
    if (reg == 0x0F) { if (n > 0) rx[1] = 0x6C; return ESP_OK; }   /* WHO_AM_I */
    if (reg < 0x22) {                                  /* config / status */
        for (int i = 0; i < n; i++) rx[1 + i] = g_regs[(reg + i) & 0x7F];
        if (reg == g_glitch_reg && n > 0) {
            rx[1] = (unsigned char)~rx[1];
            g_glitch_reg = -1;
        }
        return ESP_OK;
    }

    unsigned char b[12];
    for (int a = 0; a < 6; a++) {
        int v = g_base[a];
        if (g_amp[a])
            v += (int)(lcg() % (unsigned)(2 * g_amp[a] + 1)) - g_amp[a];
        b[2 * a]     = (unsigned char)(v & 0xFF);
        b[2 * a + 1] = (unsigned char)((v >> 8) & 0xFF);
    }
    if (g_mode == 1) for (int i = 0; i < 12; i++) b[i] = g_byte;
    if (g_mode == 2) for (int i = 0; i < 12; i++) b[i] = g_pat[i];

    const int off = reg - 0x22;                         /* OUTX_L_G */
    for (int i = 0; i < n; i++)
        rx[1 + i] = (off >= 0 && off + i < 12) ? b[off + i] : 0;
    return ESP_OK;
}

#include "lsm6dsox.c"
#define EXPORT __declspec(dllexport)

static lsm6dsox_handle_t *D;

EXPORT void t_reset(void)
{
    pool_used = 0; D = 0;
    s_uniform_reads = 0; s_frozen_reads = 0;
    g_removed = 0; g_freed = 0;
    regs_default();
    g_drop_reg = g_glitch_reg = -1; g_dead = 0; g_int1 = 0; g_now = 0;
    memset(&s_diag, 0, sizeof(s_diag));
    s_dump_logged_us = -1000000;
}
EXPORT void t_noise(const int *base, const int *amp, unsigned int seed)
{
    for (int a = 0; a < 6; a++) { g_base[a] = base[a]; g_amp[a] = amp[a]; }
    g_seed = seed; g_mode = 0; g_dead = 0;
}
EXPORT void t_byte(int byte) { g_byte = (unsigned char)byte; g_mode = 1; g_dead = 0; }
EXPORT void t_pattern(const unsigned char *p)
{ for (int i = 0; i < 12; i++) g_pat[i] = p[i]; g_mode = 2; g_dead = 0; }
EXPORT void t_dead(int byte) { g_byte = (unsigned char)byte; g_dead = 1; }
EXPORT void t_drop_write(int reg) { g_drop_reg = reg; }
EXPORT void t_glitch_read(int reg) { g_glitch_reg = reg; }
EXPORT void t_int1(int v) { g_int1 = v; }
EXPORT void t_now(long long us) { g_now = us; }
EXPORT int  t_reg(int r) { return g_regs[r & 0x7F]; }
EXPORT void t_set_int1_gpio(int pin) { D->int1_gpio = pin; }
EXPORT int  t_int1_gpio(void) { return D->int1_gpio; }
EXPORT void t_diag(int *o)
{
    lsm6dsox_diag_t g;
    lsm6dsox_diag_get(&g);
    o[0] = (int)g.episodes; o[1] = (int)g.longest; o[2] = (int)g.current;
    o[3] = g.cfg_bad; o[4] = g.have_first; o[5] = g.have_sticky;
    const lsm6dsox_dump_t *d[2] = { &g.first, &g.sticky };
    for (int k = 0; k < 2; k++) {
        int *p = o + 6 + 14 * k;
        p[0] = d[k]->kind; p[1] = d[k]->stuck; p[2] = d[k]->int1;
        p[3] = (int)d[k]->episode; p[4] = (int)(d[k]->t_us / 1000);
        p[5] = d[k]->spi_err;
        for (int i = 0; i < 8; i++) p[6 + i] = d[k]->reg[i];
    }
}

EXPORT int t_init(void) { D = lsm6dsox_init(); return D != 0; }
EXPORT int t_read(float *o)
{
    lsm6dsox_data_t d = { 9.0f, 9.0f, 9.0f, 9.0f, 9.0f, 9.0f };
    esp_err_t e = lsm6dsox_read(D, &d);
    o[0] = d.ax; o[1] = d.ay; o[2] = d.az; o[3] = d.gx; o[4] = d.gy; o[5] = d.gz;
    return e;
}
EXPORT int  t_cal(void) { return lsm6dsox_calibrate_bias(D); }
EXPORT void t_bias(float *b)
{ b[0] = D->gyro_bias.bx; b[1] = D->gyro_bias.by; b[2] = D->gyro_bias.bz; }
EXPORT void t_set_bias(float x, float y, float z)
{ D->gyro_bias.bx = x; D->gyro_bias.by = y; D->gyro_bias.bz = z; }
EXPORT void t_counts(unsigned int *o)
{
    uint32_t u = 0, f = 0;
    lsm6dsox_liveness_counts(&u, &f);
    o[0] = u; o[1] = f; o[2] = (unsigned int)g_removed; o[3] = (unsigned int)g_freed;
}
EXPORT int t_frozen_reads(void) { return LSM6DSOX_FROZEN_READS; }
EXPORT int t_sticky_reads(void) { return LSM6DSOX_STICKY_READS; }
EXPORT int t_code(int k)
{
    return k == 0 ? ESP_OK : k == 1 ? ESP_ERR_INVALID_RESPONSE
                                    : ESP_ERR_INVALID_STATE;
}
"""

# A healthy chip at rest, CHIP-frame raw counts: gyro ~ +0.51 / -0.51 /
# +0.18 dps of zero-rate offset, accel ~1 g on Z. amp is the +-noise in LSB,
# a few counts like the real part's at-rest noise.
HEALTHY = [29, -29, 10, 12, -40, 4096]
NOISE = [3, 3, 3, 4, 4, 4]
G = 0.0175                              # dps per LSB, 500 dps FS


class Host:
    def __init__(self):
        self._tmp = tempfile.TemporaryDirectory()
        d = self._tmp.name
        for rel, text in STUBS.items():
            p = os.path.join(d, rel)
            os.makedirs(os.path.dirname(p), exist_ok=True)
            with open(p, "w") as fh:
                fh.write(text)
        shim = os.path.join(d, "shim.c")
        with open(shim, "w") as fh:
            fh.write(SHIM)
        dll = os.path.join(d, "lsm.dll")
        cmd = ["clang", "-shared", "-nostdlib", "-Wl,-noentry", "-O1",
               "-fno-math-errno", "-w", "-I", d, "-I", SRC, shim, "-o", dll]
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode != 0:
            print(r.stdout + r.stderr)
            raise SystemExit("clang failed - this host test needs clang on PATH")
        self.lib = ctypes.CDLL(dll)
        L = self.lib
        ip = ctypes.POINTER(ctypes.c_int)
        L.t_noise.argtypes = [ip, ip, ctypes.c_uint]
        L.t_byte.argtypes = [ctypes.c_int]
        L.t_pattern.argtypes = [ctypes.POINTER(ctypes.c_uint8)]
        L.t_read.argtypes = [ctypes.POINTER(ctypes.c_float)]
        L.t_bias.argtypes = [ctypes.POINTER(ctypes.c_float)]
        L.t_set_bias.argtypes = [ctypes.c_float] * 3
        L.t_counts.argtypes = [ctypes.POINTER(ctypes.c_uint)]
        for f in ("t_dead", "t_drop_write", "t_glitch_read", "t_int1",
                  "t_reg", "t_set_int1_gpio"):
            getattr(L, f).argtypes = [ctypes.c_int]
        L.t_now.argtypes = [ctypes.c_longlong]
        L.t_diag.argtypes = [ctypes.POINTER(ctypes.c_int)]
        for f in ("t_init", "t_read", "t_cal", "t_frozen_reads", "t_code",
                  "t_reg", "t_int1_gpio", "t_sticky_reads"):
            getattr(L, f).restype = ctypes.c_int
        self.OK, self.RESP, self.STATE = (L.t_code(k) for k in range(3))

    def __enter__(self):
        return self

    def __exit__(self, *a):
        h = self.lib._handle
        del self.lib
        ctypes.windll.kernel32.FreeLibrary(ctypes.c_void_p(h))
        self._tmp.cleanup()

    def noise(self, base, amp=NOISE, seed=1):
        self.lib.t_noise((ctypes.c_int * 6)(*base), (ctypes.c_int * 6)(*amp),
                         seed)

    def pattern(self, p):
        self.lib.t_pattern((ctypes.c_uint8 * 12)(*p))

    def read(self):
        o = (ctypes.c_float * 6)()
        e = self.lib.t_read(o)
        return e, list(o)

    def bias(self):
        b = (ctypes.c_float * 3)()
        self.lib.t_bias(b)
        return list(b)

    def counts(self):
        o = (ctypes.c_uint * 4)()
        self.lib.t_counts(o)
        return list(o)

    def diag(self):
        o = (ctypes.c_int * 34)()
        self.lib.t_diag(o)
        o = list(o)

        def dump(p):
            return {"kind": p[0], "stuck": p[1], "int1": p[2],
                    "episode": p[3], "t_ms": p[4], "spi_err": p[5],
                    "regs": p[6:14]}
        return {"episodes": o[0], "longest": o[1], "current": o[2],
                "cfg_bad": o[3], "have_first": o[4], "have_sticky": o[5],
                "first": dump(o[6:20]), "sticky": dump(o[20:34])}


FAIL = 0


def check(name, got, want):
    global FAIL
    ok = got == want
    if not ok:
        FAIL += 1
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}")
    if not ok:
        print(f"         got {got!r}  want {want!r}")


def near(a, b, tol):
    return all(abs(x - y) <= tol for x, y in zip(a, b))


def boot_healthy(H):
    H.lib.t_reset()
    H.noise(HEALTHY)
    return H.lib.t_init()


def main():
    with Host() as H:
        FR = H.lib.t_frozen_reads()

        print("\na healthy chip still boots, calibrates and reads")
        check("init succeeds on a noisy, still chip", boot_healthy(H), 1)
        # IMU_REMAP: body = (+chip_y, -chip_x, +chip_z)
        want = [HEALTHY[1] * G, -HEALTHY[0] * G, HEALTHY[2] * G]
        check("bias is the remapped zero-rate offset (+-0.01 dps)",
              near(H.bias(), want, 0.01), True)
        errs = [H.read()[0] for _ in range(5000)]
        check("5000 noisy reads: every one ESP_OK", errs.count(H.OK), 5000)
        check("...and nothing counted as uniform or frozen",
              H.counts()[:2], [0, 0])

        print("\nUNIFORM: every byte the same is not a sample (incident 55: 0xF0)")
        for byte in (0xF0, 0x00, 0xFF):
            H.lib.t_byte(byte)
            res = [H.read() for _ in range(100)]
            check(f"0x{byte:02X} x12: all 100 reads refused",
                  sum(e == H.RESP for e, _ in res), 100)
            check(f"0x{byte:02X} x12: the caller's struct is left untouched",
                  all(o == [9.0] * 6 for _, o in res), True)
        check("uniform counter = 300, frozen counter still 0",
              H.counts()[:2], [300, 0])

        print(f"\nFROZEN: the same burst more than {FR} reads in a row")
        H.lib.t_reset()
        check("(re-boot healthy)", boot_healthy(H), 1)
        H.read()                                   # last_raw = a noise burst
        frozen = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66,
                  0x77, 0x88, 0x99, 0xAA, 0xBB, 0x0C]
        H.pattern(frozen)
        errs = [H.read()[0] for _ in range(FR + 1 + 51)]
        check(f"the first {FR + 1} identical bursts pass (a missed DRDY "
              "edge repeats one, not sixteen)",
              errs[:FR + 1], [H.OK] * (FR + 1))
        check("every identical burst after that is refused",
              errs[FR + 1:], [H.STATE] * 51)
        check("frozen counter counts the refusals", H.counts()[1], 51)
        H.noise(HEALTHY, seed=7)
        check("data moving again -> the very next read is accepted",
              H.read()[0], H.OK)

        print("\nGYROCAL refuses what the old calibration stored")
        H.lib.t_set_bias(0.1, 0.2, 0.3)
        H.lib.t_byte(0xF0)
        check("incident 55 bus (0xF0 everywhere): capture REFUSED",
              H.lib.t_cal(), H.RESP)
        check("...and the previous bias is kept, not overwritten with 67 dps",
              near(H.bias(), [0.1, 0.2, 0.3], 1e-6), True)
        H.pattern(frozen)
        check("frozen non-uniform bus: capture REFUSED", H.lib.t_cal(), H.RESP)
        H.noise(HEALTHY, amp=[0, 3, 3, 4, 4, 4])
        check("ONE gyro axis frozen, the others noisy: REFUSED",
              H.lib.t_cal(), H.RESP)
        base = list(HEALTHY)
        base[0] = 572                              # 10.01 dps on chip X
        H.noise(base)
        check("offset 10.01 dps (> LSM6DSOX_BIAS_MAX_DPS): REFUSED",
              H.lib.t_cal(), H.RESP)
        check("...previous bias still kept",
              near(H.bias(), [0.1, 0.2, 0.3], 1e-6), True)
        base[0] = 571                              # 9.99 dps
        H.noise(base)
        check("offset 9.99 dps: accepted", H.lib.t_cal(), H.OK)
        check("...and stored (body y = -chip x)",
              near(H.bias(), [HEALTHY[1] * G, -571 * G, HEALTHY[2] * G], 0.01),
              True)

        print("\nboot: an IMU with no real data fails init")
        H.lib.t_reset()
        H.lib.t_byte(0xF0)
        check("incident 55 at boot: lsm6dsox_init returns NULL", H.lib.t_init(), 0)
        check("...and gives the SPI device and bus back",
              H.counts()[2:], [1, 1])
        H.lib.t_reset()
        base = list(HEALTHY)
        base[1] = 1200                             # 21 dps on chip Y
        H.noise(base)
        check("impossible offset at boot: init returns NULL", H.lib.t_init(), 0)
        H.lib.t_reset()
        H.noise(HEALTHY, amp=[60, 60, 60, 4, 4, 4])
        check("a board MOVED during boot still inits (suspect bias, as before)",
              H.lib.t_init(), 1)

        print("\nSPI only : I2C and I3C off, BDU on, all read back")
        check("(boot healthy)", boot_healthy(H), 1)
        want = {0x18: 0xE2, 0x13: 0x06, 0x12: 0x44, 0x10: 0x6E, 0x11: 0x64,
                0x15: 0x00, 0x17: 0x09, 0x0D: 0x02}
        check("CTRL9_XL I3C_disable, CTRL4_C I2C_disable+LPF1, CTRL3_C "
              "BDU+IF_INC; ODR/FS/filters/INT1 as before",
              {r: H.lib.t_reg(r) for r in want}, want)
        check("...read back clean (cfg_bad 0)", H.diag()["cfg_bad"], 0)
        check("INT1 pin unknown to the driver until fusion sets it",
              H.lib.t_int1_gpio(), -1)
        for reg, bit, name in ((0x18, 0, "CTRL9_XL - I3C left on"),
                               (0x13, 1, "CTRL4_C - I2C left on"),
                               (0x11, 4, "CTRL2_G - gyro scale unknown")):
            H.lib.t_reset()
            H.noise(HEALTHY)
            H.lib.t_drop_write(reg)
            check(f"{name}: write ignored -> init returns NULL",
                  H.lib.t_init(), 0)
            check(f"...cfg_bad names it (bit {bit})", H.diag()["cfg_bad"],
                  1 << bit)
            check("...and gives the SPI device and bus back",
                  H.counts()[2:], [1, 1])
        H.lib.t_reset()
        H.noise(HEALTHY)
        H.lib.t_glitch_read(0x18)
        check("ONE garbled read-back: re-read, init still succeeds",
              H.lib.t_init(), 1)
        check("...cfg_bad 0", H.diag()["cfg_bad"], 0)

        ST = H.lib.t_sticky_reads()
        print("\nrefusal diagnostics: episodes, and the chip's registers at "
              "the moment it refused")
        H.lib.t_reset()
        check("(boot healthy)", boot_healthy(H), 1)
        H.lib.t_set_int1_gpio(15)
        for _ in range(50):
            H.read()
        g = H.diag()
        check("50 healthy reads: no episode, no dump",
              (g["episodes"], g["have_first"]), (0, 0))

        # The freeze: not on the bus at all, INT1 high.
        H.lib.t_now(860_829_000)
        H.lib.t_int1(1)
        H.lib.t_dead(0xE9)
        errs = [H.read()[0] for _ in range(3)]
        g = H.diag()
        check("chip off the bus: 3 bursts refused", errs, [H.RESP] * 3)
        check("...one episode, longest 3, current 3",
              (g["episodes"], g["longest"], g["current"]), (1, 3, 3))
        f = g["first"]
        check("first dump: uniform 0xE9, INT1 HIGH, episode 1, at 860829 ms",
              (f["kind"], f["stuck"], f["int1"], f["episode"], f["t_ms"]),
              (1, 0xE9, 1, 1, 860829))
        check("...every register reads the stuck byte", f["regs"], [0xE9] * 8)
        check("...no second dump for 3 refusals", g["have_sticky"], 0)
        H.noise(HEALTHY, seed=3)
        check("chip back: the next read is accepted", H.read()[0], H.OK)
        g = H.diag()
        check("episode closed: current 0, longest kept, still 1 episode",
              (g["current"], g["longest"], g["episodes"]), (0, 3, 1))

        # Uniform data from a chip that still answers, INT1 low at first.
        H.lib.t_now(900_000_000)
        H.lib.t_int1(0)
        H.lib.t_byte(0xF0)
        for _ in range(ST - 1):
            H.read()
        g = H.diag()
        check(f"{ST - 1} refusals: episode 2, no sticky dump yet",
              (g["episodes"], g["have_sticky"]), (2, 0))
        H.lib.t_now(901_050_000)
        H.lib.t_int1(1)
        H.read()
        g = H.diag()
        check(f"refusal {ST}: sticky dump taken", g["have_sticky"], 1)
        check("...episode 2, at 901050 ms, INT1 HIGH",
              (g["sticky"]["episode"], g["sticky"]["t_ms"], g["sticky"]["int1"]),
              (2, 901050, 1))
        check("episode 2 first dump: WHO_AM_I 6C and the config intact",
              g["first"]["regs"], [0x6C, 0x02, 0x6E, 0x64, 0x44, 0x06, 0xE2, 0x00])
        check("...INT1 LOW when it started", g["first"]["int1"], 0)
        H.lib.t_now(905_000_000)
        for _ in range(50):
            H.read()
        check("one sticky dump per episode (not re-taken)",
              H.diag()["sticky"]["t_ms"], 901050)
        H.noise(HEALTHY, seed=5)
        H.read()
        H.read()

        # A FROZEN run is an episode too; a new episode drops the old
        # episode's sticky dump.
        H.pattern(frozen)
        errs = [H.read()[0] for _ in range(FR + 1 + 2)]
        g = H.diag()
        check("frozen run: 17 accepted, 2 refused",
              (errs.count(H.OK), errs.count(H.STATE)), (FR + 1, 2))
        check("...episode 3, kind FROZEN, longest still episode 2's",
              (g["episodes"], g["first"]["kind"], g["longest"]),
              (3, 2, ST + 50))
        check("...episode 2's sticky dump no longer reported",
              g["have_sticky"], 0)

    print(f"\n{FAIL} FAILURE(S)\n" if FAIL else "\nALL PASS\n")
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
