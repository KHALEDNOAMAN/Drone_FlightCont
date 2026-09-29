#!/usr/bin/env python3
"""Host test for the anti-windup rework in components/pid/pid.c.

Two separate claims are checked, and the first one matters most:

  1. THE REFACTOR IS OUTPUT-IDENTICAL. Moving ki out of the accumulator and
     onto the output must not change a single command the aircraft would
     have received. The reference is the OLD algorithm, re-implemented here
     in Python from the early  source, and driven with the same
     input - including inputs that drive the i_max clamp hard, since that
     is where the two forms could diverge.

  2. THE FREEZE IS CONDITIONAL. While the mixer is saturated the integrator
     must not grow, but must still be free to shrink. A blanket freeze
     would trap it at whatever it held when authority ran out.

PID_SRC points the compiler at a doctored copy for mutation runs.

Run:  python tools/pid_windup_test.py
"""

import ctypes
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
SRC = os.environ.get("PID_SRC") or os.path.join(ROOT, "components", "pid")

SHIM = r"""
int _fltused = 0;
#include "pid.c"
#define EXPORT __declspec(dllexport)
EXPORT void  p_init(pid_state_t *s){ pid_init(s); }
EXPORT float p_calc(const pid_cfg_t *c, pid_state_t *s, float sp, float mv, float dt){
    return pid_calculate(c, s, sp, mv, dt);
}
EXPORT float p_calcsat(const pid_cfg_t *c, pid_state_t *s, float sp, float mv,
                       float dt, int sat){
    return pid_calculate_sat(c, s, sp, mv, dt, sat);
}
EXPORT unsigned long p_freezes(void){ return pid_windup_freeze_count(); }
EXPORT int p_state_size(void){ return (int)sizeof(pid_state_t); }
EXPORT int p_cfg_size(void){ return (int)sizeof(pid_cfg_t); }
"""


class CCfg(ctypes.Structure):
    _fields_ = [(n, ctypes.c_float) for n in
                ("kp", "ki", "kd", "i_max", "out_max", "tau")]


class CState(ctypes.Structure):
    _fields_ = [(n, ctypes.c_float) for n in
                ("integral", "prev_meas", "d_filtered")]


class Host:
    def __init__(self):
        self._tmp = tempfile.TemporaryDirectory()
        shim = os.path.join(self._tmp.name, "shim.c")
        with open(shim, "w") as fh:
            fh.write(SHIM)
        dll = os.path.join(self._tmp.name, "pidw.dll")
        r = subprocess.run(["clang", "-shared", "-nostdlib", "-Wl,-noentry",
                            "-O1", "-fno-math-errno", "-I", SRC, shim,
                            "-o", dll], capture_output=True, text=True)
        if r.returncode != 0:
            print(r.stdout + r.stderr)
            raise SystemExit("clang failed - this host test needs clang on PATH")
        self.lib = ctypes.CDLL(dll)
        L = self.lib
        ps, pc, f = ctypes.POINTER(CState), ctypes.POINTER(CCfg), ctypes.c_float
        L.p_init.argtypes = [ps]
        L.p_calc.argtypes = [pc, ps, f, f, f]; L.p_calc.restype = f
        L.p_calcsat.argtypes = [pc, ps, f, f, f, ctypes.c_int]
        L.p_calcsat.restype = f
        L.p_freezes.restype = ctypes.c_ulong
        L.p_state_size.restype = ctypes.c_int
        L.p_cfg_size.restype = ctypes.c_int
        for nm, got, want in (("pid_state_t", L.p_state_size(), ctypes.sizeof(CState)),
                              ("pid_cfg_t", L.p_cfg_size(), ctypes.sizeof(CCfg))):
            if got != want:
                raise SystemExit(f"{nm} layout mismatch: C {got} B, ctypes {want} B")

    def __enter__(self):
        return self

    def __exit__(self, *a):
        h = self.lib._handle
        del self.lib
        ctypes.windll.kernel32.FreeLibrary(ctypes.c_void_p(h))
        self._tmp.cleanup()

    def cfg(self, kp=1.2, ki=0.05, kd=0.03, i_max=100.0, out_max=400.0, tau=0.015):
        return CCfg(kp=kp, ki=ki, kd=kd, i_max=i_max, out_max=out_max, tau=tau)

    def fresh(self):
        s = CState()
        self.lib.p_init(ctypes.byref(s))
        return s


def old_pid_reference(kp, ki, kd, i_max, out_max, tau, samples, dt=0.004):
    """The PRE- algorithm, transcribed from the old source:
    ki folded into the accumulator, i_max clamping that scaled value."""
    integ = prev_meas = d_filt = 0.0
    out = []
    for sp, mv in samples:
        e = sp - mv
        P = e * kp
        if dt > 0.0:
            integ += e * dt * ki
            if integ > i_max:
                integ = i_max
            elif integ < -i_max:
                integ = -i_max
        I = integ
        D = 0.0
        if dt > 0.0:
            raw_d = -(mv - prev_meas) / dt
            alpha = dt / (dt + tau) if tau > 0.0 else 1.0
            d_filt += alpha * (raw_d - d_filt)
            D = d_filt * kd
        prev_meas = mv
        o = P + I + D
        o = max(-out_max, min(out_max, o))
        out.append(o)
    return out


FAIL = 0


def check(name, got, want):
    global FAIL
    ok = got == want
    if not ok:
        FAIL += 1
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}")
    if not ok:
        print(f"         got {got!r}  want {want!r}")


def near(name, got, want, tol=1e-3):
    global FAIL
    ok = abs(got - want) <= tol
    if not ok:
        FAIL += 1
    print(f"  [{'PASS' if ok else 'FAIL'}] {name:<50} {got:+.5f} "
          f"(want {want:+.5f})")


# Histories: a step the loop fights, and one big enough to slam the clamp.
STEP = [(8.0, 0.0)] * 60 + [(2.0, 1.0)] * 60 + [(-4.0, 0.5)] * 60
WOBBLE = [(5.0 * (1 if i % 7 < 4 else -1), 0.3 * (i % 5) - 0.6)
          for i in range(200)]
# Long and hard enough that the raw sum (1000*0.004*1500 = 6000) puts
# ki*integral past i_max=100 even at the LOWEST gain tested (0.02 ->
# 120). A fixture that does not actually reach the clamp proves
# nothing about the clamp.
SLAM = [(1000.0, 0.0)] * 1500


def main():
    with Host() as H:
        print("\n1. THE REFACTOR MUST NOT CHANGE A SINGLE COMMAND")
        print("   (new C vs the old algorithm re-implemented in Python)")
        for label, hist in (("a step the loop fights", STEP),
                            ("an oscillating setpoint", WOBBLE),
                            ("hard against i_max for 1.6 s", SLAM)):
            for ki in (0.05, 0.20, 0.5):
                cfg = H.cfg(ki=ki)
                st = H.fresh()
                got = [H.lib.p_calc(ctypes.byref(cfg), ctypes.byref(st),
                                    sp, mv, 0.004) for sp, mv in hist]
                want = old_pid_reference(cfg.kp, ki, cfg.kd, cfg.i_max,
                                         cfg.out_max, cfg.tau, hist)
                worst = max(abs(a - b) for a, b in zip(got, want))
                near(f"{label}, ki={ki}: worst deviation", worst, 0.0, 2e-3)

        print("\n   the clamp really was exercised (or the test proves little)")
        cfg = H.cfg(ki=0.05)
        st = H.fresh()
        for sp, mv in SLAM:
            H.lib.p_calc(ctypes.byref(cfg), ctypes.byref(st), sp, mv, 0.004)
        near("     output contribution pinned at i_max",
             st.integral * cfg.ki, cfg.i_max, 1e-2)

        print("\n2. i_max now bounds the OUTPUT CONTRIBUTION, as before")
        for ki in (0.02, 0.05, 0.4):
            cfg = H.cfg(ki=ki)
            st = H.fresh()
            for sp, mv in SLAM:
                H.lib.p_calc(ctypes.byref(cfg), ctypes.byref(st), sp, mv, 0.004)
            near(f"ki={ki}: ki*integral saturates at i_max",
                 st.integral * ki, cfg.i_max, 1e-2)

        print("\n3. the raw integral is now INDEPENDENT of ki")
        vals = []
        for ki in (0.05, 0.20, 0.50):
            st = H.fresh()
            cfg = H.cfg(ki=ki, i_max=1e9)     # clamp out of the way
            for sp, mv in STEP:
                H.lib.p_calc(ctypes.byref(cfg), ctypes.byref(st), sp, mv, 0.004)
            vals.append(st.integral)
        near("same accumulated history at every gain",
             max(vals) - min(vals), 0.0, 1e-4)
        print("       (this is why pid_retune_ki no longer has to rescale)")

        print("\n4. ki == 0 holds the accumulator at zero")
        st = H.fresh()
        cfg = H.cfg(ki=0.0)
        for sp, mv in STEP:
            H.lib.p_calc(ctypes.byref(cfg), ctypes.byref(st), sp, mv, 0.004)
        near("nothing accumulated while the term is off", st.integral, 0.0)

        print("\n5. ANTI-WINDUP: frozen while saturated, but only upward")
        cfg = H.cfg(ki=0.05)
        st = H.fresh()
        for sp, mv in [(8.0, 0.0)] * 50:      # build a positive integral
            H.lib.p_calc(ctypes.byref(cfg), ctypes.byref(st), sp, mv, 0.004)
        base = st.integral
        check("built a positive integral", base > 0.0, True)

        before = H.lib.p_freezes()
        for sp, mv in [(8.0, 0.0)] * 50:      # same-sign error, saturated
            H.lib.p_calcsat(ctypes.byref(cfg), ctypes.byref(st), sp, mv,
                            0.004, 1)
        near("saturated + error would GROW it -> frozen", st.integral, base)
        check("     and the freeze was counted",
              H.lib.p_freezes() - before == 50, True)

        for sp, mv in [(-8.0, 0.0)] * 20:     # opposite sign, still saturated
            H.lib.p_calcsat(ctypes.byref(cfg), ctypes.byref(st), sp, mv,
                            0.004, 1)
        check("saturated + error would SHRINK it -> still integrates",
              st.integral < base, True)

        print("\n   unsaturated behaves exactly as before")
        a, b = H.fresh(), H.fresh()
        for sp, mv in STEP:
            H.lib.p_calc(ctypes.byref(cfg), ctypes.byref(a), sp, mv, 0.004)
            H.lib.p_calcsat(ctypes.byref(cfg), ctypes.byref(b), sp, mv,
                            0.004, 0)
        near("pid_calculate == pid_calculate_sat(sat=0)", a.integral, b.integral)

        print("\n6. the windup it prevents, shown end to end")
        cfg = H.cfg(ki=0.05)
        naive, guarded = H.fresh(), H.fresh()
        # 2 s pinned against a limit the aircraft cannot answer...
        for sp, mv in [(30.0, 0.0)] * 500:
            H.lib.p_calcsat(ctypes.byref(cfg), ctypes.byref(naive), sp, mv,
                            0.004, 0)     # old behaviour: keeps integrating
            H.lib.p_calcsat(ctypes.byref(cfg), ctypes.byref(guarded), sp, mv,
                            0.004, 1)     # new: frozen while saturated
        check("without the guard the integrator is far larger",
              naive.integral > guarded.integral * 2.0, True)
        print(f"       naive {naive.integral:+.2f} vs guarded "
              f"{guarded.integral:+.2f} (raw units)")
        print("       that difference is command the aircraft would have")
        print("       carried into the moment authority came back")

    print(f"\n{FAIL} FAILURE(S)\n" if FAIL else "\nALL PASS\n")
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
