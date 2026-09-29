#!/usr/bin/env python3
"""Host test for pid_retune_ki() in components/pid/pid.c.

Same trick as tools/host_c.py: pid.c includes only pid.h, so clang can
compile the exact file that flies and ctypes can drive it.

The property under test is stated once and then checked by construction:

    an aircraft that has been integrating error e(t) with gain ki, and then
    has ki changed to ki', should end up with the integrator it would have
    had if it had been flying with ki' the whole time.

REWRITTEN (anti-windup). The integrator became RAW - ki is now
applied at the OUTPUT - so that property is satisfied STRUCTURALLY: stored
history no longer depends on ki at all, and the rescale this file was
written to check has become a no-op. Checking it now would be a tautology,
so the assertions moved to what is actually still load-bearing:

  * the raw integral is genuinely independent of ki (the reason the
    rescale could be dropped)
  * i_max is enforced on the OUTPUT CONTRIBUTION, ki * integral, which is
    the quantity it has always bounded
  * a retune re-clamps immediately rather than letting one oversized
    command out first

Nothing is compared against pid_retune_ki's own arithmetic.

PID_SRC points the compiler at a doctored copy for mutation runs.

Run:  python tools/pid_retune_test.py
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
EXPORT int   p_retune(pid_state_t *s, float o, float n, float im){
    return pid_retune_ki(s, o, n, im);
}
EXPORT int   p_state_size(void){ return (int)sizeof(pid_state_t); }
EXPORT int   p_cfg_size(void){ return (int)sizeof(pid_cfg_t); }
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
        dll = os.path.join(self._tmp.name, "pid.dll")
        cmd = ["clang", "-shared", "-nostdlib", "-Wl,-noentry", "-O1",
               "-fno-math-errno", "-I", SRC, shim, "-o", dll]
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode != 0:
            print(r.stdout + r.stderr)
            raise SystemExit("clang failed - this host test needs clang on PATH")
        self.lib = ctypes.CDLL(dll)
        L, ps, pc = self.lib, ctypes.POINTER(CState), ctypes.POINTER(CCfg)
        f = ctypes.c_float
        L.p_init.argtypes = [ps]
        L.p_calc.argtypes = [pc, ps, f, f, f]
        L.p_calc.restype = f
        L.p_retune.argtypes = [ps, f, f, f]
        L.p_retune.restype = ctypes.c_int
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

    def cfg(self, ki, i_max=100.0, kp=0.6, kd=0.01):
        return CCfg(kp=kp, ki=ki, kd=kd, i_max=i_max, out_max=400.0, tau=0.015)

    def fresh(self):
        s = CState()
        self.lib.p_init(ctypes.byref(s))
        return s

    def run(self, cfg, state, errors, dt=0.004, meas=None):
        """Drive pid_calculate with a setpoint (and optional measurement)
        history.

        `meas` matters for the D-term: this controller differentiates the
        MEASUREMENT, so a run that holds it at a constant leaves d_filtered
        at exactly 0 - and any test asserting d_filtered survives a retune
        is then vacuous. That gap let a mutant which zeroed d_filtered pass
        the whole suite."""
        for i, e in enumerate(errors):
            m = 0.0 if meas is None else meas[i % len(meas)]
            self.lib.p_calc(ctypes.byref(cfg), ctypes.byref(state), e, m, dt)
        return state


FAIL = 0


def near(name, got, want, tol=1e-4):
    global FAIL
    ok = abs(got - want) <= tol
    if not ok:
        FAIL += 1
    print(f"  [{'PASS' if ok else 'FAIL'}] {name:<52} {got:+.5f} "
          f"(want {want:+.5f})")


def check(name, got, want):
    global FAIL
    ok = got == want
    if not ok:
        FAIL += 1
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}")
    if not ok:
        print(f"         got {got!r}  want {want!r}")


# A realistic-ish rate error history: a step the loop is fighting.
HIST = [8.0] * 40 + [5.0] * 40 + [2.0] * 40 + [-1.0] * 20


def main():
    with Host() as H:
        print("\nthe property: a retuned controller must match one that flew")
        print("the new gain from the start (now structural, not arithmetic)")
        for ki_old, ki_new in ((0.05, 0.20), (0.20, 0.05), (0.10, 0.11),
                               (0.02, 1.50)):
            a = H.fresh()
            H.run(H.cfg(ki_old), a, HIST)
            H.lib.p_retune(ctypes.byref(a), ki_old, ki_new, 100.0)

            b = H.fresh()
            H.run(H.cfg(ki_new), b, HIST)
            near(f"ki {ki_old} -> {ki_new}: same integrator",
                 a.integral, b.integral, tol=2e-3)
            near(f"     and the same OUTPUT contribution",
                 a.integral * ki_new, b.integral * ki_new, tol=2e-3)

        print("\nunchanged gain must not touch anything")
        s = H.fresh()
        H.run(H.cfg(0.1), s, HIST)
        i0, pm, df = s.integral, s.prev_meas, s.d_filtered
        check("returns 0", H.lib.p_retune(ctypes.byref(s), 0.1, 0.1, 100.0), 0)
        check("state byte-identical",
              (s.integral, s.prev_meas, s.d_filtered),
              (i0, pm, df))

        print("\nedge cases")
        s = H.fresh()
        H.run(H.cfg(0.0), s, HIST)
        check("ki was 0 -> nothing had accumulated", round(s.integral, 6), 0.0)
        check("     retune reports a change",
              H.lib.p_retune(ctypes.byref(s), 0.0, 0.3, 100.0), 1)
        check("     and leaves it at 0", round(s.integral, 6), 0.0)

        s = H.fresh()
        H.run(H.cfg(0.2), s, HIST)
        H.lib.p_retune(ctypes.byref(s), 0.2, 0.0, 100.0)
        near("ki -> 0 empties the integrator", s.integral, 0.0)

        print("\nre-clamp: a big ki increase must not smuggle past i_max")
        # i_max bounds ki*integral, so a big ki rise can leave a legal
        # raw sum producing more than i_max. The retune must re-clamp on
        # the spot rather than let one oversized command out first.
        s = H.fresh()
        H.run(H.cfg(0.2, i_max=100.0), s, HIST)
        pre = s.integral
        KI_NEW = 100.0
        check(f"raw {pre:.2f} at ki {KI_NEW} would contribute "
              f"{pre*KI_NEW:.0f}, past i_max 100",
              abs(pre * KI_NEW) > 100.0, True)
        H.lib.p_retune(ctypes.byref(s), 0.2, KI_NEW, 100.0)
        near("     output contribution clamped to i_max",
             s.integral * KI_NEW, 100.0, 1e-2)
        s = H.fresh()
        H.run(H.cfg(0.2), s, [-e for e in HIST])
        H.lib.p_retune(ctypes.byref(s), 0.2, KI_NEW, 100.0)
        near("     and to -i_max on the negative side",
             s.integral * KI_NEW, -100.0, 1e-2)

        print("\nthe D-term and error history must survive a retune")
        # A MOVING measurement, so d_filtered is genuinely non-zero.
        wobble = [0.0, 1.5, -0.8, 2.2, -1.9, 0.4]
        s = H.fresh()
        H.run(H.cfg(0.1), s, HIST, meas=wobble)
        pm, df = s.prev_meas, s.d_filtered
        check("     fixture leaves a non-zero d_filtered to protect",
              abs(df) > 1e-6, True)
        H.lib.p_retune(ctypes.byref(s), 0.1, 0.4, 100.0)
        check("prev_meas / d_filtered untouched",
              (s.prev_meas, s.d_filtered), (pm, df))

        print("\nwhy not just reset: the trim survives")
        s = H.fresh()
        H.run(H.cfg(0.1), s, HIST)
        H.lib.p_retune(ctypes.byref(s), 0.1, 0.12, 100.0)
        check("a 20 % ki nudge leaves the integrator non-zero",
              abs(s.integral) > 0.1, True)
        print("       (a reset would have dropped it to 0.0 and with it the")
        print("        trim the aircraft was holding - mid-flight)")

        print("\nand ki == 0 still empties it, matching pid_calculate_sat")
        s = H.fresh()
        H.run(H.cfg(0.2), s, HIST)
        check("non-zero before", abs(s.integral) > 0.0, True)
        H.lib.p_retune(ctypes.byref(s), 0.2, 0.0, 100.0)
        near("     zeroed when the term is switched off", s.integral, 0.0)

    print(f"\n{FAIL} FAILURE(S)\n" if FAIL else "\nALL PASS\n")
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
