#!/usr/bin/env python3
"""ctypes binding to the firmware's own PID and mixer (see pidcore.c).

    from pidcore import PidCfg, PidState, pid_step, mix

The library is REBUILT AUTOMATICALLY whenever pidcore.c, pid.c, pid.h,
mixer.c or mixer.h is newer than the binary. That is not a convenience -
it is the guarantee the whole approach rests on. A stale .dll would mean
a host tool quietly scoring a controller the aircraft stopped running,
and nothing about the output would look wrong.

Windows needs no MSVC and no SDK: pidcore.c is freestanding, so a bare
clang links it with -nostdlib. Elsewhere any cc will do.
"""

import ctypes
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
PID_DIR = os.path.join(ROOT, "components", "pid")
MIXER_DIR = os.path.join(ROOT, "components", "mixer")

SRC = os.path.join(HERE, "pidcore.c")
DEPS = [SRC,
        os.path.join(PID_DIR, "pid.c"), os.path.join(PID_DIR, "pid.h"),
        os.path.join(MIXER_DIR, "mixer.c"), os.path.join(MIXER_DIR, "mixer.h")]
LIB = os.path.join(HERE, "pidcore.dll" if os.name == "nt" else "pidcore.so")

# fc_tasks.c:292 - compile-time, not params. Only the gains are tunable.
RATE_I_MAX = 100.0
RATE_OUT_MAX = 400.0
RATE_TAU = 0.015
# fc_tasks.c:289 - the ANGLE loops. ki/kd/i_max are zero by design, so
# there is no integrator and des inverts straight out of the log.
ANGLE_OUT_MAX = 200.0


class PidCfg(ctypes.Structure):
    _fields_ = [("kp", ctypes.c_float), ("ki", ctypes.c_float),
                ("kd", ctypes.c_float), ("i_max", ctypes.c_float),
                ("out_max", ctypes.c_float), ("tau", ctypes.c_float)]


class PidState(ctypes.Structure):
    _fields_ = [("integral", ctypes.c_float), ("prev_error", ctypes.c_float),
                ("prev_meas", ctypes.c_float), ("d_filtered", ctypes.c_float)]


def rate_cfg(kp, ki, kd):
    """A rate-loop config with the firmware's compile-time limits."""
    return PidCfg(kp, ki, kd, RATE_I_MAX, RATE_OUT_MAX, RATE_TAU)


def _build():
    cc = os.environ.get("CC") or ("clang" if os.name == "nt" else "cc")
    common = ["-O2", "-I", PID_DIR, "-I", MIXER_DIR, "-o", LIB, SRC]
    if os.name == "nt":
        # Freestanding: no CRT, no SDK, no entry point. -noentry rather
        # than /noentry because a bash-flavoured shell rewrites a leading
        # slash into a path.
        cmd = [cc, "--target=x86_64-pc-windows-msvc", "-ffreestanding",
               "-nostdlib", "-shared", "-fuse-ld=lld", "-Wl,-noentry"] + common
    else:
        cmd = [cc, "-shared", "-fPIC", "-fvisibility=hidden"] + common
    try:
        subprocess.run(cmd, check=True, capture_output=True, text=True)
    except FileNotFoundError:
        sys.exit(f"pidcore: no C compiler ('{cc}'). Set CC, or put clang on "
                 "PATH - an ESP-IDF install already ships one.")
    except subprocess.CalledProcessError as e:
        sys.exit(f"pidcore: build failed\n$ {' '.join(cmd)}\n{e.stderr}")


def _load():
    newest = max(os.path.getmtime(d) for d in DEPS)
    if not os.path.exists(LIB) or os.path.getmtime(LIB) < newest:
        _build()
    lib = ctypes.CDLL(LIB)

    lib.pc_sizeof_cfg.restype = ctypes.c_int
    lib.pc_sizeof_state.restype = ctypes.c_int
    # If these ever disagree the bindings are reading the wrong bytes and
    # every number downstream is fiction. Fail here, loudly.
    assert lib.pc_sizeof_cfg() == ctypes.sizeof(PidCfg), \
        f"pid_cfg_t is {lib.pc_sizeof_cfg()} B, binding has " \
        f"{ctypes.sizeof(PidCfg)} B - pid.h changed, update pidcore.py"
    assert lib.pc_sizeof_state() == ctypes.sizeof(PidState), \
        f"pid_state_t is {lib.pc_sizeof_state()} B, binding has " \
        f"{ctypes.sizeof(PidState)} B - pid.h changed, update pidcore.py"

    lib.pc_pid_init.argtypes = [ctypes.POINTER(PidState)]
    lib.pc_pid_step.restype = ctypes.c_float
    lib.pc_pid_step.argtypes = [ctypes.POINTER(PidCfg),
                                ctypes.POINTER(PidState),
                                ctypes.c_float, ctypes.c_float,
                                ctypes.c_float, ctypes.c_int]
    lib.pc_windup_freezes.restype = ctypes.c_ulong
    lib.pc_mix.restype = ctypes.c_int
    lib.pc_mix.argtypes = [ctypes.c_ushort, ctypes.c_short, ctypes.c_short,
                           ctypes.c_short, ctypes.c_ushort,
                           ctypes.POINTER(ctypes.c_ushort)]
    return lib


_LIB = _load()

_MOTORS = (ctypes.c_ushort * 4)()


def pid_init(state):
    _LIB.pc_pid_init(ctypes.byref(state))


def pid_step(cfg, state, setpoint, measured, dt, saturated=0):
    return _LIB.pc_pid_step(ctypes.byref(cfg), ctypes.byref(state),
                            setpoint, measured, dt, saturated)


def mix(throttle, roll, pitch, yaw, idle_us):
    """-> (saturated, [m1, m2, m3, m4]). Args are truncated the way the
    firmware truncates them: fc_tasks.c casts the float PID outputs to
    int16 before the call, so the replay must too."""
    sat = _LIB.pc_mix(int(throttle), int(roll), int(pitch), int(yaw),
                      int(idle_us), _MOTORS)
    return sat, list(_MOTORS)


def windup_freezes():
    return _LIB.pc_windup_freezes()


if __name__ == "__main__":
    # Smoke test: a 10 dps step into a P-only-ish loop. P = 1.2*10 = 12,
    # and the integrator accumulates 10*dt per cycle, so the output must
    # creep by ki*10*dt = 0.002 us per 4 ms cycle.
    cfg = rate_cfg(1.2, 0.05, 0.03)
    st = PidState()
    pid_init(st)
    out = [round(pid_step(cfg, st, 10.0, 0.0, 0.004), 5) for _ in range(4)]
    print(f"library: {LIB}")
    print(f"4 x step(setpoint=10 dps, measured=0, dt=4 ms) -> {out}")
    print(f"integral = {st.integral:.6f} (expected {4 * 10 * 0.004:.6f})")
    sat, m = mix(1500, 0, 50, 0, 1050)
    print(f"mix(thr=1500, pitch=+50) -> {m} saturated={sat}"
          "   (+pitch must raise the REAR pair m3/m4)")
