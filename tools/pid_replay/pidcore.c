/*
 * pidcore - the flight controller's rate loop and mixer, built as a host
 * shared library so tools can run the REAL code instead of a port of it.
 *
 * It #includes components/pid/pid.c and components/mixer/mixer.c. Not a
 * copy, not a re-implementation: the same translation unit the aircraft
 * flies, so an edit to either lands here on the next build and a tuner
 * can never quietly tune a controller that is not on the board.
 *
 * FREESTANDING ON PURPOSE
 * -----------------------
 * Neither file touches libc - pid.c is pure float arithmetic, mixer.c
 * pure integer, and stdint/stdbool are freestanding headers clang ships
 * itself. So this builds with -nostdlib and needs no Windows SDK, no
 * MSVC, no mingw: a bare clang is enough, which is all a machine set up
 * for ESP-IDF is guaranteed to have. The one price is `_fltused`, an
 * MSVC CRT symbol the compiler emits a reference to whenever a
 * translation unit uses floats; defining it here satisfies the linker
 * and does nothing else.
 *
 * Everything else - CSV, sweeps, scoring, plots - stays in Python, where
 * changing it does not risk changing the controller.
 */
#include "pid.c"
#include "mixer.c"

#if defined(_WIN32)
#define EXPORT __declspec(dllexport)
int _fltused = 0;               /* see above; never read */
#else
#define EXPORT __attribute__((visibility("default")))
#endif

/* The ctypes side declares these structs by hand, so it has to agree on
 * their layout. Publish the sizes and let the binding assert on them -
 * a silent field-order mismatch would produce numbers that look
 * plausible and are wrong, which is the worst failure available here. */
EXPORT int pc_sizeof_cfg(void)   { return (int)sizeof(pid_cfg_t); }
EXPORT int pc_sizeof_state(void) { return (int)sizeof(pid_state_t); }

EXPORT void pc_pid_init(pid_state_t *s) { pid_init(s); }

EXPORT float pc_pid_step(const pid_cfg_t *cfg, pid_state_t *st,
                         float setpoint, float measured, float dt,
                         int saturated)
{
    return pid_calculate_sat(cfg, st, setpoint, measured, dt, saturated);
}

EXPORT unsigned long pc_windup_freezes(void)
{
    return pid_windup_freeze_count();
}

/* Returns the mixer's saturation flag; motors[4] is filled in. */
EXPORT int pc_mix(unsigned short throttle, short roll, short pitch,
                  short yaw, unsigned short idle_us, unsigned short *motors)
{
    return mixer_quad_x(throttle, roll, pitch, yaw, idle_us, motors);
}
