"""Shared definitions for the host tools - one copy instead of one per tool.

HEALTH_BITS mirrors SENSOR_* in components/common/nav_state.h (the D-line
field 31 / telemetry CSV field 52 / blackbox sensor_health byte). Keep it in
step with that header; every tool that names health bits imports it from
here.

open_port() opens the S3's USB console WITHOUT resetting it: pyserial asserts
DTR/RTS on open, which resets the S3, so both are set low on an UNOPENED port
first. pyserial is imported lazily so a tool that only
parses files still runs on a machine without it.
"""

HEALTH_BITS = [(1, "IMU"), (2, "BARO"), (4, "MAG"), (8, "MAG_GATED"),
               (16, "BARO2")]


def health_names(v):
    """(present, missing) bit names for a sensor_health value."""
    ok = [n for b, n in HEALTH_BITS if v & b]
    bad = [n for b, n in HEALTH_BITS if not v & b]
    return ok, bad


def open_port(port, baud=115200, timeout=0.2, no_reset=True, serial_cls=None):
    """Open `port`. no_reset=True keeps the S3 running (DTR/RTS held low).
    serial_cls overrides serial.Serial - the seam blackbox_pull_test uses."""
    if serial_cls is None:
        import serial
        serial_cls = serial.Serial
    if not no_reset:
        return serial_cls(port, baud, timeout=timeout)
    s = serial_cls()
    s.port, s.baudrate, s.timeout = port, baud, timeout
    s.dtr = s.rts = False
    s.open()
    return s
