#!/usr/bin/env python3
"""Live viewer for attitude, height and the flow-derived ground track.

Reads the 50 Hz `D,` line off the USB console, serves a page on localhost,
and does all the maths HERE - nothing new runs on the flight controller.
The firmware publishes raw counts; this decides what they mean. That is
deliberate: the flow sensor is passive until it has earned its place, and
a scale factor nobody has measured yet has no business in the flight path.

    python tools/viewer.py                 # auto-pick the S3's port
    python tools/viewer.py --port COM4
    python tools/viewer.py --record run.csv

Then open http://127.0.0.1:8765/ (it opens itself unless --no-open).

THE PACK MUST BE IN. On USB alone the sensor rail is dead - every sensor
reads zero, the flow sensor does not answer, and the page will correctly
show a lot of nothing ("WITH THE S3 ON USB, THE WHOLE SENSOR RAIL
IS DOWN. DO NOT INVESTIGATE IT").

WHAT THE TRACK IS, AND IS NOT. It is dead reckoning from optical flow:
velocity = height * (flow rate - gyro rate), integrated. It drifts, it
needs a height, and it is meaningless below the sensor's 80 mm focus
distance or over a featureless surface. It is a bench instrument for
checking that flow responds correctly and for measuring the counts-per-
radian scale - not a position estimate to fly on.
"""

import argparse
import json
import math
import os
import sys
import threading
import time
import webbrowser
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    # Refused in main(), not here: the decode and flow maths below need no
    # serial port, and tools/viewer_test.py imports this file to test them
    # on CI, which is stdlib-only. Exiting at import failed that job.
    serial = list_ports = None

HERE = os.path.dirname(os.path.abspath(__file__))
PAGE = os.path.join(HERE, "viewer.html")

# D line field indices. 0-31 are the frozen original layout, 32-40 were
# appended for this viewer (see the comment over stream_task in
# main/debug_stream.c). Every read is guarded on the line's LENGTH, so an
# older firmware degrades to "no flow" instead of crashing the page - the
# same rule the C6 dashboard follows.
D_MIN_FIELDS = 32          # the pre-flow layout
D_FLOW_FIELDS = 41         # with flow + ToF appended

I_T_US, I_ACC, I_GYRO, I_MAG = 1, 2, 5, 8
I_QUAT, I_ROLL, I_FUSION_DT = 11, 15, 18
I_MOTOR, I_ARMED, I_HEARTBEAT, I_ALT = 20, 24, 25, 26
I_LAT, I_LON, I_SATS, I_FIX, I_HEALTH = 27, 28, 29, 30, 31
I_TOF, I_FSEQ, I_FSUMX, I_FSUMY = 32, 33, 34, 35
I_FVALID, I_FLOST, I_FSQUAL, I_FSHUT, I_FFLAGS = 36, 37, 38, 39, 40

from fc_common import HEALTH_BITS  # noqa: E402  (nav_state.h SENSOR_*)
FLOW_FLAGS = [(1, "VALID"), (2, "MOTION"), (4, "FIRST"), (8, "BUS_STUCK"),
              (16, "TORN"), (32, "RAILED"), (64, "RATE"), (128, "AFTER_LOST"),
              (256, "BAD_DT")]

# Published scalings disagree by 2x - ArduPilot 1.26e-3 rad/count, PX4
# 1/385 = 2.6e-3 - so this is a STARTING POINT to be measured, not a fact.
# Measure it: park at a known height over texture, move a known distance,
# and set the scale until the track agrees with the tape measure.
DEFAULT_SCALE = 1.26e-3

# Below this the PMW3901 is inside its 80 mm focus limit and the image is
# not usable. Refuse to integrate rather than integrate rubbish.
MIN_HEIGHT_M = 0.08
TOF_NO_TARGET = 65535

STALE_S = 0.4              # no D line for this long = the link is stale


def now():
    return time.monotonic()


class State:
    """Everything the page shows. One lock, one writer (the serial thread)."""

    def __init__(self, cfg):
        self.lock = threading.Lock()
        self.cfg = cfg
        self.port = None
        self.connected = False
        self.last_line_at = 0.0
        self.lines = 0
        self.rate = 0.0
        self._rate_mark = (now(), 0)

        self.d = {}            # newest decoded D line
        self.loop = {}         # newest P line
        self.console = deque(maxlen=60)

        # flow integration
        self.prev = None       # (t_us, sum_dx, sum_dy, valid, lost)
        self.nav = {"x": 0.0, "y": 0.0, "vx": 0.0, "vy": 0.0, "dist": 0.0,
                    "height_m": None, "height_src": "none",
                    "integrating": False, "why": "no data yet",
                    "rate_x": 0.0, "rate_y": 0.0}
        self.track = [(0.0, 0.0)]
        self.track_base = 0    # index of track[0] in the absolute series
        self.track_gen = 1     # bumped on reset so the page drops its copy

    # ---- writer side -------------------------------------------------

    def note_line(self):
        self.lines += 1
        self.last_line_at = now()
        t, n = self._rate_mark
        if self.last_line_at - t >= 1.0:
            self.rate = (self.lines - n) / (self.last_line_at - t)
            self._rate_mark = (self.last_line_at, self.lines)

    def reset_track(self):
        with self.lock:
            self.nav["x"] = self.nav["y"] = self.nav["dist"] = 0.0
            self.track = [(0.0, 0.0)]
            self.track_base = 0
            self.track_gen += 1     # the page drops its copy when this moves
            self.prev = None

    def set_cfg(self, updates):
        with self.lock:
            for k, v in updates.items():
                if k in self.cfg:
                    self.cfg[k] = v


def decode(line, st):
    """Parse one line from the FC. Returns nothing; updates st."""
    f = line.split(",")
    tag = f[0]

    if tag == "D" and len(f) >= D_MIN_FIELDS:
        try:
            d = {
                "t_us": int(f[I_T_US]),
                "acc": [float(f[I_ACC + i]) for i in range(3)],
                "gyro": [float(f[I_GYRO + i]) for i in range(3)],
                "mag": [float(f[I_MAG + i]) for i in range(3)],
                "quat": [float(f[I_QUAT + i]) for i in range(4)],
                "rpy": [float(f[I_ROLL + i]) for i in range(3)],
                "fusion_dt": float(f[I_FUSION_DT]),
                "loop_dt": float(f[I_FUSION_DT + 1]),
                "motors": [int(f[I_MOTOR + i]) for i in range(4)],
                "armed": int(f[I_ARMED]),
                "heartbeat": int(f[I_HEARTBEAT]),
                "alt_baro": float(f[I_ALT]),
                "lat": float(f[I_LAT]), "lon": float(f[I_LON]),
                "sats": int(f[I_SATS]), "fix": int(f[I_FIX]),
                "health": int(f[I_HEALTH]),
            }
        except (ValueError, IndexError):
            return
        d["health_names"] = [n for b, n in HEALTH_BITS if d["health"] & b]
        d["health_missing"] = [n for b, n in HEALTH_BITS if not d["health"] & b]

        if len(f) >= D_FLOW_FIELDS:
            try:
                d["flow"] = {
                    "seq": int(f[I_FSEQ]),
                    "sum_dx": int(f[I_FSUMX]), "sum_dy": int(f[I_FSUMY]),
                    "valid": int(f[I_FVALID]), "lost": int(f[I_FLOST]),
                    "squal": int(f[I_FSQUAL]), "shutter": int(f[I_FSHUT]),
                    "flags": int(f[I_FFLAGS]),
                }
                d["tof_mm"] = int(f[I_TOF])
            except (ValueError, IndexError):
                d["flow"] = None
                d["tof_mm"] = 0
            if d["flow"]:
                d["flow"]["flag_names"] = [n for b, n in FLOW_FLAGS
                                           if d["flow"]["flags"] & b]
        else:
            d["flow"] = None
            d["tof_mm"] = None      # None = this firmware predates the field

        with st.lock:
            st.d = d
            integrate(st, d)
        st.note_line()
        return

    if tag == "P" and len(f) >= 15:
        try:
            with st.lock:
                st.loop = {"loop_max_us": int(f[5]), "loop_mean_us": int(f[4]),
                           "heap": int(f[14])}
        except (ValueError, IndexError):
            pass
        return

    # Anything else is a command reply or a log line - keep it visible.
    if line.strip():
        with st.lock:
            st.console.append({"t": time.strftime("%H:%M:%S"),
                               "line": line.rstrip()})


def integrate(st, d):
    """Flow counts -> velocity -> track. Caller holds the lock.

    Differences the ABSOLUTE sums, so a dropped D line costs accuracy in
    one window and nothing after it. Refuses to integrate rather than
    guess whenever the inputs do not support it, and says why."""
    nav, cfg = st.nav, st.cfg
    fl = d.get("flow")
    if not fl:
        nav.update(integrating=False, why="firmware has no flow fields")
        return

    cur = (d["t_us"], fl["sum_dx"], fl["sum_dy"], fl["valid"], fl["lost"])
    prev, st.prev = st.prev, cur
    if prev is None:
        nav.update(integrating=False, why="first line")
        return

    dt = (cur[0] - prev[0]) / 1e6
    d_valid = cur[3] - prev[3]
    if dt <= 0 or dt > 0.5:
        nav.update(integrating=False, why="window too long - link stalled")
        return
    if d_valid <= 0:
        # No usable window closed in this interval. Zero counts would be a
        # claim that the aircraft held still; silence is not a measurement.
        nav.update(integrating=False, vx=0.0, vy=0.0,
                   why="no valid flow window (sensor absent, or rejected)")
        return

    # height
    tof = d.get("tof_mm")
    h = None
    src = "none"
    if cfg["use_tof"] and tof not in (None, 0, TOF_NO_TARGET):
        h = tof / 1000.0
        src = "ToF"
    elif cfg["manual_height_m"] > 0:
        h = cfg["manual_height_m"]
        src = "manual"
    if h is None:
        nav.update(integrating=False, height_m=None, height_src="none",
                   why="no height: ToF has no target and no manual height set")
        return
    if h < MIN_HEIGHT_M:
        nav.update(integrating=False, height_m=h, height_src=src,
                   why=f"height {h:.3f} m is inside the sensor's 80 mm focus limit")
        return

    # counts -> image rate (rad/s), in the SENSOR frame
    rate_x = (cur[1] - prev[1]) * cfg["scale"] / dt
    rate_y = (cur[2] - prev[2]) * cfg["scale"] / dt

    # The sensor sees translation AND rotation. Rotation about the body Y
    # axis (pitch) moves the image along X and vice versa, so the gyro
    # pairing is crossed. Signs are unknown until the bench says so, hence
    # the toggles.
    gx, gy = math.radians(d["gyro"][0]), math.radians(d["gyro"][1])
    if cfg["gyro_comp"]:
        rate_x -= cfg["gyro_sign"] * gy
        rate_y -= cfg["gyro_sign"] * gx

    vx_s, vy_s = h * rate_x, h * rate_y
    if cfg["swap_xy"]:
        vx_s, vy_s = vy_s, vx_s
    if cfg["invert_x"]:
        vx_s = -vx_s
    if cfg["invert_y"]:
        vy_s = -vy_s

    # body -> world using yaw, so walking a square draws a square whatever
    # way the nose points.
    if cfg["rotate_by_yaw"]:
        psi = math.radians(d["rpy"][2] + cfg["yaw_offset_deg"])
        c, s = math.cos(psi), math.sin(psi)
        vx, vy = vx_s * c - vy_s * s, vx_s * s + vy_s * c
    else:
        vx, vy = vx_s, vy_s

    nav["vx"], nav["vy"] = vx, vy
    nav["x"] += vx * dt
    nav["y"] += vy * dt
    nav["dist"] += math.hypot(vx, vy) * dt
    nav["rate_x"], nav["rate_y"] = rate_x, rate_y
    nav.update(height_m=h, height_src=src, integrating=True, why="")

    last = st.track[-1]
    if math.hypot(nav["x"] - last[0], nav["y"] - last[1]) > 0.002:
        st.track.append((nav["x"], nav["y"]))
        if len(st.track) > 20000:            # keep memory bounded
            drop = len(st.track) - 20000
            st.track = st.track[drop:]
            st.track_base += drop


def serial_thread(st, port_hint, record_path):
    rec = open(record_path, "a", buffering=1) if record_path else None
    buf = b""
    while True:
        port = port_hint or autodetect()
        if not port:
            with st.lock:
                st.connected = False
                st.port = None
            time.sleep(1.0)
            continue
        try:
            with serial.Serial(port, 115200, timeout=0.2) as ser:
                with st.lock:
                    st.connected, st.port = True, port
                print(f"viewer: reading {port}")
                # STREAM,1 in case a previous session turned it off.
                ser.write(b"STREAM,1\n")
                while True:
                    chunk = ser.read(4096) or b""
                    if chunk:
                        buf += chunk
                        while b"\n" in buf:
                            raw, buf = buf.split(b"\n", 1)
                            line = raw.decode("utf-8", "replace").strip("\r")
                            if rec:
                                rec.write(line + "\n")
                            decode(line, st)
                    with st.lock:
                        pending, st.pending = getattr(st, "pending", []), []
                    for cmd in pending:
                        ser.write((cmd + "\n").encode())
        except (serial.SerialException, OSError) as e:
            # The S3's USB CDC disappears on reset and can come back
            # elsewhere - reconnecting is normal, not an error.
            with st.lock:
                st.connected = False
            print(f"viewer: {port} gone ({e}) - retrying")
            time.sleep(1.0)


def autodetect():
    cands = [p.device for p in list_ports.comports()
             if "USB" in (p.description or "").upper()
             or "JTAG" in (p.description or "").upper()
             or "CDC" in (p.description or "").upper()]
    return cands[0] if cands else None


def snapshot(st, since):
    with st.lock:
        age = now() - st.last_line_at if st.last_line_at else None
        d = dict(st.d)
        nav = dict(st.nav)
        first = max(since - st.track_base, 0)
        pts = st.track[first:] if first < len(st.track) else []
        out = {
            "link": {"port": st.port, "connected": st.connected,
                     "age_ms": None if age is None else int(age * 1000),
                     "stale": age is None or age > STALE_S,
                     "rate": round(st.rate, 1), "lines": st.lines},
            "d": d, "loop": dict(st.loop), "nav": nav,
            "cfg": dict(st.cfg),
            "track": {"next": st.track_base + len(st.track),
                      "gen": st.track_gen,
                      "pts": [[round(x, 4), round(y, 4)] for x, y in pts]},
            "console": list(st.console)[-25:],
        }
    return out


class Handler(BaseHTTPRequestHandler):
    st = None

    def log_message(self, *a):
        pass                     # the page polls 30x a second; no spam

    def _send(self, code, body, ctype="application/json"):
        body = body if isinstance(body, bytes) else body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split("?")[0]
        if path in ("/", "/index.html"):
            try:
                with open(PAGE, "rb") as fh:
                    self._send(200, fh.read(), "text/html; charset=utf-8")
            except OSError:
                self._send(500, b"viewer.html missing next to viewer.py", "text/plain")
            return
        if path == "/state":
            since = 0
            if "?" in self.path:
                q = self.path.split("?", 1)[1]
                for part in q.split("&"):
                    if part.startswith("since="):
                        try:
                            since = int(part[6:])
                        except ValueError:
                            since = 0
            self._send(200, json.dumps(snapshot(self.st, since)))
            return
        self._send(404, b"no", "text/plain")

    def do_POST(self):
        n = int(self.headers.get("Content-Length", 0))
        try:
            body = json.loads(self.rfile.read(n) or b"{}")
        except ValueError:
            self._send(400, b'{"error":"bad json"}')
            return
        if self.path == "/cmd":
            line = str(body.get("line", "")).strip()
            if line:
                with self.st.lock:
                    self.st.pending = getattr(self.st, "pending", []) + [line]
                    self.st.console.append({"t": time.strftime("%H:%M:%S"),
                                            "line": "> " + line})
            self._send(200, b'{"ok":true}')
            return
        if self.path == "/set":
            if body.pop("reset_track", False):
                self.st.reset_track()
            self.st.set_cfg(body)
            self._send(200, b'{"ok":true}')
            return
        self._send(404, b'{"error":"no"}')


def main():
    if serial is None:
        sys.exit("pyserial missing:  python -m pip install pyserial")
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial port (default: autodetect)")
    ap.add_argument("--http", type=int, default=8765)
    ap.add_argument("--scale", type=float, default=DEFAULT_SCALE,
                    help="rad per flow count (MEASURE IT; default %(default).2e)")
    ap.add_argument("--height", type=float, default=0.0,
                    help="manual height in m, used when the ToF has no target")
    ap.add_argument("--record", help="also append every line to this file")
    ap.add_argument("--no-open", action="store_true")
    a = ap.parse_args()

    cfg = {"scale": a.scale, "manual_height_m": a.height, "use_tof": True,
           "gyro_comp": True, "gyro_sign": 1.0, "swap_xy": False,
           "invert_x": False, "invert_y": False, "rotate_by_yaw": True,
           "yaw_offset_deg": 0.0, "invert_pitch": False}
    st = State(cfg)
    st.pending = []

    threading.Thread(target=serial_thread, args=(st, a.port, a.record),
                     daemon=True).start()

    Handler.st = st
    srv = ThreadingHTTPServer(("127.0.0.1", a.http), Handler)
    url = f"http://127.0.0.1:{a.http}/"
    print(f"viewer: {url}   (pack must be IN - on USB alone every sensor reads 0)")
    if not a.no_open:
        threading.Timer(0.6, lambda: webbrowser.open(url)).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nviewer: stopped")


if __name__ == "__main__":
    main()
