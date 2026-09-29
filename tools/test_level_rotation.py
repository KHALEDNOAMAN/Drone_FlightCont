"""Verify the LEVELCAL rotation matrix as it is literally written in
fusion.c. No host C runtime is installed, so instead of transcribing the
maths by hand - which is precisely where a sign error would hide - the
nine coefficient expressions are PARSED OUT of the source and evaluated.
A transpose or a flipped sign in the shipping text fails this.

Claim under test: a board that is physically level but reads (r0, p0)
must read exactly level after the correction, and a real tilt on top of
the offset must survive it.
"""
import math
import os
import re
import sys

SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir,
                   "components", "fusion", "fusion.c")

src = open(SRC, encoding="utf-8-sig").read()

# --- pull the 9 matrix element expressions out of level_rot_update -----
body = src.split("static void level_rot_update")[1].split("\n}")[0]
elems = dict()
for idx, expr in re.findall(r"L->m\[(\d)\]\s*=\s*([^;]+);", body):
    elems[int(idx)] = expr.strip()
assert len(elems) == 9, f"expected 9 matrix elements, parsed {len(elems)}"
print("parsed from fusion.c:")
for i in range(9):
    print(f"  m[{i}] = {elems[i]}")


def build_R(r_deg, p_deg):
    r, p = math.radians(r_deg), math.radians(p_deg)
    env = {"sr": math.sin(r), "cr": math.cos(r),
           "sp": math.sin(p), "cp": math.cos(p)}
    vals = [eval(elems[i].replace("f", ""), {"__builtins__": {}}, env)
            for i in range(9)]
    return [vals[0:3], vals[3:6], vals[6:9]]


def apply_R(R, v):
    return [sum(R[i][j] * v[j] for j in range(3)) for i in range(3)]


def euler_from_accel(x, y, z):
    """Same two formulas as seed_attitude_from_sensors (fusion.c:564) and
    run_level_cal - verified identical in both places by inspection."""
    return (math.degrees(math.atan2(y, z)),
            math.degrees(math.atan2(-x, math.hypot(y, z))))


def accel_when_tilted(r_deg, p_deg):
    """What a stationary board reads at attitude (r,p): earth-up in body
    coordinates = the third row of R."""
    r, p = math.radians(r_deg), math.radians(p_deg)
    return [-math.sin(p), math.cos(p) * math.sin(r), math.cos(p) * math.cos(r)]


fails = 0


def check(what, got, want, tol):
    global fails
    ok = abs(got - want) <= tol
    if not ok:
        fails += 1
    print(f"  {what:<44} {got:+10.6f} (want {want:+.6f}) "
          f"{'ok' if ok else '<<<< FAIL'}")


for r0, p0 in [(-1.83, 2.04), (0.0, -2.0), (7.5, 7.5), (-10.0, 10.0),
               (3.0, 0.0), (0.0, 0.0)]:
    print(f"\noffset roll {r0:+.2f} pitch {p0:+.2f} deg")
    R = build_R(r0, p0)

    # 1. level board, tilted sensor -> must correct to dead level
    a = accel_when_tilted(r0, p0)
    mr, mp = euler_from_accel(*a)
    check("uncorrected reads roll", mr, r0, 1e-9)
    check("uncorrected reads pitch", mp, p0, 1e-9)
    ca = apply_R(R, a)
    mr, mp = euler_from_accel(*ca)
    check("CORRECTED roll", mr, 0.0, 1e-9)
    check("CORRECTED pitch", mp, 0.0, 1e-9)
    check("norm preserved", math.sqrt(sum(c * c for c in ca)), 1.0, 1e-12)

    # 2. A REAL manoeuvre on top of the mount error.
    #
    #    Composed properly, NOT as Euler addition - Euler angles do not
    #    add under composition once pitch is non-zero, and pretending
    #    they do is what made an earlier version of this test "fail" by
    #    0.09 deg at a 7.5 deg mount offset.
    #
    #    Sensor frame S = airframe frame A turned by the fixed mount
    #    error M, so R_S = R_A * M and the sensor measures
    #        a_S = R_S^T e3 = M^T R_A^T e3.
    #    The correction multiplies by M, giving M M^T R_A^T e3 = R_A^T e3
    #    - the AIRFRAME's own gravity vector, exactly, at any attitude.
    for bank, pitch_cmd in [(10.0, 0.0), (0.0, 15.0), (-25.0, 8.0)]:
        RA = build_R(bank, pitch_cmd)
        RAt_e3 = [RA[2][0], RA[2][1], RA[2][2]]          # R_A^T e3
        aS = [sum(R[k][i] * RAt_e3[k] for k in range(3))  # M^T (...)
              for i in range(3)]
        mr, mp = euler_from_accel(*apply_R(R, aS))
        check(f"real ({bank:+.0f},{pitch_cmd:+.0f}) reads roll", mr, bank, 1e-9)
        check(f"real ({bank:+.0f},{pitch_cmd:+.0f}) reads pitch", mp,
              pitch_cmd, 1e-9)

    # 3. R must be a proper rotation: R*R^T = I, det = +1 (not a
    #    reflection, which would silently mirror an axis)
    err = max(abs(sum(R[i][k] * R[j][k] for k in range(3)) - (i == j))
              for i in range(3) for j in range(3))
    check("R orthonormal (max |RR^T - I|)", err, 0.0, 1e-12)
    det = (R[0][0] * (R[1][1] * R[2][2] - R[1][2] * R[2][1])
           - R[0][1] * (R[1][0] * R[2][2] - R[1][2] * R[2][0])
           + R[0][2] * (R[1][0] * R[2][1] - R[1][1] * R[2][0]))
    check("det(R) = +1", det, 1.0, 1e-12)

print(f"\n{'FAILED' if fails else 'ALL PASS'} ({fails} failures)")
sys.exit(1 if fails else 0)
