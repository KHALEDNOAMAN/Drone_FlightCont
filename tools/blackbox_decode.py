#!/usr/bin/env python3
"""Decode a blackbox dump (tools/blackbox_pull.py output) into per-
session CSVs plus a .params.txt with the gains that flew each session.

    python tools/blackbox_decode.py flight.bb
      -> flight_s3.csv, flight_s3.params.txt, ...

On-flash format (must match components/blackbox/blackbox.c):
  sector 4096 B = header {u32 magic 'BBX1', u32 seq, u16 session,
  u16 flags} + TLV records {u8 type, u8 len, payload}:
    1 = params snapshot (u32 version + params_t floats)
    2 = frame (bb_frame_t below)
    3 = session end
  0xFF padding to the sector end. Sector seq is monotonic; sectors are
  decoded in seq order so the ring wrap does not matter.
"""
import base64
import os
import shutil
import struct
import sys

MAGIC = 0x31584242
HDR = struct.Struct("<IIHH")
# bb_frame_t, all three generations. The TLV len byte is the
# discriminator, so logs pulled across every firmware revision decode:
# the 34 B frame ended in a spare pad byte, the 35 B one spends it plus
# one more on pack voltage, the 43 B one appends the vertical channel,
# and the 48 B one the alt-hold channel. Fields a log predates come out
# 0 - the same "no reading" convention the firmware uses, and a column
# that is 0 for an entire session is unmistakably "this firmware did not
# record it".
FRAME_34 = struct.Struct("<I3h3h3h4HHBB")       # early, _pad
FRAME_35 = struct.Struct("<I3h3h3h4HHBH")       # + vbat_mv
FRAME_43 = struct.Struct("<I3h3h3h4HHBH3hH")    # + alt/vz/accz/tof
FRAME_48 = struct.Struct("<I3h3h3h4HHBH3hH2hB")  # + alt_est/alt_sp/flags
FRAME_50 = struct.Struct("<I3h3h3h4HHBH3hH2hBh")  # + baro2_cm (MS5611)
FRAME_54 = struct.Struct("<I3h3h3h4HHBH3hH2hBhf")  # + accz_dv_ms (float)
FRAME_66 = struct.Struct("<I3h3h3h4HHBH3hH2hBhf4hHBB")  # + GPS channel
FRAME_74 = struct.Struct("<I3h3h3h4HHBH3hH2hBhf4hHBB2f")  # + horiz delta-v
FRAME_75 = struct.Struct("<I3h3h3h4HHBH3hH2hBhf4hHBB2fB")  # + ctrl_flags
FRAMES = {FRAME_34.size: FRAME_34, FRAME_35.size: FRAME_35,
          FRAME_43.size: FRAME_43, FRAME_48.size: FRAME_48,
          FRAME_50.size: FRAME_50, FRAME_54.size: FRAME_54,
          FRAME_66.size: FRAME_66, FRAME_74.size: FRAME_74,
          FRAME_75.size: FRAME_75}
# Field names are per params VERSION, not global.
#
# They used to be one flat list mapped onto a snapshot by COUNT, which
# assumed params_t only ever grew at the end. On six fields
# were inserted mid-struct, and from then on the same count meant two
# different layouts. Every params.txt written since read the wrong names
# from alt_acc_sigma down, and the firmware made the identical mistake
# against NVS - that one enabled both holds on a flight nobody chose.
#
# The snapshot carries `version`. Use it.
_NAMES_COMMON = [
    "rate_kp", "rate_ki", "rate_kd",
    "yaw_kp", "yaw_ki", "yaw_kd",
    "angle_kp", "angle_max_deg", "yaw_rate_max",
    "rc_deadband", "min_idle_us",
    "gyro_lpf_hz", "notch_hz", "notch_q",
    # appended in params v2
    "m1_scale", "m2_scale", "m3_scale", "m4_scale",
    # appended in params v3
    "level_roll_deg", "level_pitch_deg",
    # appended in params v4 (P7 alt-hold)
    "alt_hover_thr", "alt_vref", "alt_vz_max", "alt_kp_z", "alt_kp_vz",
    "alt_ki_vz", "alt_i_max", "alt_thr_db",
]
_TOF_TAIL = ["tof_sig_lim", "tof_pre_pclk", "tof_fin_pclk", "tof_budget_ms"]

# v6 and earlier, as SHIPPED: 33 fields, alt_acc_sigma at slot 28.
PARAM_NAMES_V6 = _NAMES_COMMON + ["alt_acc_sigma"] + _TOF_TAIL

# The build. Stamped v6 like its predecessor while having a
# DIFFERENT layout, which is the whole defect - it is reconstructed here
# by float count rather than by version because the version cannot tell
# them apart. Kept because real flights were logged against it (s176-s182)
# and they still have to be readable.
PARAM_NAMES_V6_INSERTED = (
    _NAMES_COMMON + ["hold_auto", "vel_kp", "vel_ki", "vel_i_max",
                     "vel_tilt_max", "vel_max", "alt_acc_sigma"] + _TOF_TAIL)

# v7: hold_auto renamed alt_hold_auto and split, vel_hold_auto appended
# at the END, and the blob now carries a digest of these names so this
# whole class of confusion is refused rather than guessed at.
# thr_vcomp appended (battery-sag throttle compensation).
# Snapshots written before it simply carry one float fewer, which is what
# the prefix rule guarantees and what zip() below already handles.
PARAM_NAMES_V7 = (
    _NAMES_COMMON + ["alt_hold_auto", "vel_kp", "vel_ki", "vel_i_max",
                     "vel_tilt_max", "vel_max", "alt_acc_sigma"]
    + _TOF_TAIL + ["vel_hold_auto", "thr_vcomp"])


def param_names(version, n_vals):
    """Which name list a snapshot's floats belong to.

    v6 is ambiguous by version alone - two builds shipped under it - so
    the float count disambiguates, and only there."""
    if version >= 7:
        return PARAM_NAMES_V7, ""
    if n_vals > len(PARAM_NAMES_V6):
        return (PARAM_NAMES_V6_INSERTED,
                "# NOTE: v%d with %d values = the build, which "
                "inserted\n#       six fields mid-struct without a version "
                "bump. Names below are\n#       reconstructed from the "
                "float count, not from the version.\n"
                % (version, n_vals))
    return PARAM_NAMES_V6, ""

CSV_HEADER = ("t_us,gyro_roll,gyro_pitch,gyro_yaw,"
              "setp_roll,setp_pitch,setp_yaw,"
              "roll,pitch,yaw,m1,m2,m3,m4,throttle,sensor_health,"
              "vbat_mv,"
              # Vertical channel, in SI: alt_m is metres above the arm
              # point, vz_ms and accz_ms2 positive = up, tof_m keeps the
              # firmware's sentinels as 0 (absent) and 65.535 (no target)
              # so a MATLAB plot shows them as obvious out-of-band values
              # rather than silently interpolating through them.
              "alt_m,vz_ms,accz_ms2,tof_m,"
              # Alt-hold channel (frame 48). alt_est_m and alt_sp_m share
              # the vertical FILTER's ground datum, which is NOT the same
              # zero as alt_m - that one is raw baro relative to the arm
              # edge. Plotting them on one axis without knowing that is a
              # guaranteed wrong conclusion.
              "alt_est_m,alt_sp_m,vert_flags,"
              # Second barometer (frame 50), metres above the arm edge -
              # the SAME datum as alt_m, so the two plot on one axis. This
              # is the DPS310 control experiment: check bit 4 of
              # sensor_health for which samples are real. 0 for a whole
              # session means no MS5611 was fitted.
              "baro2_m,"
              # Velocity the vertical filter has integrated since boot,
              # m/s. An ABSOLUTE running sum, so what you actually want
              # is always a DIFFERENCE between two rows - which stays
              # exact across dropped or transport-lost frames, where an
              # on-board delta would not.
              #
              # accz_ms2 beside it is an instantaneous 250 Hz sample of
              # the same signal the filter consumes at 416 Hz, and hover
              # vibration is sd 3.67 m/s^2 - so that column cannot
              # reconstruct the integral, which is the whole reason this
              # one exists. See components/blackbox/blackbox.h.
              # 0 for a whole session = firmware older than.
              "accz_dv_ms,"
              # GPS channel (frame 66). pos_n_m/pos_e_m are metres NORTH
              # and EAST of the home point latched at arm, and are only
              # meaningful when gps_flags bit 4 (HOME_VALID) is set -
              # without a home there is no origin and both read 0.
              #
              # This is what the Stage A exit test plots: E against N must
              # draw the path that was walked, vel_n/vel_e must sit near
              # 1.4 m/s at walking pace, and h_acc_m says whether to
              # believe any of it. gps_flags: bits 0-2 fixType (3 = 3D),
              # bit 3 gnssFixOK, bit 4 home_valid.
              # 0 for a whole session = firmware older than.
              "pos_n_m,pos_e_m,vel_n_ms,vel_e_ms,h_acc_m,sats,gps_flags,"
              # Earth-frame horizontal delta-v, ABSOLUTE running sums in
              # m/s - DIFFERENCE two rows to get the interval's velocity
              # change. Identical argument to accz_dv_ms above, and the
              # reason a 6-state horizontal filter can be prototyped
              # offline at all: GPS alone tests corrections, never
              # predictions. 0 for a session = firmware older than this.
              "accn_dv_ms,acce_dv_ms,"
              # Rate I-term ground hold (frame 75, components/common/
              # iterm_gate.h): bit 0 HELD, 1 TOF_REF (0 = throttle-latch
              # fallback), 2 LIFTED, 3 FRESH, 4 THR. 0 for a whole session
              # = firmware older than.
              "ctrl_flags\n")


def load_sectors(path):
    """BB,<seq>,<off>,<b64> lines -> ({seq: bytearray(4096)},
    {seq: coverage bytearray, 1 = byte received}), seq-sorted.

    Unreceived bytes stay 0xFF (the TLV padding value) and are flagged
    uncovered, so the record parser can stop at a transport hole
    instead of decoding ghost frames from filler. The old zero-fill
    produced fake frames (thr=0, motors=0) that cost a debug session."""
    sectors = {}
    coverage = {}
    with open(path) as fh:
        for line in fh:
            if not line.startswith("BB,"):
                continue
            try:
                _, seq, off, b64 = line.strip().split(",", 3)
                seq, off = int(seq), int(off)
                chunk = base64.b64decode(b64)
            except (ValueError, base64.binascii.Error):
                continue
            sec = sectors.setdefault(seq, bytearray(b"\xff" * 4096))
            cov = coverage.setdefault(seq, bytearray(4096))
            sec[off:off + len(chunk)] = chunk
            cov[off:off + len(chunk)] = b"\x01" * len(chunk)
    return dict(sorted(sectors.items())), coverage


def main():
    if len(sys.argv) != 2:
        sys.exit("usage: blackbox_decode.py <dump.bb>")
    path = sys.argv[1]
    stem = path.rsplit(".", 1)[0]

    sectors, coverage = load_sectors(path)
    if not sectors:
        sys.exit("no BB, lines found - is this a blackbox_pull.py file?")
    seqs = list(sectors)
    n_holes = seqs[-1] - seqs[0] + 1 - len(seqs)
    print(f"{len(sectors)} sectors" +
          (f" ({n_holes} lost in transport)" if n_holes else ""))

    out = None
    cur_session = None
    frames = 0
    sessions = []
    n_trunc = 0                 # sectors cut short at a transport hole
    n_corrupt = 0               # frames rejected by the sanity gate

    def close():
        nonlocal out, frames
        if out:
            out.close()
            print(f"  session {cur_session}: {frames} frames "
                  f"({frames / 250.0:.1f} s) -> {sessions[-1]}")
        out = None
        frames = 0

    for seq, sec in sectors.items():
        magic, _, session, flags = HDR.unpack_from(sec, 0)
        if magic != MAGIC:
            continue
        # A gap in seq means the ring overwrote the middle of an old
        # session; a session id change means a new arm. Both close the
        # current file.
        if session != cur_session:
            close()
            cur_session = session
        cov = coverage[seq]
        i = HDR.size
        while i + 2 <= len(sec):
            rtype, rlen = sec[i], sec[i + 1]
            if rtype == 0xFF:
                break                       # erased tail padding
            if not all(cov[i:i + 2 + rlen]):
                n_trunc += 1                # transport hole: bytes after
                break                       # it can't be trusted
            payload = sec[i + 2:i + 2 + rlen]
            i += 2 + rlen
            if rtype == 1:                  # params snapshot = session start
                close()
                cur_session = session
                name = f"{stem}_s{session}.csv"
                sessions.append(name)
                out = open(name, "w")
                out.write(CSV_HEADER)
                (version,) = struct.unpack_from("<I", payload, 0)
                n_vals = (len(payload) - 4) // 4     # append-only ABI
                vals = struct.unpack_from(f"<{n_vals}f", payload, 4)
                with open(f"{stem}_s{session}.params.txt", "w") as pf:
                    pf.write(f"# params snapshot v{version}, "
                             f"session {session}\n")
                    names, note = param_names(version, n_vals)
                    if note:
                        pf.write(note)
                    # Fewer values than names is normal - an older log
                    # mapping onto a prefix. MORE is not: params_t grew
                    # and this file did not, so every name past the end
                    # is a guess. Name them UNKNOWN_n and say so, rather
                    # than letting zip() drop the tail in silence, which
                    # is what hid the whole shift.
                    if n_vals > len(names):
                        pf.write(f"# WARNING: {n_vals} values but only "
                                 f"{len(names)} names known for v{version}. "
                                 f"The tables in tools/blackbox_decode.py "
                                 f"are STALE - add the new params_t "
                                 f"fields.\n")
                        print(f"  WARNING session {session}: params "
                              f"snapshot has {n_vals} values, v{version} "
                              f"table knows {len(names)} - table is stale")
                    for i_n, v in enumerate(vals):
                        n = (names[i_n] if i_n < len(names)
                             else f"UNKNOWN_{i_n}")
                        pf.write(f"{n} = {v:.6g}\n")
            elif rtype == 2 and out:
                fmt = FRAMES.get(len(payload))
                if fmt is None:
                    n_corrupt += 1      # not a frame length we know
                    continue
                vals = fmt.unpack(payload)
                (t_us, gx, gy, gz, sr, sp, sy, r, p, y,
                 m1, m2, m3, m4, thr, health, vbat) = vals[:17]
                if fmt is FRAME_34:
                    vbat = 0            # trailing byte was padding
                # Vertical channel, then the alt-hold channel. Each
                # generation contributes what it has and older logs get
                # zeros - the firmware's own "no reading" convention, and
                # a column that is 0 for a whole session reads clearly as
                # "this firmware did not record it".
                alt_cm = vz_cms = accz_cms2 = tof_mm = 0
                alt_est_cm = alt_sp_cm = vert_flags = 0
                baro2_cm = 0
                accz_dv_ms = 0.0
                pos_n_dm = pos_e_dm = vel_n_cms = vel_e_cms = 0
                gps_h_acc_mm = sats = gps_flags = 0
                accn_dv_ms = acce_dv_ms = 0.0
                ctrl_flags = 0
                if fmt is FRAME_43:
                    (alt_cm, vz_cms, accz_cms2, tof_mm) = vals[17:]
                elif fmt is FRAME_48:
                    (alt_cm, vz_cms, accz_cms2, tof_mm,
                     alt_est_cm, alt_sp_cm, vert_flags) = vals[17:]
                elif fmt is FRAME_50:
                    (alt_cm, vz_cms, accz_cms2, tof_mm,
                     alt_est_cm, alt_sp_cm, vert_flags, baro2_cm) = vals[17:]
                elif fmt is FRAME_54:
                    (alt_cm, vz_cms, accz_cms2, tof_mm,
                     alt_est_cm, alt_sp_cm, vert_flags, baro2_cm,
                     accz_dv_ms) = vals[17:]
                elif fmt is FRAME_66:
                    (alt_cm, vz_cms, accz_cms2, tof_mm,
                     alt_est_cm, alt_sp_cm, vert_flags, baro2_cm,
                     accz_dv_ms, pos_n_dm, pos_e_dm, vel_n_cms, vel_e_cms,
                     gps_h_acc_mm, sats, gps_flags) = vals[17:]
                elif fmt is FRAME_74:
                    (alt_cm, vz_cms, accz_cms2, tof_mm,
                     alt_est_cm, alt_sp_cm, vert_flags, baro2_cm,
                     accz_dv_ms, pos_n_dm, pos_e_dm, vel_n_cms, vel_e_cms,
                     gps_h_acc_mm, sats, gps_flags,
                     accn_dv_ms, acce_dv_ms) = vals[17:]
                elif fmt is FRAME_75:
                    (alt_cm, vz_cms, accz_cms2, tof_mm,
                     alt_est_cm, alt_sp_cm, vert_flags, baro2_cm,
                     accz_dv_ms, pos_n_dm, pos_e_dm, vel_n_cms, vel_e_cms,
                     gps_h_acc_mm, sats, gps_flags,
                     accn_dv_ms, acce_dv_ms, ctrl_flags) = vals[17:]
                # Transport corruption can arrive as valid-length base64
                # with garbled content. Motors are mixer-clamped to
                # <=2000 and raw throttle is <=2000 in any armed frame,
                # so anything above is a mangled record, not data.
                if max(m1, m2, m3, m4, thr) > 2100:
                    n_corrupt += 1
                    continue
                out.write(f"{t_us},{gx/16.0:.3f},{gy/16.0:.3f},{gz/16.0:.3f},"
                          f"{sr/16.0:.3f},{sp/16.0:.3f},{sy/16.0:.3f},"
                          f"{r/100.0:.2f},{p/100.0:.2f},{y/100.0:.2f},"
                          f"{m1},{m2},{m3},{m4},{thr},{health},{vbat},"
                          f"{alt_cm/100.0:.2f},{vz_cms/100.0:.2f},"
                          f"{accz_cms2/100.0:.2f},{tof_mm/1000.0:.3f},"
                          f"{alt_est_cm/100.0:.2f},{alt_sp_cm/100.0:.2f},"
                          f"{vert_flags},{baro2_cm/100.0:.2f},"
                          # 6 dp: this column is differenced, and one
                          # frame's step is of order 1e-3 m/s.
                          f"{accz_dv_ms:.6f},"
                          f"{pos_n_dm / 10.0:.1f},{pos_e_dm / 10.0:.1f},"
                          f"{vel_n_cms / 100.0:.2f},{vel_e_cms / 100.0:.2f},"
                          f"{gps_h_acc_mm / 1000.0:.3f},{sats},{gps_flags},"
                          # 6 dp for the same reason accz_dv_ms uses it:
                          # these columns are differenced, and one frame's
                          # step is of order 1e-3 m/s.
                          f"{accn_dv_ms:.6f},{acce_dv_ms:.6f},"
                          f"{ctrl_flags}\n")
                frames += 1
            elif rtype == 3:
                close()
    close()

    if n_holes or n_trunc or n_corrupt:
        per_sector = (4096 - HDR.size) // (FRAME_75.size + 2)
        print(f"transport damage: {n_holes} sector(s) lost, {n_trunc} "
              f"truncated at a hole (~{n_holes*per_sector + n_trunc*(per_sector//2)} frames "
              f"missing), {n_corrupt} corrupt frame(s) dropped - re-pull "
              f"with the verifying blackbox_pull.py")
    if not sessions:
        print("no sessions found (partition may only hold pre-P4 data - "
              "BBERASE and fly again)")
    else:
        print(f"{len(sessions)} session file(s). The gyro/setpoint columns "
              "feed fft_analyze-style spectra; params.txt says what gains "
              "flew.")
        # Sectors are decoded in seq order, which is monotonic, and a new
        # arm always gets a higher session id - so the last session closed
        # is always the most recent arm on the device, with no wall-clock
        # needed (the S3 has no RTC). Copied under a fixed name so "which
        # file is the last flight" never has to be re-derived by hand.
        latest_csv = sessions[-1]
        latest_params = latest_csv.rsplit(".", 1)[0] + ".params.txt"
        latest_csv_copy = f"{stem}_LATEST.csv"
        latest_params_copy = f"{stem}_LATEST.params.txt"
        shutil.copyfile(latest_csv, latest_csv_copy)
        if os.path.exists(latest_params):
            shutil.copyfile(latest_params, latest_params_copy)
        print(f"latest session: {latest_csv} <- LATEST, also copied to "
              f"{latest_csv_copy} / {latest_params_copy}")


if __name__ == "__main__":
    main()
