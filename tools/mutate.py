#!/usr/bin/env python3
"""Mutation driver for the freestanding host tests.

A passing test suite is not evidence. It becomes evidence only when you
have watched it FAIL for the right reason. This script breaks one line of
the source at a time, reruns the suite against the broken copy, and
insists the suite notices. A mutant that survives is a hole in the tests,
not a curiosity.

The mechanism is the _SRC environment variable each host test already
honours: the source directory is copied to a temp dir, one regex
substitution is applied, and the test is pointed at the copy. The real
tree is never touched.

Mutants are CURATED rather than generated. An auto-generated set is mostly
equivalent mutants - changes that cannot alter behaviour - and chasing
those teaches nothing. Every entry below is a specific wrong decision that
a person could plausibly have written instead.

Run:  python tools/mutate.py            (everything)
      python tools/mutate.py gnss_guard (one target)
"""

import os
import re
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

# target -> (source dir, env var, test script, [(file, pattern, repl, why)])
TARGETS = {
    "ubx": (
        os.path.join(ROOT, "components", "tbsM10"),
        "UBX_SRC",
        os.path.join(HERE, "ubx_test.py"),
        [
            ("ubx.c", r"if \(b\[0\] == 0x01\)", "if (b[0] == 0x02)",
             "SEC-SIG dispatches V1 on the V2 version byte"),
            ("ubx.c", r"out->jamming_state   = \(uint8_t\)\(\(b\[4\] >> 1\) & 0x03u\);",
             "out->jamming_state   = (uint8_t)(b[4] & 0x03u);",
             "V1 jammingState read without the shift"),
            ("ubx.c", r"out->spoofing_state  = \(uint8_t\)\(\(b\[8\] >> 1\) & 0x07u\);",
             "out->spoofing_state  = (uint8_t)((b[8] >> 1) & 0x03u);",
             "V1 spoofingState masked to 2 bits instead of 3"),
            ("ubx.c", r"out->spoofing_state  = \(uint8_t\)\(\(b\[1\] >> 4\) & 0x07u\);",
             "out->spoofing_state  = (uint8_t)((b[1] >> 3) & 0x07u);",
             "V2 spoofingState shifted one bit short"),
            ("ubx.c", r"\+ \(uint32_t\)n \* MON_RF_BLOCK_LEN > p->len",
             "+ (uint32_t)n * MON_RF_BLOCK_LEN > 65535u",
             "MON-RF block-count bound check defeated"),
            ("ubx.c", r"if \(n == 0\) return 0;", "if (n == 99) return 0;",
             "MON-RF zero-block guard removed"),
            ("ubx.c", r"if \(st > worst\) worst = st;", "if (st < worst) worst = st;",
             "MON-RF keeps the BEST jamming state, not the worst"),
            ("ubx.c", r"} else if \(r == SIG_REC_LAST\) \{",
             "} else if (r == SIG_OFF_FLAGS0) {",
             "NAV-SIG record committed before its flags are read"),
            ("ubx.c", r"#define SIG_FLAG_PRUSED 0x08u", "#define SIG_FLAG_PRUSED 0x04u",
             "prUsed read from the wrong sigFlags bit"),
            ("ubx.c", r"#define SIG_OFF_CNO     6u", "#define SIG_OFF_CNO     7u",
             "cno read from qualityInd's offset"),
            ("ubx.c", r"if \(p->streamed\) return 0;\n    if \(p->cls != UBX_CLASS_NAV",
             "if (p->cls != UBX_CLASS_NAV",
             "streamed guard removed from NAV-PVT decode"),
            ("ubx.c", r"            if \(p->cno_active\) \{\n                p->cno_stats = p->cno_acc;\n                p->cno_ready = 1;\n            \}\n            return 1;",
             "            return 1;",
             "C/N0 snapshot never published"),
            ("ubx.c", r"\*acked = \(uint8_t\)\(p->id == UBX_ID_ACK_ACK\);",
             "*acked = (uint8_t)(p->id == UBX_ID_ACK_NAK);",
             "ACK and NAK swapped - a rejected config reported as taken"),
            ("ubx.c", r"if \(p->len != 2\) return 0;",
             "if (p->len < 2) return 0;",
             "ACK of any length accepted"),
            ("ubx.c", r"if \(p->cls != UBX_CLASS_ACK\) return 0;",
             "if (0) return 0;",
             "any class with id 0/1 read as an ACK"),
            ("ubx.c", r"if \(p->id != UBX_ID_ACK_ACK && p->id != UBX_ID_ACK_NAK\) return 0;",
             "if (0) return 0;",
             "any ACK-class id read as ACK or NAK"),
            ("ubx.c", r"\*cls = p->buf\[0\];\n    \*id = p->buf\[1\];",
             "*cls = p->buf[1];\n    *id = p->buf[0];",
             "answered message's class and id swapped"),
            ("ubx.c", r"if \(p->streamed\) return 0;\n    if \(p->cls != UBX_CLASS_ACK\)",
             "if (p->cls != UBX_CLASS_ACK)",
             "streamed guard removed from ACK decode"),
        ],
    ),
    "gnss_guard": (
        os.path.join(ROOT, "components", "gnss_guard"),
        "GNSS_GUARD_SRC",
        os.path.join(HERE, "gnss_guard_test.py"),
        [
            ("gnss_guard.c", r"if \(n_own >= 2\) \{", "if (n_own >= 1) {",
             "one heuristic alone can DENY - the corroboration rule gone"),
            ("gnss_guard.c", r"if \(n_own >= 1\) \{", "if (n_own >= 3) {",
             "a single heuristic raises nothing at all"),
            ("gnss_guard.c", r"set_bit\(g, GG_R_AGC_SHIFT, agc_drop > \(int32_t\)g->cfg.agc_delta\);",
             "set_bit(g, GG_R_AGC_SHIFT, agc_drop < -(int32_t)g->cfg.agc_delta);",
             "AGC tested in the wrong direction - a quiet band trips it"),
            ("gnss_guard.c", r"n_used >= g->cfg.cno_min_used && lhs < rhs\);",
             "lhs < rhs);",
             "C/N0 spread test fires on three satellites"),
            ("gnss_guard.c", r"int64_t lhs = num \* 100;", "int64_t lhs = num;",
             "uniformity threshold off by 100 - never fires"),
            ("gnss_guard.c", r"\(now_ms - g->first_bad_ms\) < g->cfg.confirm_ms",
             "(now_ms - g->first_bad_ms) < 0",
             "confirmation delay removed - one frame can drop the fix"),
            ("gnss_guard.c", r"\(now_ms - g->last_clean_ms\) >= g->cfg.clear_ms",
             "(now_ms - g->last_clean_ms) >= 0",
             "latch releases the instant the sky blinks clean"),
            ("gnss_guard.c", r"if \(!have_inertial\) \{", "if (0) {",
             "velocity cross-check runs with no inertial reference"),
            ("gnss_guard.c", r"if \(g->st.pos == GG_CLEAN && g->live_reasons == 0\) \{",
             "if (1) {",
             "baseline learns the jammer as normal"),
            ("gnss_guard.c", r"\(\(now_ms - g->last_rf_ms\) > g->cfg.stale_ms\);",
             "((now_ms - g->last_rf_ms) < g->cfg.stale_ms);",
             "staleness inverted - a blind guard reports CLEAN"),
            ("gnss_guard.c", r"set_bit\(g, GG_R_MOD_SPOOF_MULT, g->last_sec_spoof == UBX_SPF_MULTIPLE\);",
             "set_bit(g, GG_R_MOD_SPOOF_MULT, 0);",
             "the module's strongest spoofing verdict ignored"),
            ("gnss_guard.c", r"if \(r & GG_R_MOD_JAM_CRIT\) \{ pos = worse\(pos, GG_DENIED\);",
             "if (r & GG_R_MOD_JAM_CRIT) { pos = worse(pos, GG_SUSPECT);",
             "critical jamming only inflates R instead of dropping GPS"),
            ("gnss_guard.c", r"for \(int i = 0; i < 4; i\+\+\) r = 0.5f \* \(r \+ x / r\);",
             "for (int i = 0; i < 1; i++) r = 0.5f * (r + x / r);",
             "sqrt left at its seed's accuracy"),
            ("gnss_guard.c",
             r"            g->sat_jump_ms = now_ms \? now_ms : 1;  /\* 0 means \"never\" \*/\n            set_bit\(g, GG_R_SAT_JUMP, 1\);\n        \}",
             "            set_bit(g, GG_R_SAT_JUMP, 1);\n        } else {\n            set_bit(g, GG_R_SAT_JUMP, 0);\n        }",
             "satellite jump back to set-and-clear - erased by the next solution"),
            ("gnss_guard.c",
             r"\(now_ms - g->sat_jump_ms\) >= g->cfg.sat_jump_hold_ms\) \{",
             "(now_ms - g->sat_jump_ms) >= 0xFFFFFFFFu) {",
             "satellite jump never released - stands forever"),
        ],
    ),
    # The rate I-term ground hold. Not included: removing seq_moved from the
    # freshness test - equivalent on the aircraft, where nothing can arm in
    # the first 500 ms after boot (fusion warm-up alone is ~2.6 s).
    "iterm_gate": (
        os.path.join(ROOT, "components", "common"),
        "ITERM_SRC",
        os.path.join(HERE, "iterm_gate_test.py"),
        [
            ("iterm_gate.c", r"if \(g->fresh && d > ITERM_GATE_NEAR_MM\)",
             "if (g->fresh && d >= ITERM_GATE_NEAR_MM)",
             "liftoff declared at +30 mm, inside the on-its-legs band"),
            ("iterm_gate.c", r"d >= -ITERM_GATE_NEAR_MM &&",
             "d > -ITERM_GATE_NEAR_MM &&",
             "-30 mm counted as off the ground"),
            ("iterm_gate.c", r"d <= ITERM_GATE_NEAR_MM;",
             "d < ITERM_GATE_NEAR_MM;",
             "+30 mm counted as off the ground"),
            ("iterm_gate.c",
             r"g->hold = g->lifted \? \(near && throttle_us < ITERM_GATE_THR_US\) : near;",
             "g->hold = g->lifted ? near : near;",
             "throttle dropped after liftoff - a low hover gets held"),
            ("iterm_gate.c",
             r"g->hold = g->lifted \? \(near && throttle_us < ITERM_GATE_THR_US\) : near;",
             "g->hold = near && throttle_us < ITERM_GATE_THR_US;",
             "plain throttle rule - spool-up and bench ramps wind up again"),
            ("iterm_gate.c", r"\(near && throttle_us < ITERM_GATE_THR_US\)",
             "(near && throttle_us <= ITERM_GATE_THR_US)",
             "1250 us after liftoff counted as not asking to fly"),
            ("iterm_gate.c",
             r"\(now_us - g->seq_us\) <= ITERM_GATE_FRESH_US;",
             "(now_us - g->seq_us) >= 0;",
             "freshness window removed - a frozen reading counts forever"),
            ("iterm_gate.c", r"\} else if \(tof_seq != g->seq\) \{",
             "} else if (tof_seq == g->seq) {",
             "sample counter read backwards"),
            ("iterm_gate.c", r"tof_mm != ITERM_GATE_TOF_NO_TARGET &&",
             "1 &&",
             "65535 (nothing in range) taken as a distance"),
            ("iterm_gate.c", r"g->fresh = tof_mm != ITERM_GATE_TOF_NONE &&",
             "g->fresh =",
             "0 (reads failing) taken as a distance"),
            ("iterm_gate.c", r"g->ref_ok = g->fresh;",
             "g->ref_ok = true;",
             "reference taken at arm without checking it is fresh"),
            ("iterm_gate.c",
             r"g->ref_mm = tof_mm;\n(\s+)g->lifted = false;",
             "g->ref_mm = tof_mm;\n\\1/* lifted carried over */",
             "LIFTED carried over from the previous flight"),
            ("iterm_gate.c",
             r"if \(throttle_us >= ITERM_GATE_THR_US\) g->thr_seen = true;",
             "if (throttle_us > ITERM_GATE_THR_US) g->thr_seen = true;",
             "fallback latch releases only above 1250 us"),
            ("iterm_gate.c", r"g->hold = !g->thr_seen;",
             "g->hold = throttle_us < ITERM_GATE_THR_US;",
             "fallback not latched - holds again at idle in the air"),
            ("iterm_gate.c",
             r"g->hold = false;\n(\s+)return false;",
             "g->hold = true;\n\\1return true;",
             "disarmed reported as held"),
            ("iterm_gate.c",
             r"\(g->lifted   \? ITERM_GATE_F_LIFTED  : 0u\)",
             "(g->lifted   ? ITERM_GATE_F_FRESH   : 0u)",
             "LIFTED logged in the FRESH bit"),
            ("iterm_gate.h", r"#define ITERM_GATE_NEAR_MM   30 ",
             "#define ITERM_GATE_NEAR_MM   40 ",
             "on-its-legs band widened to 4 cm"),
            ("iterm_gate.h", r"#define ITERM_GATE_THR_US    1250u",
             "#define ITERM_GATE_THR_US    1200u",
             "throttle threshold moved to 1200 us"),
            ("iterm_gate.h", r"#define ITERM_GATE_FRESH_US  500000",
             "#define ITERM_GATE_FRESH_US  5000000",
             "freshness window stretched to 5 s"),
        ],
    ),
    # The SPI-only config and the refusal diagnostics. The
    # older liveness checks predate this driver and have no curated set.
    "lsm6dsox": (
        os.path.join(ROOT, "components", "lsm6dsox"),
        "LSM_SRC",
        os.path.join(HERE, "lsm6dsox_liveness_test.py"),
        [
            ("lsm6dsox.h", r"#define LSM6DSOX_CTRL9_XL_VAL 0xE2",
             "#define LSM6DSOX_CTRL9_XL_VAL 0xE0",
             "I3C left enabled (the reset default)"),
            ("lsm6dsox.h", r"#define LSM6DSOX_CTRL4_C_VAL  0x06",
             "#define LSM6DSOX_CTRL4_C_VAL  0x02",
             "I2C left enabled (the old CTRL4_C)"),
            ("lsm6dsox.h", r"#define LSM6DSOX_CTRL3_C_VAL  0x44",
             "#define LSM6DSOX_CTRL3_C_VAL  0x40",
             "BDU written over IF_INC - the 12-byte burst loses auto-increment"),
            ("lsm6dsox.c",
             r"    write_reg\(dev->spi_dev, LSM6DSOX_CTRL9_XL, LSM6DSOX_CTRL9_XL_VAL\);\n",
             "",
             "the I3C write dropped from init"),
            ("lsm6dsox.c", r"    if \(bad\) \{", "    if (0) {",
             "a config that did not stick is only logged, init carries on"),
            ("lsm6dsox.c",
             r"        read_reg\(dev->spi_dev, k_cfg\[i\]\.reg, &v2, 1\);\n",
             "",
             "no re-read - one garbled read-back fails the boot"),
            ("lsm6dsox.c", r"        s_diag\.episodes\+\+;\n", "",
             "episodes never counted"),
            ("lsm6dsox.c",
             r"\} else if \(dev->refused_run == LSM6DSOX_STICKY_READS\) \{",
             "} else if (dev->refused_run == LSM6DSOX_STICKY_READS + 1) {",
             "sticky dump one refusal late"),
            ("lsm6dsox.c", r"\? \(int8_t\)gpio_get_level\(\(gpio_num_t\)dev->int1_gpio\) : -1;",
             "? 0 : -1;",
             "INT1 level never actually sampled"),
            ("lsm6dsox.c", r"        dev->refused_run = 0;\n", "",
             "a good read does not close the episode"),
            ("lsm6dsox.c", r"        s_diag\.have_sticky = false;.*\n", "",
             "an old episode's sticky dump reported under a new one"),
            ("lsm6dsox.c", r"        note_refusal\(dev, 2, raw\[0\]\);\n", "",
             "FROZEN refusals not counted as an episode"),
            ("lsm6dsox.c", r"for \(int i = 0; i < 8; i\+\+\) \{\n        esp_err_t e",
             "for (int i = 0; i < 0; i++) {\n        esp_err_t e",
             "dump never reads the registers"),
            ("lsm6dsox.c", r"dev->int1_gpio = -1;", "dev->int1_gpio = 0;",
             "INT1 pin defaults to GPIO0 instead of unknown"),
        ],
    ),
    # C6 -> S3 command-link line hygiene.
    "telem_line": (
        os.path.join(ROOT, "components", "telemetry"),
        "TELEM_LINE_SRC",
        os.path.join(HERE, "telem_line_test.py"),
        [
            ("telem_line.c", r"u > 0x7E\)", "u > 0x7F)",
             "DEL accepted as printable"),
            ("telem_line.c", r"u < 0x20 \|\|", "u < 0x10 ||",
             "control characters 0x10..0x1F accepted"),
            ("telem_line.c", r"    if \(s->pos > 0\) drop\(s, s->pos\);",
             "    if (0) drop(s, s->pos);",
             "silence keeps the partial line - THE BUG"),
            ("telem_line.c", r"    s->skip = true;", "    s->skip = false;",
             "overlong tail run as a command"),
            ("telem_line.c", r"    memcpy\(s->junk_head, s->buf, \(size_t\)s->junk_head_n\);\n",
             "",
             "junk counted but never sampled"),
            ("telem_line.c", r"    s->pos = 0;\n\}", "}",
             "a drop leaves the junk in the buffer"),
            ("telem_line.c", r"        s->lines\+\+;\n", "",
             "good lines not counted"),
            ("telem_line.c",
             r"if \(b == '\\n' \|\| b == '\\r'\) \{\n        const int n",
             r"if (b == '\\n') {\n        const int n",
             "CR not a terminator - CRLF commands carry a CR"),
            ("telem_line.c", r"    s->skip = false;             /\* silence",
             "    /* silence",
             "silence does not end an overlong skip"),
            ("telem_line.c", r"drop\(s, n\);\n                return NULL;",
             "drop(s, n);\n                return s->buf;",
             "a noisy line is counted AND still run"),
        ],
    ),
}


def run_one(src_dir, env_var, test, fname, pattern, repl, why):
    with tempfile.TemporaryDirectory() as tmp:
        dst = os.path.join(tmp, "src")
        shutil.copytree(src_dir, dst)
        path = os.path.join(dst, fname)
        with open(path, "r", encoding="utf-8") as fh:
            text = fh.read()
        new, n = re.subn(pattern, repl, text, count=1)
        if n != 1:
            return "NOPATCH", why
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(new)
        env = dict(os.environ, **{env_var: dst})
        r = subprocess.run([sys.executable, test], env=env,
                           capture_output=True, text=True)
        # Non-zero means the suite noticed. A compile failure also counts as
        # noticed, but is reported separately so it cannot be mistaken for a
        # behavioural catch.
        if r.returncode == 0:
            return "SURVIVED", why
        if "clang failed" in (r.stdout + r.stderr):
            return "NOCOMPILE", why
        return "KILLED", why


def main():
    which = sys.argv[1:] or list(TARGETS)
    total_k = total_s = total_bad = 0
    for name in which:
        if name not in TARGETS:
            print(f"unknown target {name!r}; known: {', '.join(TARGETS)}")
            return 2
        src, env_var, test, mutants = TARGETS[name]
        print(f"\n=== {name}: {len(mutants)} mutants ===")
        for fname, pat, repl, why in mutants:
            status, why = run_one(src, env_var, test, fname, pat, repl, why)
            mark = {"KILLED": "kill", "SURVIVED": "SURVIVED",
                    "NOPATCH": "NO-PATCH", "NOCOMPILE": "NO-COMPILE"}[status]
            print(f"  [{mark:>10}] {why}")
            if status == "KILLED":
                total_k += 1
            elif status == "SURVIVED":
                total_s += 1
            else:
                total_bad += 1
    print(f"\n{total_k} killed, {total_s} survived, {total_bad} not applied")
    if total_s or total_bad:
        print("a survivor is a hole in the tests, not a curiosity\n")
    return 1 if (total_s or total_bad) else 0


if __name__ == "__main__":
    sys.exit(main())
