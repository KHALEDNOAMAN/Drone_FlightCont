/* dash_health_test.js -- regression test for the sensor-health chips on the
 * C6 dashboard.  Run:  node tools/dash_health_test.js
 * Optional arg: a path to an alternate index.html (used for mutation runs).
 *
 * WHY THIS EXISTS.  The dashboard read field 52 (the sensor_health bitmask)
 * as `parseInt(f[52], 10) || 0`.  An empty or non-numeric field became 0,
 * and 0 paints all four chips red -- so "I could not read it" was displayed
 * as "every sensor is dead".  The realistic trigger is not a dropped frame:
 * the page is embedded in the C6 binary, so a C6 that was not reflashed
 * after an S3 telemetry-format change parses the new line with the old
 * field indices, index 52 lands on another column, and a stale-C6 mistake
 * is reported as four dead sensors.
 *
 * MUTATION-TESTED.  Restoring the old one-liner in a copy of
 * index.html fails 6 of these 14 checks (the empty, non-numeric, partial-
 * number, dimming, reason and recovery cases).  A test that has never been
 * made to fail is not evidence.  Worth noting what the mutant printed for
 * a partial number: parseInt("12abc") = 12, which rendered as IMU and BARO
 * dead and MAG and 9-DOF alive -- a completely invented reading.
 *
 * The page script is "use strict", so declarations do not leak out of eval;
 * it is run under vm and asked to hand back render() explicitly.
 */
const fs = require("fs"), vm = require("vm");
const src = fs.readFileSync(process.argv[2] || "tools/c6_wifi_rc/main/web/index.html", "utf8");
const js  = [...src.matchAll(/<script[^>]*>([\s\S]*?)<\/script>/gi)].map(m=>m[1]).join("\n");

const els = {};
function el(id){
  return els[id] || (els[id] = {
    id, className:"", textContent:"", value:"", innerHTML:"",
    style:new Proxy({},{set:()=>true,get:()=>""}),
    classList:{add(){},remove(){},toggle(){}},
    addEventListener(){}, appendChild(){}, querySelector:()=>null,
  });
}
const sandbox = {
  console,
  document:{
    getElementById: el, querySelector: () => el("_q"), querySelectorAll: () => [],
    getElementsByClassName: () => [], getElementsByTagName: () => [],
    addEventListener(){}, body: el("_body"), createElement: () => el("_c"),
  },
  window:{ addEventListener(){}, location:{host:"x"} },
  setInterval:()=>0, setTimeout:()=>0, clearInterval(){}, clearTimeout(){},
  fetch:()=>new Promise(()=>{}),
  WebSocket:function(){ this.addEventListener=()=>{}; },
  localStorage:{ getItem:()=>null, setItem(){}, removeItem(){} },
  Math, JSON, Date, parseInt, parseFloat, isFinite, isNaN, String, Number, Array, Object,
};
sandbox.globalThis = sandbox;
vm.runInNewContext(js + "\n;globalThis.__render = render;", sandbox);
const render = sandbox.__render;
if (typeof render !== "function") { console.log("could not reach render()"); process.exit(2); }

/* A well-formed extended line: 68 comma-separated fields, mostly zeros. */
function line(h){ const f = new Array(68).fill("0"); f[52] = h; return f.join(","); }
const chips = () => ["h_imu","h_baro","h_mag","h_9dof"].map(i => el(i).className.replace("chip","").trim() || "—");
const seed  = () => render(line("15"));

let fail = 0;
function check(name, got, want){
  const ok = JSON.stringify(got) === JSON.stringify(want);
  if (!ok) fail++;
  console.log(`  [${ok?"PASS":"FAIL"}] ${name}`);
  if (!ok) console.log(`         got  ${JSON.stringify(got)}\n         want ${JSON.stringify(want)}`);
}

console.log("\nfield 52 is a real reading -> render it, whatever it says");
seed();             check("15 = all healthy",          chips(), ["ok","ok","ok","ok"]);
render(line("13")); check("13 = baro down",            chips(), ["ok","bad","ok","ok"]);
render(line("0"));  check("0 = genuinely all dead, NOT hidden", chips(), ["bad","bad","bad","bad"]);
check("     ...and not flagged stale", el("h_stale").textContent, "");

console.log("\nfield 52 unreadable -> hold last good value, never invent 0");
seed();             check("baseline healthy",          chips(), ["ok","ok","ok","ok"]);
render(line(""));   check("EMPTY holds  <-- THE REGRESSION (was 4x red)", chips(), ["ok","ok","ok","ok"]);
check("     row dimmed",   el("chips_health").className, "chips stale");
check("     reason shown", /needs reflashing/.test(el("h_stale").textContent), true);
seed(); render(line("nan"));   check("non-numeric holds",         chips(), ["ok","ok","ok","ok"]);
seed(); render(line("12abc")); check("partial number NOT trusted", chips(), ["ok","ok","ok","ok"]);
seed(); render(line("-1"));    check("negative NOT trusted",       chips(), ["ok","ok","ok","ok"]);

console.log("\nrecovery");
render(line("15")); check("good frame clears stale", el("chips_health").className, "chips");
check("     reason cleared", el("h_stale").textContent, "");

console.log("\nshort line -> block never runs, chips untouched (pre-existing)");
seed(); render(new Array(40).fill("0").join(","));
check("40-field line leaves chips alone", chips(), ["ok","ok","ok","ok"]);

/* ---- ToF chip (field 62) -------------------------
 * Driven by field 62, NOT the field-52 bitmask, so it has its own cases.
 * ORDER MATTERS: tofSeen is a page-lifetime latch with no reset, exactly
 * as baro2Seen is, so "never seen" must be asserted before anything sets
 * it. Do not reorder these without reading renderTofChip.            */
function lineT(tof){ const f = new Array(68).fill("0"); f[52]="15"; f[62]=tof; return f.join(","); }
const tofChip = () => [el("h_tof").className, el("h_tof").textContent];

console.log("\nToF chip: absent vs died -- different states, different actions");
render(lineT("0"));
check("0 before ever alive = 'not seen', NOT red", tofChip(), ["chip","— ToF (not seen)"]);
render(lineT("1700"));
check("1700 mm = alive with a range",             tofChip(), ["chip ok","✓ ToF 1.70 m"]);
render(lineT("65535"));
check("65535 = alive, no target -> STILL OK  <-- the sunlight trap",
                                                  tofChip(), ["chip ok","✓ ToF (no target)"]);
render(lineT("0"));
check("0 AFTER being alive = DIED (red)",         tofChip(), ["chip bad","✗ ToF DIED"]);

console.log("\nToF chip: field 62 validated like field 52, never a bare parseInt");
render(lineT("12abc")); check("partial number NOT trusted", tofChip(), ["chip","— ToF (unreadable)"]);
render(lineT(""));      check("empty NOT trusted",          tofChip(), ["chip","— ToF (unreadable)"]);
render(lineT("-1"));    check("negative NOT trusted",       tofChip(), ["chip","— ToF (unreadable)"]);
render(new Array(40).fill("0").join(","));
check("short line = no field 62 at all",          tofChip(), ["chip","— ToF (unreadable)"]);

console.log("\nToF chip is independent of the field-52 bitmask");
const f52bad = new Array(68).fill("0"); f52bad[52] = ""; f52bad[62] = "1200";
render(f52bad.join(","));
check("unreadable field 52 does NOT blank the ToF chip", tofChip(), ["chip ok","✓ ToF 1.20 m"]);

/* ---- Arming blockers (field 54) -------------------
 * WHY. BLOCK_BITS stopped at 64. 969c652 added bit 7 (128,
 * PREARM_PARAMS_DEFAULTED) to the firmware and never touched this file,
 * and two later commits edited index.html without noticing. A board
 * blocked ONLY by that bit therefore rendered "arming blocked:" with an
 * empty list -- which the comment above BLOCK_BITS itself warns is worse
 * than no message, because it reads like a firmware fault rather than a
 * pre-flight check doing its job. Nothing tested this file's blocker
 * list, so nothing caught it.
 *
 * The unmapped-bit case is here so the NEXT bit cannot go silent the
 * same way: an unrecognised blocker must say something, not nothing.  */
function lineB(mask){
  const f = new Array(68).fill("0"); f[52] = "15"; f[54] = mask;
  return f.join(",");
}
const blockedMsg = () => el("blocked").textContent;

console.log("\narming blockers: every bit the firmware can set must be named");
render(lineB("1"));
check("bit 0 gyro-moving", blockedMsg(), "arming blocked: gyro-moving");
render(lineB("32"));
check("bit 5 sensor dead", blockedMsg(), "arming blocked: SENSOR DEAD");
render(lineB("128"));
check("bit 7 params-defaulted  <-- THE REGRESSION (was an empty list)",
      blockedMsg(), "arming blocked: PARAMS DEFAULTED (SAVE to clear)");
render(lineB("160"));
check("bits 5+7 together", blockedMsg(),
      "arming blocked: SENSOR DEAD, PARAMS DEFAULTED (SAVE to clear)");
render(lineB("256"));
check("bit 8 blackbox not ready (arm_blocked is 16 bit)",
      blockedMsg(), "arming blocked: BLACKBOX NOT READY (BBINFO)");
render(lineB("512"));
check("bit 9 arm switch (the arm-switch interlock)",
      blockedMsg(), "arming blocked: ARM SWITCH: cycle it OFF then ON");
render(lineB("1023"));
check("every bit set names ten blockers",
      blockedMsg().split(", ").length, 10);

/* The unmapped case moved from bit 9 to bit 10 when bit 9 got a name.
 * It has to stay ONE ABOVE the highest named bit, or this section stops
 * testing the fallback and starts testing a real blocker. */
console.log("\narming blockers: a bit with no entry must not render as nothing");
render(lineB("1024"));
check("unmapped bit still says SOMETHING", blockedMsg() !== "" &&
      !/blocked:\s*$/.test(blockedMsg()), true);
check("     ...and shows the raw mask", /1024|0x400/.test(blockedMsg()), true);
render(lineB("1152"));  /* 128 known | 1024 unknown */
check("known + unknown bits: names the known one",
      /params defaulted/i.test(blockedMsg()), true);
check("     ...and still flags the unknown one",
      /1024|0x400/.test(blockedMsg()), true);

console.log("\narming blockers: quiet when there is nothing to say");
render(lineB("0"));
check("no blockers = empty message", blockedMsg(), "");
const armedBlocked = new Array(68).fill("0");
armedBlocked[0] = "1"; armedBlocked[52] = "15"; armedBlocked[54] = "128";
render(armedBlocked.join(","));
check("armed: pre-arm bits are stale, say nothing", blockedMsg(), "");
render(new Array(40).fill("0").join(","));
check("short line leaves the message alone", blockedMsg(), "");

console.log(fail ? `\n${fail} FAILURE(S)\n` : "\nALL PASS\n");
process.exit(fail ? 1 : 0);
