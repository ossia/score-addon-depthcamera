// End-to-end validation of the depth-camera device, inside score.
//
// Covers the chain the other harnesses do not: the protocol factory, the
// enumerators, DeviceSettings serialization through QVariant, device creation
// and the resulting node tree -- i.e. everything a user touches in the device
// dialog.
//
// Run headless:
//   ossia-score --no-gui --script="$(cat depthcam_e2e.js)"
//
// --no-gui matters: with a GUI the start screen defers document creation, and
// the script only evaluates once a document exists. Results go to a file
// because console.log is not routed to stdout in this configuration.

var PROTOCOL = "a3bd48ba-f5db-43b7-aa92-2b1a37a88a79"; // Depth Camera Input
var REPORT = "/tmp/depthcam_report.txt";

var out = [];
var failures = 0, checks = 0;

function log(s) { out.push(s); Util.writeFile(REPORT, out.join("\n")); }
function check(cond, what) {
  checks++;
  if (!cond) failures++;
  log((cond ? "  PASS  " : "  FAIL  ") + what);
  return cond;
}
function finish() {
  log(failures === 0 ? "=== ALL " + checks + " CHECKS PASSED ==="
                     : "=== " + failures + " OF " + checks + " CHECKS FAILED ===");
}

try {
  log("=== depth camera end-to-end ===");

  // --- 1. the protocol enumerates cameras -----------------------------------
  var en = Score.enumerateDevices(PROTOCOL);
  check(en !== null && en !== undefined, "protocol exposes an enumerator");

  var cams = [];
  if (en) {
    en.enumerate = true;
    var list = en.devices;
    for (var i = 0; i < list.length; i++)
      cams.push({ name: list[i].name, category: list[i].category, settings: list[i].settings });
  }

  log("  enumerated " + cams.length + " camera(s):");
  for (var i = 0; i < cams.length; i++)
    log("    [" + cams[i].category + "] " + cams[i].name);
  check(cams.length > 0, "at least one camera enumerated");

  // --- 2. each camera creates and exposes its streams -----------------------
  var expected = ["rgb", "depth", "pointcloud"];

  for (var i = 0; i < cams.length; i++) {
    var name = "e2e_" + i;
    log("  --- " + cams[i].name + " ---");

    Score.createDevice(name, PROTOCOL, cams[i].settings);
    var dev = Score.device(name);
    if (!check(dev !== null && dev !== undefined, "device created")) continue;

    var nodes = [];
    Score.iterateDevice(name, function(a) {
      // String(a) collapses to just the device name; the node path lives on
      // the object.
      // iterateDevice passes the full address as a string, "device:/path".
      nodes.push(String(a));
    });
    log("    nodes: " + nodes.join(", "));

    for (var e = 0; e < expected.length; e++) {
      var want = expected[e], has = false;
      for (var c = 0; c < nodes.length; c++)
        if (nodes[c].indexOf(want) >= 0) has = true;
      check(has, "exposes '" + want + "'");
    }

    Score.removeDevice(name);
  }

  // --- 3. a hand-typed address, the path a networked camera takes -----------
  // Impossible before the settings widget gained an editable address field.
  log("  --- manual address ---");
  Score.createDevice("e2e_manual", PROTOCOL,
                     { device: "", rgb: true, depth: true, pointcloud: true });
  check(Score.device("e2e_manual") !== null, "created from a typed address");
  Score.removeDevice("e2e_manual");

  finish();
} catch (e) {
  log("EXCEPTION: " + e);
  failures++;
  finish();
}
