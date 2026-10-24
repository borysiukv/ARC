// ARC'26 COMPACT — ESP32 AP + LittleFS UI + Manual/Grid/Step/Pick&Place API
// SIMULATION ONLY. No motors, limit switches or physical safety system wired here.
// Supports Arduino-ESP32 3.x and ArduinoJson 7.x.
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <math.h>

// Change the demo password before sharing a physical Wi-Fi network.
static const char *AP_SSID = "ARC26-Robot";
static const char *AP_PASSWORD = "ARC26_demo_change_me";
constexpr int MAX_CELLS = 64;
constexpr int X_MAX = 5;
constexpr int AXES = 5;
const char *AXIS_NAMES[AXES] = {"base", "shoulder", "elbow", "gripper", "rail"};
WebServer web(80);

struct GridConfig {
  int columns = 3, rows = 3;
  float pitchX = 50, pitchY = 50, originX = 0, originY = 0;
  float railTravel = 300, rotationStep = 15, baseZero = 0;
  float gripOpen = 35, gripClosed = 90;
} cfg;
struct Pose {
  float a[AXES] = {90, 90, 90, 35, 0};
  int x = 0, r = 6;
  bool closed = false;
};
struct CellPose {
  bool taught = false;
  Pose pose;
} cells[MAX_CELLS];
Pose current;
Pose savedHome, savedSafe;
bool locked = false, active = false;
int cycleIndex = 0;
String fromCell, toCell, lastAction;
unsigned long commandCount = 0;
const char *PHASES[8] = {"APPROACH", "CLOSE", "SAFE", "TRAVEL", "LOWER", "OPEN", "RAISE", "HOME"};

bool isNumber(JsonVariantConst v) {
  return v.is<int>() || v.is<long>() || v.is<float>() || v.is<double>();
}
float asNumber(JsonVariantConst v, float fallback) {
  if (!isNumber(v)) return fallback;
  float n = v.as<float>();
  return isfinite(n) ? n : fallback;
}
void jsonReply(int status, JsonDocument &doc) {
  String body; serializeJson(doc, body);
  web.sendHeader("Cache-Control", "no-store");
  web.send(status, "application/json; charset=utf-8", body);
}
void sendError(int status, const String &reason) {
  JsonDocument doc; doc["error"] = reason; jsonReply(status, doc);
}
bool loadRequest(JsonDocument &doc) {
  if (!web.hasArg("plain") || web.arg("plain").length() > 12000) {
    sendError(400, "JSON body missing or too large"); return false;
  }
  if (deserializeJson(doc, web.arg("plain")) || !doc.is<JsonObject>()) {
    sendError(400, "Expected valid JSON object"); return false;
  }
  return true;
}
bool cellId(const String &name, int &idx) {
  if (name.length() < 2 || name.length() > 3) return false;
  char letter = name[0];
  if (letter < 'A' || letter >= 'A' + cfg.columns) return false;
  String tail = name.substring(1);
  for (unsigned int i = 0; i < tail.length(); ++i)
    if (tail[i] < '0' || tail[i] > '9') return false;
  int row = tail.toInt();
  if (row < 1 || row > cfg.rows) return false;
  idx = (row - 1) * cfg.columns + (letter - 'A');
  return idx >= 0 && idx < MAX_CELLS;
}
String nameOf(int idx) {
  return String(char('A' + (idx % cfg.columns))) + String(idx / cfg.columns + 1);
}
void writeAxes(JsonObject obj, const Pose &p) {
  for (int i = 0; i < AXES; ++i) obj[AXIS_NAMES[i]] = p.a[i];
}
void writePose(JsonObject obj, const Pose &p) {
  for (int i = 0; i < AXES; ++i) obj[AXIS_NAMES[i]] = p.a[i];
  obj["x_step"] = p.x; obj["rotation_step"] = p.r;
  obj["grip_closed"] = p.closed;
}
bool validatePose(JsonVariantConst src, Pose &out) {
  if (!src.is<JsonObjectConst>()) return false;
  auto o = src.as<JsonObjectConst>();
  for (int i = 0; i < AXES; ++i) {
    auto v = o[AXIS_NAMES[i]];
    if (!isNumber(v)) return false;
    float n = v.as<float>();
    if (!isfinite(n) || n < 0 || n > (i == 4 ? 100 : 180)) return false;
    out.a[i] = n;
  }
  if (!o["grip_closed"].is<bool>()) return false;
  out.closed = o["grip_closed"].as<bool>();
  if (o["x_step"].is<int>() && o["rotation_step"].is<int>()) {
    out.x = o["x_step"].as<int>(); out.r = o["rotation_step"].as<int>();
  } else { // compatibility with previous v0.5 GRID poses
    out.x = constrain(int(lroundf(out.a[4] * X_MAX / 100.0f)), 0, X_MAX);
    out.r = int(lroundf((out.a[0] - cfg.baseZero) / cfg.rotationStep));
  }
  if (out.x < 0 || out.x > X_MAX || out.r < 0 || out.r > 12) return false;
  float baseAngle = cfg.baseZero + out.r * cfg.rotationStep;
  return baseAngle >= 0 && baseAngle <= 180;
}
void writeGrid(JsonObject g) {
  g["columns"] = cfg.columns; g["rows"] = cfg.rows;
  g["pitch_x_mm"] = cfg.pitchX; g["pitch_y_mm"] = cfg.pitchY;
  g["origin_x_mm"] = cfg.originX; g["origin_y_mm"] = cfg.originY;
  g["rail_travel_mm"] = cfg.railTravel;
  g["rotation_step_deg"] = cfg.rotationStep; g["base_zero_deg"] = cfg.baseZero;
  g["grip_open_deg"] = cfg.gripOpen; g["grip_closed_deg"] = cfg.gripClosed;
}
bool validateGrid(JsonVariantConst v, GridConfig &out) {
  if (!v.is<JsonObjectConst>()) return false;
  auto g = v.as<JsonObjectConst>();
  if (!g["columns"].is<int>() || !g["rows"].is<int>()) return false;
  out.columns = g["columns"].as<int>(); out.rows = g["rows"].as<int>();
  if (out.columns < 1 || out.columns > 8 || out.rows < 1 || out.rows > 8) return false;
  const char *keys[9] = {"pitch_x_mm", "pitch_y_mm", "origin_x_mm", "origin_y_mm", "rail_travel_mm", "rotation_step_deg", "base_zero_deg", "grip_open_deg", "grip_closed_deg"};
  float *values[9] = {&out.pitchX, &out.pitchY, &out.originX, &out.originY, &out.railTravel, &out.rotationStep, &out.baseZero, &out.gripOpen, &out.gripClosed};
  for (int i = 0; i < 9; ++i) {
    if (!isNumber(g[keys[i]])) return false;
    *values[i] = g[keys[i]].as<float>();
    if (!isfinite(*values[i])) return false;
  }
  if (out.pitchX < 1 || out.pitchX > 500 || out.pitchY < 1 || out.pitchY > 500) return false;
  if (out.railTravel < 1 || out.railTravel > 3000) return false;
  if (out.rotationStep < 1 || out.rotationStep > 90) return false;
  return out.baseZero >= 0 && out.baseZero <= 180 && out.gripOpen >= 0 && out.gripOpen <= 180 && out.gripClosed >= 0 && out.gripClosed <= 180;
}
void saveCalibration() {
  JsonDocument doc;
  writeGrid(doc["grid"].to<JsonObject>());
  JsonObject cp = doc["cells"].to<JsonObject>();
  for (int i = 0; i < cfg.columns * cfg.rows; ++i)
    if (cells[i].taught) writePose(cp[nameOf(i)].to<JsonObject>(), cells[i].pose);
  File f = LittleFS.open("/calibration.json", "w");
  if (f) {serializeJson(doc, f); f.close();}
}
void restoreCalibration() {
  File f = LittleFS.open("/calibration.json", "r");
  if (!f) return;
  JsonDocument d;
  DeserializationError err = deserializeJson(d, f);
  f.close();
  if (err) return;
  GridConfig next = cfg;
  if (!validateGrid(d["grid"], next)) return;
  cfg = next;
  JsonObjectConst entries = d["cells"].as<JsonObjectConst>();
  for (JsonPairConst p : entries) {
    int i = -1; Pose pos;
    if (cellId(String(p.key().c_str()), i) && validatePose(p.value(), pos)) {
      cells[i].taught = true; cells[i].pose = pos;
    }
  }
}
void writeCycle(JsonDocument &d) {
  if (!active) {d["cycle"] = nullptr; return;}
  JsonObject o = d["cycle"].to<JsonObject>();
  o["from"] = fromCell; o["to"] = toCell;
  o["step"] = cycleIndex; o["total"] = 8;
  o["last_step"] = lastAction.length() ? lastAction : "";
}
void writeStep(JsonDocument &d) {
  d["x_step"] = current.x; d["rotation_step"] = current.r;
  d["rotation_deg"] = cfg.baseZero + current.r * cfg.rotationStep;
  d["grip_closed"] = current.closed;
  d["locked"] = locked; d["simulation_only"] = true;
}
void sendState() {
  JsonDocument doc;
  writeAxes(doc["axes"].to<JsonObject>(), current);
  doc["locked"] = locked; doc["mode"] = "ESP32_SIM";
  doc["hardware"] = false; doc["simulation_only"] = true;
  doc["commands"] = commandCount;
  writeCycle(doc); jsonReply(200, doc);
}
void sendGrid() {
  JsonDocument d;
  writeGrid(d["grid"].to<JsonObject>());
  JsonObject taught = d["cells"].to<JsonObject>();
  JsonObject geometry = d["logical_cells"].to<JsonObject>();
  for (int i = 0; i < cfg.columns * cfg.rows; ++i) {
    String name = nameOf(i);
    int col = i % cfg.columns, row = i / cfg.columns;
    JsonObject xy = geometry[name].to<JsonObject>();
    xy["x_mm"] = cfg.originX + col * cfg.pitchX;
    xy["y_mm"] = cfg.originY + row * cfg.pitchY;
    if (cells[i].taught) writePose(taught[name].to<JsonObject>(), cells[i].pose);
  }
  writeAxes(d["axes"].to<JsonObject>(), current);
  d["locked"] = locked; d["simulation_only"] = true;
  writeCycle(d); jsonReply(200, d);
}
void sendStep() {JsonDocument d; writeStep(d); jsonReply(200, d);}
void setGrip(bool closed) {
  current.closed = closed;
  current.a[3] = closed ? cfg.gripClosed : cfg.gripOpen;
}
void setX(int x) {
  current.x = x;
  current.a[4] = x * 100.0f / X_MAX;  // legacy rail percent, NOT actual mm.
}
void setR(int r) {
  current.r = r;
  current.a[0] = cfg.baseZero + r * cfg.rotationStep;
}
Pose safeAt(const Pose &reference, bool gripped) {
  Pose result = savedSafe;
  result.x = reference.x; result.r = reference.r;
  result.a[0] = reference.a[0]; result.a[4] = reference.a[4];
  result.closed = gripped;
  result.a[3] = gripped ? cfg.gripClosed : cfg.gripOpen;
  return result;
}
void advanceCycle() {
  int a = -1, b = -1;
  if (!cellId(fromCell, a) || !cellId(toCell, b) || !cells[a].taught || !cells[b].taught) {
    active = false; return;
  }
  const Pose &source = cells[a].pose, &target = cells[b].pose;
  switch (cycleIndex) {
    case 0: current = source; setGrip(false); break;
    case 1: current = source; setGrip(true); break;
    case 2: current = safeAt(source, true); break;
    case 3: current = safeAt(target, true); break;
    case 4: current = target; setGrip(true); break;
    case 5: current = target; setGrip(false); break;
    case 6: current = safeAt(target, false); break;
    case 7: current = savedHome; setGrip(false); break;
  }
  lastAction = PHASES[cycleIndex];
  ++cycleIndex;
  if (cycleIndex >= 8) active = false;
}
void dispatchApi() {
  String path = web.uri();
  if (web.method() == HTTP_GET) {
    if (path == "/api/health") {
      JsonDocument d; d["app"] = "ARC26 Compact ESP32"; d["version"] = "0.8";
      d["grid_enabled"] = true; d["step_enabled"] = true;
      d["simulation_only"] = true; jsonReply(200, d); return;
    }
    if (path == "/api/state") {sendState(); return;}
    if (path == "/api/step") {sendStep(); return;}
    if (path == "/api/grid") {sendGrid(); return;}
    if (path == "/api/positions") {
      JsonDocument d; JsonObject o = d["positions"].to<JsonObject>();
      writeAxes(o["HOME"].to<JsonObject>(), savedHome);
      writeAxes(o["SAFE"].to<JsonObject>(), savedSafe);
      jsonReply(200, d); return;
    }
    sendError(404, "Unknown endpoint"); return;
  }
  if (web.method() != HTTP_POST) {sendError(405, "Method not allowed"); return;}
  // Software lock is NOT a hardware E-stop.
  if (path == "/api/stop" || path == "/api/grid/stop") {
    locked = true; active = false; ++commandCount; sendState(); return;
  }
  if (path == "/api/unlock" || path == "/api/grid/unlock") {
    locked = false; ++commandCount; sendState(); return;
  }
  if (locked) {sendError(409, "Control locked"); return;}
  JsonDocument req;
  if (!loadRequest(req)) return;
  if (active && path != "/api/grid/cycle/next" && path != "/api/grid/cycle/reset") {
    sendError(409, "Cycle active"); return;
  }
  if (path == "/api/step/x") {
    if (!req["step"].is<int>()) {sendError(400, "step must be integer"); return;}
    int x = req["step"].as<int>();
    if (x < 0 || x > X_MAX) {sendError(400, "X step must be 0..5"); return;}
    setX(x); ++commandCount; sendStep(); return;
  }
  if (path == "/api/step/rotation") {
    if (!req["step"].is<int>()) {sendError(400, "step must be integer"); return;}
    int r = req["step"].as<int>(); float angle = cfg.baseZero + r * cfg.rotationStep;
    if (r < 0 || r > 12 || angle < 0 || angle > 180) {sendError(400, "Rotation outside limits"); return;}
    setR(r); ++commandCount; sendStep(); return;
  }
  if (path == "/api/step/grip") {
    if (!req["closed"].is<bool>()) {sendError(400, "closed must be boolean"); return;}
    setGrip(req["closed"].as<bool>()); ++commandCount; sendStep(); return;
  }
  if (path == "/api/position/load") {
    String name = req["name"].as<String>();
    if (name == "HOME") current = savedHome;
    else if (name == "SAFE") current = savedSafe;
    else {int i = -1;
      if (!cellId(name, i) || !cells[i].taught) {
        sendError(400, "Unknown saved position"); return;
      }
      current = cells[i].pose;
    }
    ++commandCount; sendState(); return;
  }
  if (path == "/api/axes") {
    Pose p = current;
    // Manual set preserves step fields only approximately.
    JsonObjectConst o = req["axes"].as<JsonObjectConst>();
    if (o.isNull()) {sendError(400, "axes object required"); return;}
    for (int i = 0; i < AXES; ++i) {
      auto v = o[AXIS_NAMES[i]];
      if (!isNumber(v)) {sendError(400, "Missing axis"); return;}
      float n = v.as<float>();
      if (!isfinite(n) || n < 0 || n > (i == 4 ? 100 : 180)) {sendError(400, "Invalid axis"); return;}
      p.a[i] = n;
    }
    p.x = constrain(int(lroundf(p.a[4] * X_MAX / 100.f)), 0, X_MAX);
    p.r = constrain(int(lroundf((p.a[0] - cfg.baseZero) / cfg.rotationStep)), 0, 12);
    current = p; ++commandCount; sendState(); return;
  }
  if (path == "/api/grid/config") {
    GridConfig next = cfg;
    if (!validateGrid(req["grid"], next)) {sendError(400, "Invalid grid settings"); return;}
    bool dimensionsChanged = next.rows != cfg.rows || next.columns != cfg.columns;
    cfg = next;
    if (dimensionsChanged) for (auto &cell : cells) cell.taught = false;
    saveCalibration(); ++commandCount; sendGrid(); return;
  }
  if (path == "/api/grid/teach") {
    int i = -1; String name = req["cell"].as<String>();
    Pose p;
    if (!cellId(name, i) || !validatePose(req["pose"], p) || p.closed) {
      sendError(400, "Invalid cell/pose; teach with grip open"); return;
    }
    cells[i].pose = p; cells[i].taught = true;
    saveCalibration(); ++commandCount; sendGrid(); return;
  }
  if (path == "/api/grid/goto") {
    int i = -1; String name = req["cell"].as<String>();
    if (!cellId(name, i) || !cells[i].taught) {sendError(400, "Cell not taught"); return;}
    current = cells[i].pose; ++commandCount; sendGrid(); return;
  }
  if (path == "/api/grid/cycle/start") {
    int a = -1, b = -1;
    String aa = req["from"].as<String>(), bb = req["to"].as<String>();
    if (!cellId(aa, a) || !cellId(bb, b) || a == b || !cells[a].taught || !cells[b].taught) {
      sendError(400, "Teach two distinct cells first"); return;
    }
    fromCell = aa; toCell = bb; active = true;
    cycleIndex = 0; lastAction = "";
    ++commandCount; sendGrid(); return;
  }
  if (path == "/api/grid/cycle/next") {
    if (!active) {sendError(400, "No active cycle"); return;}
    advanceCycle(); ++commandCount; sendGrid(); return;
  }
  if (path == "/api/grid/cycle/reset") {
    active = false; cycleIndex = 0; lastAction = "";
    ++commandCount; sendGrid(); return;
  }
  sendError(404, "Unknown endpoint");
}
void servePage() {
  File file = LittleFS.open("/index.html", "r");
  if (!file) {web.send(503, "text/plain; charset=utf-8", "Missing index.html: upload LittleFS with pio run -t uploadfs"); return;}
  web.streamFile(file, "text/html; charset=utf-8");
  file.close();
}
void setup() {
  Serial.begin(115200);
  savedHome = Pose{}; savedHome.a[3] = cfg.gripOpen;
  savedSafe = savedHome;
  savedSafe.a[1] = 125; savedSafe.a[2] = 55;
  current = savedHome;
  bool fsOK = LittleFS.begin(false);
  if (!fsOK) Serial.println("ERROR: LittleFS missing. Upload filesystem.");
  else restoreCalibration();
  savedHome.a[3] = cfg.gripOpen;
  savedSafe.a[3] = cfg.gripOpen;
  current = savedHome;
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(AP_SSID, AP_PASSWORD)) Serial.println("ERROR: WiFi AP start failed");
  web.on("/", HTTP_GET, servePage);
  web.on("/grid", HTTP_GET, servePage);
  web.on("/index.html", HTTP_GET, servePage);
  web.on("/favicon.ico", HTTP_GET, [](){web.send(204);});
  // Handler covers GET and POST; /api/* always returns JSON.
  web.onNotFound([]() {
    if (web.uri().startsWith("/api/")) dispatchApi();
    else sendError(404, "Page not found");
  });
  web.begin();
  Serial.printf("\nARC26 Compact ready (SIMULATED; motors disabled)\nSSID: %s\nURL: http://%s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
}
void loop() {web.handleClient();delay(2);}
