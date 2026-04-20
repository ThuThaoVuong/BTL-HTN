#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <math.h>

// ================= CẤU HÌNH WIFI =================
const char* ssid     = "PTIT_WIFI_KTX";
const char* password = "PTIT@33daimo";

// ================= CẤU HÌNH RADAR =================
#define RADAR_SERIAL Serial2
#define RADAR_BAUD   115200
#define RADAR_RX_PIN 16
#define RADAR_TX_PIN 17

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ================= HẰNG SỐ TRACKING =================
#define RADAR_MAX_TARGETS 2
#define MAX_TRACKS        2
#define MAX_ZONES         5
#define HISTORY_LEN       10

const float MATCH_DIST_MAX     = 900.0f;
const uint32_t TRACK_HOLD_MS   = 5000;
const uint32_t TRACK_REMOVE_MS = 8000;
const float COST_UNMATCHED     = 3000.0f;
const float DT_FALLBACK        = 0.10f;

const float W_DIST  = 1.0f;
const float W_SPEED = 0.35f;
const float W_DIR   = 120.0f;

// ================= STRUCT =================
struct Zone {
  int16_t x_min, y_min, x_max, y_max;
  bool active = false;
};

struct TargetState {
  int16_t  x_history[HISTORY_LEN] = {0};
  int16_t  y_history[HISTORY_LEN] = {0};
  uint8_t  h_idx = 0;
  bool     potential_fall = false;
  uint32_t fall_timer = 0;
  float    smooth_doppler = 0;
  String   action = "STILL";
};

struct Detection {
  bool valid = false;
  int16_t x = 0;
  int16_t y = 0;
  int16_t s = 0;
};

struct Track {
  bool active = false;
  uint8_t id = 0;

  float x = 0;
  float y = 0;
  float vx = 0;
  float vy = 0;

  float pred_x = 0;
  float pred_y = 0;

  int16_t raw_speed = 0;

  uint32_t first_seen = 0;
  uint32_t last_seen = 0;
  uint32_t last_update = 0;

  uint16_t age_frames = 0;
  uint16_t miss_count = 0;

  bool matched_in_frame = false;
};

// ================= BIẾN TOÀN CỤC =================
Zone safeZones[MAX_ZONES];
int zoneCount = 0;

Track tracks[MAX_TRACKS + 1];
TargetState ts[MAX_TRACKS + 1];

// ================= HTML =================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head><meta charset="UTF-8"><title>Radar Safe Guard</title>
<style>
  :root { --bg: #f0f2f5; --card: #ffffff; --text: #333; --safe: #28a745; --danger: #dc3545; }
  body { background: var(--bg); color: var(--text); font-family: 'Segoe UI', sans-serif; text-align: center; margin: 0; padding: 10px; }
  .panel { background: var(--card); max-width: 500px; margin: 10px auto; padding: 15px; border-radius: 12px; box-shadow: 0 4px 10px rgba(0,0,0,0.1); }
  canvas { background: #fafafa; border: 1px solid #ddd; border-radius: 8px; cursor: crosshair; max-width: 100%; }
  .btn { padding: 10px 15px; margin: 5px; cursor: pointer; background: #007bff; color: white; border: none; border-radius: 5px; font-weight: bold; }
  .btn.active { background: #ffc107; color: black; }
  .tracking-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); gap: 10px; max-width: 600px; margin: auto; }
  .card { background: var(--card); padding: 10px; border-radius: 8px; border-left: 5px solid #ccc; text-align: left; font-size: 13px; }
  .card.active { border-left-color: var(--safe); }
  .card.danger { border-left-color: var(--danger); background: #fff5f5; animation: blink 1s infinite; }
  .card.ghost { border-left-color: #999; background: #f7f7f7; opacity: 0.85; }
  @keyframes blink { 50% { opacity: 0.7; } }
  .t-name { font-weight: bold; font-size: 15px; border-bottom: 1px solid #eee; margin-bottom: 5px; }
</style></head><body>

  <div class="panel">
    <h3>HỆ THỐNG GIÁM SÁT AN TOÀN</h3>
    <button class="btn" id="bz" onclick="startDraw()">Vẽ Vùng An Toàn</button>
    <button class="btn" style="background:#6c757d" onclick="clearZone()">Xóa Vùng</button>
    <br><canvas id="rd" width="400" height="400"></canvas>
  </div>

  <div class="tracking-grid" id="tg">
    <div id="card-1" class="card"><div class="t-name">Người 1</div><div id="s-1">Trống</div></div>
    <div id="card-2" class="card"><div class="t-name">Người 2</div><div id="s-2">Trống</div></div>
  </div>

<script>
  let socket = new WebSocket(`ws://${location.host}/ws`);
  let cv = document.getElementById("rd"), ctx = cv.getContext("2d");
  let drawing = false, isDragging = false, startX, startY, currentX, currentY;
  let zones = [];
  let lastTargets = [];

  socket.onmessage = (e) => {
    lastTargets = JSON.parse(e.data);
    render();
    updateUI(lastTargets);
  };

  function startDraw() {
    drawing = true;
    document.getElementById("bz").classList.add("active");
  }

  function clearZone() {
    zones = [];
    socket.send("CLEAR_ALL");
    render();
  }

  cv.onmousedown = (e) => {
    if(!drawing) return;
    startX = e.offsetX;
    startY = e.offsetY;
    isDragging = true;
  };

  cv.onmousemove = (e) => {
    if(isDragging) {
      currentX = e.offsetX;
      currentY = e.offsetY;
      render();
    }
  };

  cv.onmouseup = (e) => {
    if(!isDragging) return;
    isDragging = false;

    let x1 = (startX - 200) * 30, y1 = (200 - startY) * 30;
    let x2 = (e.offsetX - 200) * 30, y2 = (200 - e.offsetY) * 30;

    let newZone = {
      x_min: Math.min(x1, x2),
      y_min: Math.min(y1, y2),
      x_max: Math.max(x1, x2),
      y_max: Math.max(y1, y2)
    };
    zones.push(newZone);

    socket.send(`ADD_ZONE:${newZone.x_min},${newZone.y_min},${newZone.x_max},${newZone.y_max}`);
    drawing = false;
    document.getElementById("bz").classList.remove("active");
    render();
  };

  function updateUI(targets) {
    for(let i = 1; i <= 2; i++) {
      let card = document.getElementById(`card-${i}`);
      let status = document.getElementById(`s-${i}`);
      if(card && status) {
        status.innerText = "Trống";
        card.className = "card";
      }
    }

    targets.forEach(t => {
      let card = document.getElementById(`card-${t.id}`);
      let status = document.getElementById(`s-${t.id}`);
      if(!card || !status) return;

      let ghostText = t.ghost ? "<br><small>Mất dấu tạm thời...</small>" : "";
      status.innerHTML =
        `<b>${t.a}</b><br>X: ${t.x}, Y: ${t.y}<br><small>Move: ${t.dist}mm</small>${ghostText}`;

      if (t.ghost) {
        card.className = "card ghost";
      } else if (t.a.includes("FALL") || t.a.includes("IMMOBILE")) {
        card.className = "card danger";
      } else if (t.a.includes("FALLING")) {
        card.className = "card danger";
      } else {
        card.className = "card active";
      }
    });
  }

  function render() {
    ctx.clearRect(0,0,400,400);

    ctx.strokeStyle = "#eee";
    ctx.lineWidth = 1;
    for(let i = 0; i <= 400; i += 50) {
      ctx.beginPath(); ctx.moveTo(i,0); ctx.lineTo(i,400); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(0,i); ctx.lineTo(400,i); ctx.stroke();
    }

    ctx.fillStyle = "#ffaaaa";
    ctx.font = "bold 14px Arial";
    ctx.textAlign = "left";
    ctx.fillText("← BÊN TRÁI", 10, 210);
    ctx.textAlign = "right";
    ctx.fillText("BÊN PHẢI →", 390, 210);
    ctx.textAlign = "center";
    ctx.fillStyle = "#aaa";
    ctx.fillText("↑ PHÍA TRƯỚC (XA) ↑", 200, 20);

    ctx.fillStyle = "#007bff";
    ctx.beginPath();
    ctx.arc(200, 200, 12, 0, 7);
    ctx.fill();

    ctx.strokeStyle = "#007bff";
    ctx.beginPath();
    ctx.moveTo(200,200);
    ctx.lineTo(170,170);
    ctx.lineTo(230,170);
    ctx.closePath();
    ctx.stroke();

    zones.forEach((z, index) => {
      ctx.fillStyle = "rgba(40,167,69,0.15)";
      ctx.strokeStyle = "#28a745";
      let rx = 200 + z.x_min/30, ry = 200 - z.y_max/30;
      let rw = (z.x_max - z.x_min)/30, rh = (z.y_max - z.y_min)/30;
      ctx.fillRect(rx, ry, rw, rh);
      ctx.strokeRect(rx, ry, rw, rh);
      ctx.fillStyle = "#28a745";
      ctx.font = "10px Arial";
      ctx.textAlign = "left";
      ctx.fillText("Vùng " + (index+1), rx + 5, ry + 12);
    });

    if(isDragging) {
      ctx.strokeStyle = "#007bff";
      ctx.setLineDash([5,5]);
      ctx.strokeRect(startX, startY, currentX - startX, currentY - startY);
      ctx.setLineDash([]);
    }

    lastTargets.forEach(t => {
      let px = 200 + t.x/30;
      let py = 200 - t.y/30;

      if (t.ghost) {
        ctx.fillStyle = "#888";
      } else if (t.a.includes("FALL") || t.a.includes("IMMOBILE")) {
        ctx.fillStyle = "red";
      } else {
        ctx.fillStyle = "#28a745";
      }

      ctx.beginPath();
      ctx.arc(px, py, 10, 0, 7);
      ctx.fill();

      ctx.fillStyle = "black";
      ctx.font = "bold 12px Arial";
      ctx.textAlign = "left";
      ctx.fillText("T" + t.id, px + 14, py - 10);
    });
  }
</script></body></html>
)rawliteral";

// ================= PROTOTYPE =================
bool checkIfInAnySafeZone(int16_t x, int16_t y);
void onWsEvent(AsyncWebSocket *s, AsyncWebSocketClient *c, AwsEventType t, void *arg, uint8_t *d, size_t l);
void processTarget(int16_t x, int16_t y, int16_t s, int id, JsonArray& root);
void handleRadar();

float calcDistance(float x1, float y1, float x2, float y2);
float calcDirectionDiff(float vx1, float vy1, float vx2, float vy2);
void decodeRadarFrame(uint8_t* buf, Detection dets[RADAR_MAX_TARGETS], int &detCount);
void predictTracks(uint32_t now);
float computeMatchCost(const Track& tr, const Detection& det);
void assignDetectionsToTracks(Detection dets[RADAR_MAX_TARGETS], int detCount, uint32_t now);
int getFreeTrackId();
void initTrackFromDetection(Track& tr, const Detection& det, uint32_t now, uint8_t id);
void updateTrackWithDetection(Track& tr, const Detection& det, uint32_t now);
void updateUnmatchedTracks(uint32_t now);
void publishTracks();
void solveAssignmentBruteforce(float cost[MAX_TRACKS][RADAR_MAX_TARGETS], bool trackUsable[MAX_TRACKS], bool detUsable[RADAR_MAX_TARGETS], int detCount, int bestAssign[MAX_TRACKS]);

// ================= HÀM HỖ TRỢ =================
float calcDistance(float x1, float y1, float x2, float y2) {
  float dx = x1 - x2;
  float dy = y1 - y2;
  return sqrtf(dx * dx + dy * dy);
}

float calcDirectionDiff(float vx1, float vy1, float vx2, float vy2) {
  float mag1 = sqrtf(vx1 * vx1 + vy1 * vy1);
  float mag2 = sqrtf(vx2 * vx2 + vy2 * vy2);

  if (mag1 < 1.0f || mag2 < 1.0f) return 0.0f;

  float dot = vx1 * vx2 + vy1 * vy2;
  float c = dot / (mag1 * mag2);
  if (c > 1.0f) c = 1.0f;
  if (c < -1.0f) c = -1.0f;

  return acosf(c);
}

bool checkIfInAnySafeZone(int16_t x, int16_t y) {
  for (int i = 0; i < MAX_ZONES; i++) {
    if (safeZones[i].active) {
      if (x >= safeZones[i].x_min && x <= safeZones[i].x_max &&
          y >= safeZones[i].y_min && y <= safeZones[i].y_max) {
        return true;
      }
    }
  }
  return false;
}

// ================= XỬ LÝ NGÃ =================
void processTarget(int16_t x, int16_t y, int16_t s, int id, JsonArray& root) {
  TargetState& t = ts[id];
  uint32_t now = millis();

  y = -y;

  int16_t x_old = t.x_history[t.h_idx];
  int16_t y_old = t.y_history[t.h_idx];

  t.x_history[t.h_idx] = x;
  t.y_history[t.h_idx] = y;
  t.h_idx = (t.h_idx + 1) % HISTORY_LEN;

  float distanceMoved = sqrtf(powf(x - x_old, 2) + powf(y - y_old, 2));
  float vY = (y - y_old) * 1.0f;

  t.smooth_doppler = (t.smooth_doppler * 0.7f) + (abs(s) * 0.3f);

  bool inSafeZone = checkIfInAnySafeZone(x, y);

  if (!t.potential_fall && !inSafeZone) {
    if (vY < -250 && (t.smooth_doppler > 100 || distanceMoved > 250)) {
      t.potential_fall = true;
      t.fall_timer = now;
    }
  }

  if (t.potential_fall) {
    uint32_t elapsed = now - t.fall_timer;

    if (elapsed > 1000 && (t.smooth_doppler > 150 || distanceMoved > 350 || inSafeZone)) {
      t.potential_fall = false;
      t.action = "MOVING";
    }
    else if (elapsed > 10000) {
      t.action = "IMMOBILE";
    }
    else if (elapsed > 2500) {
      if (t.smooth_doppler < 80 && distanceMoved < 200) t.action = "FALL";
      else t.action = "WAITING";
    }
    else {
      t.action = "FALLING ???";
    }
  } else {
    t.action = (t.smooth_doppler > 35 || distanceMoved > 120) ? "MOVING" : "STILL";
  }

  JsonObject obj = root.createNestedObject();
  obj["id"] = id;
  obj["x"] = x;
  obj["y"] = y;
  obj["spd"] = (int)t.smooth_doppler;
  obj["dist"] = (int)distanceMoved;
  obj["a"] = t.action;
  obj["ghost"] = false;
}

// ================= DECODE RADAR: CHỈ ĐỌC 2 TARGET =================
void decodeRadarFrame(uint8_t* buf, Detection dets[RADAR_MAX_TARGETS], int &detCount) {
  detCount = 0;

  for (int j = 0; j < RADAR_MAX_TARGETS; j++) {
    int off = 4 + (j * 8);

    int16_t raw_x = (buf[off + 1] & 0x80)
      ? -(((buf[off + 1] & 0x7F) << 8) | buf[off])
      : (((buf[off + 1] & 0x7F) << 8) | buf[off]);

    int16_t raw_y = (buf[off + 3] & 0x80)
      ? -(((buf[off + 3] & 0x7F) << 8) | buf[off + 2])
      : (((buf[off + 3] & 0x7F) << 8) | buf[off + 2]);

    int16_t raw_s = (buf[off + 5] & 0x80)
      ? -(((buf[off + 5] & 0x7F) << 8) | buf[off + 4])
      : (((buf[off + 5] & 0x7F) << 8) | buf[off + 4]);

    if (raw_x == 0 && raw_y == 0) continue;

    dets[detCount].valid = true;
    dets[detCount].x = raw_x;
    dets[detCount].y = raw_y;
    dets[detCount].s = raw_s;
    detCount++;
  }
}

// ================= TRACKING =================
void predictTracks(uint32_t now) {
  for (int i = 1; i <= MAX_TRACKS; i++) {
    if (!tracks[i].active) continue;

    float dt = (now - tracks[i].last_update) / 1000.0f;
    if (dt <= 0.0f || dt > 1.0f) dt = DT_FALLBACK;

    tracks[i].pred_x = tracks[i].x + tracks[i].vx * dt;
    tracks[i].pred_y = tracks[i].y + tracks[i].vy * dt;
    tracks[i].matched_in_frame = false;
  }
}

float computeMatchCost(const Track& tr, const Detection& det) {
  float dist = calcDistance(tr.pred_x, tr.pred_y, det.x, det.y);
  if (dist > MATCH_DIST_MAX) return COST_UNMATCHED;

  float det_vx = det.x - tr.x;
  float det_vy = det.y - tr.y;
  float speedDiff = fabsf((float)abs(det.s) - (float)abs(tr.raw_speed));
  float dirDiff = calcDirectionDiff(tr.vx, tr.vy, det_vx, det_vy);

  return (W_DIST * dist) + (W_SPEED * speedDiff) + (W_DIR * dirDiff);
}

void solveAssignmentBruteforce(float cost[MAX_TRACKS][RADAR_MAX_TARGETS],
                               bool trackUsable[MAX_TRACKS],
                               bool detUsable[RADAR_MAX_TARGETS],
                               int detCount,
                               int bestAssign[MAX_TRACKS]) {
  int currentAssign[MAX_TRACKS];
  bool usedDet[RADAR_MAX_TARGETS] = {false, false};
  float bestTotal = 1e9f;

  for (int i = 0; i < MAX_TRACKS; i++) {
    bestAssign[i] = -1;
    currentAssign[i] = -1;
  }

  std::function<void(int, float)> dfs = [&](int ti, float curTotal) {
    if (curTotal >= bestTotal) return;

    if (ti == MAX_TRACKS) {
      if (curTotal < bestTotal) {
        bestTotal = curTotal;
        for (int i = 0; i < MAX_TRACKS; i++) bestAssign[i] = currentAssign[i];
      }
      return;
    }

    if (!trackUsable[ti]) {
      currentAssign[ti] = -1;
      dfs(ti + 1, curTotal);
      return;
    }

    currentAssign[ti] = -1;
    dfs(ti + 1, curTotal + COST_UNMATCHED);

    for (int dj = 0; dj < detCount; dj++) {
      if (!detUsable[dj] || usedDet[dj]) continue;
      if (cost[ti][dj] >= COST_UNMATCHED) continue;

      usedDet[dj] = true;
      currentAssign[ti] = dj;
      dfs(ti + 1, curTotal + cost[ti][dj]);
      usedDet[dj] = false;
      currentAssign[ti] = -1;
    }
  };

  dfs(0, 0.0f);
}

void initTrackFromDetection(Track& tr, const Detection& det, uint32_t now, uint8_t id) {
  tr.active = true;
  tr.id = id;
  tr.x = det.x;
  tr.y = det.y;
  tr.vx = 0;
  tr.vy = 0;
  tr.pred_x = det.x;
  tr.pred_y = det.y;
  tr.raw_speed = det.s;
  tr.first_seen = now;
  tr.last_seen = now;
  tr.last_update = now;
  tr.age_frames = 1;
  tr.miss_count = 0;
  tr.matched_in_frame = true;
}

void updateTrackWithDetection(Track& tr, const Detection& det, uint32_t now) {
  float dt = (now - tr.last_update) / 1000.0f;
  if (dt <= 0.0f || dt > 1.0f) dt = DT_FALLBACK;

  float newVx = (det.x - tr.x) / dt;
  float newVy = (det.y - tr.y) / dt;

  tr.vx = tr.vx * 0.65f + newVx * 0.35f;
  tr.vy = tr.vy * 0.65f + newVy * 0.35f;

  tr.x = det.x;
  tr.y = det.y;
  tr.raw_speed = det.s;
  tr.last_seen = now;
  tr.last_update = now;
  tr.age_frames++;
  tr.miss_count = 0;
  tr.matched_in_frame = true;
}

int getFreeTrackId() {
  for (int i = 1; i <= MAX_TRACKS; i++) {
    if (!tracks[i].active) return i;
  }
  return -1;
}

void assignDetectionsToTracks(Detection dets[RADAR_MAX_TARGETS], int detCount, uint32_t now) {
  float cost[MAX_TRACKS][RADAR_MAX_TARGETS];
  bool trackUsable[MAX_TRACKS];
  bool detUsable[RADAR_MAX_TARGETS];
  int bestAssign[MAX_TRACKS];
  bool detAssigned[RADAR_MAX_TARGETS] = {false, false};

  for (int ti = 0; ti < MAX_TRACKS; ti++) {
    int id = ti + 1;
    trackUsable[ti] = tracks[id].active;
    for (int dj = 0; dj < RADAR_MAX_TARGETS; dj++) {
      cost[ti][dj] = COST_UNMATCHED;
    }
  }

  for (int dj = 0; dj < RADAR_MAX_TARGETS; dj++) {
    detUsable[dj] = (dj < detCount && dets[dj].valid);
  }

  for (int ti = 0; ti < MAX_TRACKS; ti++) {
    int id = ti + 1;
    if (!tracks[id].active) continue;

    for (int dj = 0; dj < detCount; dj++) {
      if (!dets[dj].valid) continue;
      cost[ti][dj] = computeMatchCost(tracks[id], dets[dj]);
    }
  }

  solveAssignmentBruteforce(cost, trackUsable, detUsable, detCount, bestAssign);

  for (int ti = 0; ti < MAX_TRACKS; ti++) {
    int id = ti + 1;
    if (!tracks[id].active) continue;

    int dj = bestAssign[ti];
    if (dj >= 0 && dj < detCount && cost[ti][dj] < COST_UNMATCHED) {
      updateTrackWithDetection(tracks[id], dets[dj], now);
      detAssigned[dj] = true;
    }
  }

  for (int dj = 0; dj < detCount; dj++) {
    if (!dets[dj].valid || detAssigned[dj]) continue;

    int freeId = getFreeTrackId();
    if (freeId != -1) {
      initTrackFromDetection(tracks[freeId], dets[dj], now, freeId);
    }
  }
}

void updateUnmatchedTracks(uint32_t now) {
  for (int i = 1; i <= MAX_TRACKS; i++) {
    if (!tracks[i].active) continue;

    if (!tracks[i].matched_in_frame) {
      tracks[i].miss_count++;

      uint32_t lostMs = now - tracks[i].last_seen;
      if (lostMs > TRACK_REMOVE_MS) {
        tracks[i] = Track();
        ts[i] = TargetState();
      }
    }
  }
}

void publishTracks() {
  StaticJsonDocument<768> doc;
  JsonArray arr = doc.to<JsonArray>();
  uint32_t now = millis();

  for (int i = 1; i <= MAX_TRACKS; i++) {
    if (!tracks[i].active) continue;

    uint32_t lostMs = now - tracks[i].last_seen;
    bool isGhost = lostMs > 400 && lostMs <= TRACK_HOLD_MS;

    if (!isGhost) {
      processTarget((int16_t)tracks[i].x, (int16_t)tracks[i].y, tracks[i].raw_speed, i, arr);
    } else {
      JsonObject obj = arr.createNestedObject();
      obj["id"] = i;
      obj["x"] = (int)tracks[i].x;
      obj["y"] = (int)tracks[i].y;
      obj["spd"] = (int)abs(tracks[i].raw_speed);
      obj["dist"] = 0;
      obj["a"] = "STILL";
      obj["ghost"] = true;
    }
  }

  String out;
  serializeJson(doc, out);
  ws.textAll(out);
}

// ================= HANDLE RADAR =================
void handleRadar() {
  static uint8_t buf[30];
  static uint8_t idx = 0;

  while (RADAR_SERIAL.available()) {
    uint8_t b = RADAR_SERIAL.read();

    if (idx == 0 && b != 0xAA) continue;
    buf[idx++] = b;

    if (idx == 30) {
      if (buf[1] == 0xFF && buf[28] == 0x55) {
        Detection dets[RADAR_MAX_TARGETS];
        int detCount = 0;
        uint32_t now = millis();

        decodeRadarFrame(buf, dets, detCount);
        predictTracks(now);
        assignDetectionsToTracks(dets, detCount, now);
        updateUnmatchedTracks(now);
        publishTracks();
      }
      idx = 0;
    }
  }
}

// ================= WEBSOCKET =================
void onWsEvent(AsyncWebSocket *s, AsyncWebSocketClient *c, AwsEventType t, void *arg, uint8_t *d, size_t l) {
  if (t == WS_EVT_DATA) {
    String m = "";
    for (size_t i = 0; i < l; i++) m += (char)d[i];

    if (m.startsWith("ADD_ZONE:")) {
      if (zoneCount < MAX_ZONES) {
        int16_t x1, y1, x2, y2;
        sscanf(m.substring(9).c_str(), "%hd,%hd,%hd,%hd", &x1, &y1, &x2, &y2);
        safeZones[zoneCount] = {x1, y1, x2, y2, true};
        zoneCount++;
      }
    } else if (m == "CLEAR_ALL") {
      for (int i = 0; i < MAX_ZONES; i++) safeZones[i].active = false;
      zoneCount = 0;
    }
  }
}

// ================= SETUP / LOOP =================
void setup() {
  Serial.begin(115200);
  RADAR_SERIAL.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *r) {
    r->send_P(200, "text/html", index_html);
  });

  server.begin();
  Serial.println("Server started");
}

void loop() {
  handleRadar();
  ws.cleanupClients();
}
