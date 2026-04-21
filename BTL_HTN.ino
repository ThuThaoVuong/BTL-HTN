#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// ================= CẤU HÌNH WIFI =================
const char* ssid     = "PTIT_WIFI_KTX";
const char* password = "PTIT@33daimo";

// ================= CẤU HÌNH RADAR LD2450 =================
#define RADAR_SERIAL    Serial2
#define RADAR_BAUD      115200
#define RADAR_RX_PIN    16
#define RADAR_TX_PIN    17

// ================= GIỚI HẠN TẦM QUÉT MẶT TRƯỚC LD2450 =================
// Chỉ nhận mục tiêu nằm phía TRƯỚC radar (Y > 0)
// Giới hạn góc ±60° => tại Y=6000mm thì X tối đa ≈ ±3464mm
// Có thể chỉnh MIN_Y để bỏ vùng quá gần (nhiễu)
#define MIN_Y       200    // mm – bỏ qua mục tiêu quá sát (< 20cm)
#define MAX_Y       6000   // mm – tầm tối đa phía trước (6m)
#define MAX_X_ABS   3500   // mm – giới hạn ngang (±3.5m)

// ================= FREERTOS: QUEUE & MUTEX =================
// Queue truyền dữ liệu thô từ Task_Radar -> Task_Logic
struct RadarRaw {
  int16_t x, y, speed;
};
QueueHandle_t xRadarQueue;   // Queue dữ liệu thô

// Mutex bảo vệ biến kết quả (Task_Logic ghi – Task_WebServer đọc)
SemaphoreHandle_t xResultMutex;

// ================= DỮ LIỆU KẾT QUẢ CHIA SẺ GIỮA TASK =================
struct TargetResult {
  int16_t  x, y;
  int      speed;
  int      dist;
  String   action;
  bool     valid;          // true = đang có mục tiêu
} sharedResult;

// ================= WEB SERVER & WEBSOCKET =================
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ================= VÙNG AN TOÀN =================
#define MAX_ZONES 5
struct Zone {
  int16_t x_min, y_min, x_max, y_max;
  bool active = false;
} safeZones[MAX_ZONES];
int zoneCount = 0;
SemaphoreHandle_t xZoneMutex;  // Bảo vệ safeZones khi WebSocket callback ghi

// ================= TRẠNG THÁI THEO DÕI DUY NHẤT 1 NGƯỜI =================
struct TargetState {
  int16_t  x_history[10];
  int16_t  y_history[10];
  uint8_t  h_idx;
  bool     potential_fall;
  uint32_t fall_timer;
  float    smooth_doppler;
  String   action;
  uint32_t last_seen;      // millis() lần cuối thấy mục tiêu
} ts1;                     // Chỉ 1 target

// ============================================================
//                     GIAO DIỆN WEB
// ============================================================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head><meta charset="UTF-8"><title>Radar Safe Guard</title>
<style>
  :root { --bg:#f0f2f5; --card:#ffffff; --text:#333; --safe:#28a745; --danger:#dc3545; }
  body { background:var(--bg); color:var(--text); font-family:'Segoe UI',sans-serif; text-align:center; margin:0; padding:10px; }
  .panel { background:var(--card); max-width:500px; margin:10px auto; padding:15px; border-radius:12px; box-shadow:0 4px 10px rgba(0,0,0,0.1); }
  canvas { background:#fafafa; border:1px solid #ddd; border-radius:8px; cursor:crosshair; max-width:100%; }
  .btn { padding:10px 15px; margin:5px; cursor:pointer; background:#007bff; color:white; border:none; border-radius:5px; font-weight:bold; }
  .btn.active { background:#ffc107; color:black; }
  .card { background:var(--card); padding:15px; border-radius:8px; border-left:5px solid #ccc; text-align:left; font-size:14px; max-width:300px; margin:10px auto; }
  .card.active { border-left-color:var(--safe); }
  .card.danger { border-left-color:var(--danger); background:#fff5f5; animation:blink 1s infinite; }
  .card.empty  { border-left-color:#ccc; }
  @keyframes blink { 50%{ opacity:0.7; } }
  .t-name { font-weight:bold; font-size:16px; border-bottom:1px solid #eee; margin-bottom:8px; }
  .zone-info { font-size:12px; color:#666; margin-top:6px; }
</style></head><body>

<div class="panel">
  <h3>HỆ THỐNG GIÁM SÁT AN TOÀN – 1 NGƯỜI</h3>
  <button class="btn" id="bz" onclick="startDraw()">Vẽ Vùng An Toàn</button>
  <button class="btn" style="background:#6c757d" onclick="clearZone()">Xóa Vùng</button>
  <br><small id="ws-status" style="font-size:12px;color:#ffc107;">🟡 Đang kết nối…</small>
  <br><canvas id="rd" width="400" height="400"></canvas>
  <div class="zone-info" id="zi">Chưa có vùng an toàn</div>
</div>

<div id="card-1" class="card empty">
  <div class="t-name">👤 Người theo dõi</div>
  <div id="s-1">Không phát hiện</div>
</div>

<script>
let cv = document.getElementById("rd"), ctx = cv.getContext("2d");
let drawing=false, isDragging=false, startX, startY, currentX, currentY;
let zones = [];
let lastTarget = null;

// ── AUTO-RECONNECT WEBSOCKET ──────────────────────────────────
let socket = null;
let reconnectTimer = null;
let reconnectDelay = 2000;   // bắt đầu retry sau 2s
const MAX_DELAY     = 15000; // tối đa 15s giữa các lần retry

function wsConnect() {
  if (socket && (socket.readyState === WebSocket.OPEN ||
                 socket.readyState === WebSocket.CONNECTING)) return;

  socket = new WebSocket(`ws://${location.host}/ws`);
  setStatus("connecting");

  socket.onopen = () => {
    setStatus("online");
    reconnectDelay = 2000; // reset backoff khi kết nối thành công
    clearTimeout(reconnectTimer);
  };

  socket.onmessage = (e) => {
    try {
      let data = JSON.parse(e.data);
      lastTarget = data.valid ? data : null;
      render();
      updateUI(data);
    } catch(_) {}
  };

  socket.onclose = () => {
    setStatus("offline");
    scheduleReconnect();
  };

  socket.onerror = () => {
    socket.close(); // kích hoạt onclose để retry
  };
}

function scheduleReconnect() {
  clearTimeout(reconnectTimer);
  reconnectTimer = setTimeout(() => {
    wsConnect();
    reconnectDelay = Math.min(reconnectDelay * 1.5, MAX_DELAY); // exponential backoff
  }, reconnectDelay);
}

// Wrapper an toàn cho socket.send (tránh gửi khi chưa kết nối)
function wsSend(msg) {
  if (socket && socket.readyState === WebSocket.OPEN) socket.send(msg);
}

// Hiển thị trạng thái kết nối lên UI
function setStatus(state) {
  const el = document.getElementById("ws-status");
  if (!el) return;
  const map = {
    online:     { text:"🟢 Đã kết nối",   color:"#28a745" },
    offline:    { text:"🔴 Mất kết nối – đang thử lại…", color:"#dc3545" },
    connecting: { text:"🟡 Đang kết nối…", color:"#ffc107" },
  };
  const s = map[state] || map.offline;
  el.innerText  = s.text;
  el.style.color = s.color;
  if (state === "offline") {
    // Reset UI khi mất kết nối
    lastTarget = null;
    render();
    updateUI({ valid: false });
  }
}

// Khởi động lần đầu
wsConnect();

function startDraw(){ drawing=true; document.getElementById("bz").classList.add("active"); }

function clearZone(){
  zones=[];
  wsSend("CLEAR_ALL");
  document.getElementById("zi").innerText="Chưa có vùng an toàn";
  render();
}

cv.onmousedown=(e)=>{ if(!drawing) return; startX=e.offsetX; startY=e.offsetY; isDragging=true; };
cv.onmousemove=(e)=>{ if(isDragging){ currentX=e.offsetX; currentY=e.offsetY; render(); } };
cv.onmouseup=(e)=>{
  if(!isDragging) return;
  isDragging=false;
  let x1=(startX-200)*30, y1=(200-startY)*30;
  let x2=(e.offsetX-200)*30, y2=(200-e.offsetY)*30;
  // Chỉ vẽ vùng ở phía trước (y > 0)
  y1=Math.max(y1,0); y2=Math.max(y2,0);
  let z={x_min:Math.min(x1,x2), y_min:Math.min(y1,y2), x_max:Math.max(x1,x2), y_max:Math.max(y1,y2)};
  zones.push(z);
  wsSend(`ADD_ZONE:${z.x_min},${z.y_min},${z.x_max},${z.y_max}`);
  document.getElementById("zi").innerText=`${zones.length} vùng an toàn đang hoạt động`;
  drawing=false;
  document.getElementById("bz").classList.remove("active");
  render();
};

function updateUI(t){
  let card=document.getElementById("card-1");
  let status=document.getElementById("s-1");
  if(!t.valid){
    status.innerText="Không phát hiện";
    card.className="card empty";
    return;
  }
  status.innerHTML=`<b>${t.a}</b><br>X: ${t.x} mm &nbsp; Y: ${t.y} mm<br><small>Tốc độ: ${t.spd} mm/s &nbsp; Di chuyển: ${t.dist} mm</small>`;
  if(t.a.includes("FALL")||t.a.includes("IMMOBILE")) card.className="card danger";
  else if(t.a.includes("FALLING")) card.className="card danger";
  else card.className="card active";
}

function render(){
  ctx.clearRect(0,0,400,400);

  // --- LƯỚI ---
  ctx.strokeStyle="#eee"; ctx.lineWidth=1;
  for(let i=0;i<=400;i+=50){
    ctx.beginPath(); ctx.moveTo(i,0); ctx.lineTo(i,400); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(0,i); ctx.lineTo(400,i); ctx.stroke();
  }

  // --- VÙNG PHÍA SAU (mờ – không theo dõi) ---
  ctx.fillStyle="rgba(200,200,200,0.25)";
  ctx.fillRect(0,200,400,200); // y < 0 trên canvas = nửa dưới
  ctx.fillStyle="#999"; ctx.font="11px Arial"; ctx.textAlign="center";
  ctx.fillText("PHÍA SAU – KHÔNG THEO DÕI", 200, 370);

  // --- NHÃN HƯỚNG ---
  ctx.fillStyle="#ffaaaa"; ctx.font="bold 13px Arial"; ctx.textAlign="left";
  ctx.fillText("← TRÁI", 10, 210);
  ctx.textAlign="right"; ctx.fillText("PHẢI →", 390, 210);
  ctx.textAlign="center"; ctx.fillStyle="#aaa";
  ctx.fillText("↑ PHÍA TRƯỚC ↑", 200, 20);

  // --- RADAR ---
  ctx.fillStyle="#007bff"; ctx.beginPath(); ctx.arc(200,200,12,0,7); ctx.fill();
  ctx.strokeStyle="#007bff"; ctx.beginPath(); ctx.moveTo(200,200); ctx.lineTo(172,172); ctx.lineTo(228,172); ctx.closePath(); ctx.stroke();

  // --- VÙNG AN TOÀN ---
  zones.forEach((z,i)=>{
    ctx.fillStyle="rgba(40,167,69,0.15)"; ctx.strokeStyle="#28a745"; ctx.lineWidth=2;
    let rx=200+z.x_min/30, ry=200-z.y_max/30;
    let rw=(z.x_max-z.x_min)/30, rh=(z.y_max-z.y_min)/30;
    ctx.fillRect(rx,ry,rw,rh); ctx.strokeRect(rx,ry,rw,rh);
    ctx.fillStyle="#28a745"; ctx.font="10px Arial"; ctx.textAlign="left";
    ctx.fillText("Vùng "+(i+1), rx+4, ry+12);
  });

  // Preview khi kéo
  if(isDragging){
    ctx.strokeStyle="#007bff"; ctx.setLineDash([5,5]); ctx.lineWidth=1;
    ctx.strokeRect(startX,startY,currentX-startX,currentY-startY);
    ctx.setLineDash([]);
  }

  // --- VẼ NGƯỜI (chỉ 1) ---
  if(lastTarget && lastTarget.valid){
    let px=200+lastTarget.x/30, py=200-lastTarget.y/30;
    // Clamp trong canvas
    px=Math.max(6,Math.min(394,px)); py=Math.max(6,Math.min(394,py));
    let isDanger=lastTarget.a.includes("FALL")||lastTarget.a.includes("IMMOBILE");
    ctx.fillStyle=isDanger?"red":"#28a745";
    ctx.beginPath(); ctx.arc(px,py,10,0,7); ctx.fill();
    ctx.fillStyle="black"; ctx.font="bold 11px Arial"; ctx.textAlign="left";
    ctx.fillText(lastTarget.a, px+14, py-8);
  }
}
</script></body></html>
)rawliteral";

// ============================================================
//            HÀM TIỆN ÍCH DÙNG CHUNG
// ============================================================
bool checkIfInAnySafeZone(int16_t x, int16_t y) {
  // Gọi bên trong Task_Logic – đã nắm xZoneMutex trước khi gọi
  for (int i = 0; i < MAX_ZONES; i++) {
    if (safeZones[i].active &&
        x >= safeZones[i].x_min && x <= safeZones[i].x_max &&
        y >= safeZones[i].y_min && y <= safeZones[i].y_max) {
      return true;
    }
  }
  return false;
}

// ============================================================
//   TASK 1 – Task_Radar  (Core 0, Priority 3) – ĐỌC UART
// ============================================================
void Task_Radar(void* pvParam) {
  static uint8_t buf[30];
  static uint8_t idx = 0;

  for (;;) {
    while (RADAR_SERIAL.available()) {
      uint8_t b = RADAR_SERIAL.read();

      // Đồng bộ đầu frame
      if (idx == 0 && b != 0xAA) continue;
      buf[idx++] = b;

      if (idx == 30) {
        idx = 0;
        // Kiểm tra frame hợp lệ: byte[1]=0xFF, byte[28]=0x55
        if (buf[1] != 0xFF || buf[28] != 0x55) continue;

        // LD2450 trả 3 slot target; chỉ lấy slot đầu tiên khác (0,0)
        // Nếu slot đầu trống thì kiểm tra slot tiếp theo
        for (int j = 0; j < 3; j++) {
          int off = 4 + j * 8;
          int16_t rx = (buf[off+1] & 0x80)
                       ? -((int16_t)(buf[off+1] & 0x7F) << 8 | buf[off])
                       :  ((int16_t)(buf[off+1] & 0x7F) << 8 | buf[off]);
          int16_t ry = (buf[off+3] & 0x80)
                       ? -((int16_t)(buf[off+3] & 0x7F) << 8 | buf[off+2])
                       :  ((int16_t)(buf[off+3] & 0x7F) << 8 | buf[off+2]);
          int16_t rs = (buf[off+5] & 0x80)
                       ? -((int16_t)(buf[off+5] & 0x7F) << 8 | buf[off+4])
                       :  ((int16_t)(buf[off+5] & 0x7F) << 8 | buf[off+4]);

          if (rx == 0 && ry == 0) continue; // slot trống

          // ── GIỚI HẠN TẦM QUÉT MẶT TRƯỚC ──
          // LD2450 trả Y âm = phía trước nên cần đảo dấu Y (giống processTarget cũ)
          // Sau đảo: phía trước = Y dương
          int16_t fy = -ry;  // đảo chiều như code gốc
          if (fy < MIN_Y || fy > MAX_Y)       continue; // ngoài dải phía trước
          if (abs(rx) > MAX_X_ABS)            continue; // quá rộng

          // Đẩy vào queue (non-blocking; nếu đầy bỏ qua frame này)
          RadarRaw raw = { rx, ry, rs };
          xQueueSend(xRadarQueue, &raw, 0);
          break; // chỉ lấy 1 mục tiêu/frame
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5)); // yield nhỏ tránh watchdog
  }
}

// ============================================================
//   TASK 2 – Task_Logic  (Core 0, Priority 2) – FSM PHÁT HIỆN NGÃ
// ============================================================
void Task_Logic(void* pvParam) {
  RadarRaw raw;

  for (;;) {
    // Chờ dữ liệu từ Task_Radar (block tối đa 200ms)
    if (xQueueReceive(xRadarQueue, &raw, pdMS_TO_TICKS(200)) == pdTRUE) {

      uint32_t now = millis();
      int16_t x = raw.x;
      int16_t y = -raw.y; // đảo chiều: phía trước = Y dương
      int16_t s = raw.speed;

      // --- Lịch sử vị trí ---
      int16_t x_old = ts1.x_history[ts1.h_idx];
      int16_t y_old = ts1.y_history[ts1.h_idx];
      ts1.x_history[ts1.h_idx] = x;
      ts1.y_history[ts1.h_idx] = y;
      ts1.h_idx = (ts1.h_idx + 1) % 10;

      float distMoved  = sqrtf(powf(x - x_old, 2) + powf(y - y_old, 2));
      float vY         = (float)(y - y_old);
      ts1.smooth_doppler = ts1.smooth_doppler * 0.7f + fabsf(s) * 0.3f;
      ts1.last_seen    = now;

      // --- Kiểm tra vùng an toàn (lấy mutex để đọc an toàn) ---
      bool inSafe = false;
      if (xSemaphoreTake(xZoneMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        inSafe = checkIfInAnySafeZone(x, y);
        xSemaphoreGive(xZoneMutex);
      }

      // --- FSM ---
      if (!ts1.potential_fall && !inSafe) {
        if (vY < -250 && (ts1.smooth_doppler > 100 || distMoved > 250)) {
          ts1.potential_fall = true;
          ts1.fall_timer     = now;
        }
      }

      if (ts1.potential_fall) {
        uint32_t elapsed = now - ts1.fall_timer;
        if (elapsed > 1000 && (ts1.smooth_doppler > 150 || distMoved > 350 || inSafe)) {
          ts1.potential_fall = false;
          ts1.action = "DI CHUYỂN";
        } else if (elapsed > 10000) {
          ts1.action = "IMMOBILE";
        } else if (elapsed > 2500) {
          ts1.action = (ts1.smooth_doppler < 80 && distMoved < 200) ? "FALL" : "WAITING";
        } else {
          ts1.action = "FALLING ???";
        }
      } else {
        ts1.action = (ts1.smooth_doppler > 35 || distMoved > 120) ? "MOVING" : "STILL";
      }

      // --- Ghi kết quả dùng chung ---
      if (xSemaphoreTake(xResultMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        sharedResult = { x, y, (int)ts1.smooth_doppler, (int)distMoved, ts1.action, true };
        xSemaphoreGive(xResultMutex);
      }

    } else {
      // Không nhận được dữ liệu trong 200ms → mục tiêu mất
      if (xSemaphoreTake(xResultMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        sharedResult.valid = false;
        xSemaphoreGive(xResultMutex);
      }
      // Reset FSM nếu mất dữ liệu lâu
      if (millis() - ts1.last_seen > 3000) {
        ts1.potential_fall = false;
        ts1.smooth_doppler = 0;
      }
    }
  }
}

// ============================================================
//   TASK 3 – Task_WebServer  (Core 1) – WiFi + HTTP + WS
// ============================================================

// Callback WebSocket (chạy trên Core 1 từ ESPAsync nội bộ)
void onWsEvent(AsyncWebSocket* s, AsyncWebSocketClient* c,
               AwsEventType t, void* arg, uint8_t* d, size_t l) {
  if (t != WS_EVT_DATA) return;
  String m = "";
  for (size_t i = 0; i < l; i++) m += (char)d[i];

  if (m.startsWith("ADD_ZONE:")) {
    if (xSemaphoreTake(xZoneMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      if (zoneCount < MAX_ZONES) {
        int16_t x1, y1, x2, y2;
        sscanf(m.substring(9).c_str(), "%hd,%hd,%hd,%hd", &x1, &y1, &x2, &y2);
        safeZones[zoneCount++] = { x1, y1, x2, y2, true };
        Serial.printf("Đã thêm vùng an toàn #%d\n", zoneCount);
      }
      xSemaphoreGive(xZoneMutex);
    }
  } else if (m == "CLEAR_ALL") {
    if (xSemaphoreTake(xZoneMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      for (int i = 0; i < MAX_ZONES; i++) safeZones[i].active = false;
      zoneCount = 0;
      xSemaphoreGive(xZoneMutex);
    }
    Serial.println("Đã xóa tất cả vùng an toàn");
  }
}

void Task_WebServer(void* pvParam) {
  // Khởi tạo WiFi
  WiFi.begin(ssid, password);
  Serial.print("Đang kết nối WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.print(".");
  }
  Serial.print("\nIP: "); Serial.println(WiFi.localIP());

  // Khởi tạo WebSocket & HTTP
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* r) {
    r->send_P(200, "text/html", index_html);
  });
  server.begin();
  Serial.println("Web server đã khởi động");

  // Vòng lặp: đọc kết quả và gửi WS mỗi 100ms
  for (;;) {
    if (ws.count() > 0) { // chỉ gửi khi có client kết nối
      TargetResult res;

      if (xSemaphoreTake(xResultMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        res = sharedResult;
        xSemaphoreGive(xResultMutex);
      }

      StaticJsonDocument<256> doc;
      doc["valid"] = res.valid;
      if (res.valid) {
        doc["x"]    = res.x;
        doc["y"]    = res.y;
        doc["spd"]  = res.speed;
        doc["dist"] = res.dist;
        doc["a"]    = res.action;
      }
      String out;
      serializeJson(doc, out);
      ws.textAll(out);
    }

    ws.cleanupClients();
    vTaskDelay(pdMS_TO_TICKS(100)); // gửi 10 lần/giây
  }
}

// ============================================================
//                         SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  RADAR_SERIAL.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);

  // Khởi tạo FreeRTOS objects
  xRadarQueue   = xQueueCreate(20, sizeof(RadarRaw));
  xResultMutex  = xSemaphoreCreateMutex();
  xZoneMutex    = xSemaphoreCreateMutex();

  // Khởi tạo trạng thái
  memset(&ts1,          0, sizeof(ts1));
  memset(&sharedResult, 0, sizeof(sharedResult));
  sharedResult.valid = false;

  // ── Tạo Task ──
  //                  func           name            stack   param  prio  handle  core
  xTaskCreatePinnedToCore(Task_Radar,     "Task_Radar",     4096, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(Task_Logic,     "Task_Logic",     8192, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(Task_WebServer, "Task_WebServer", 8192, NULL, 1, NULL, 1);

  Serial.println("Tất cả task đã được khởi tạo");
}

// loop() không làm gì – toàn bộ logic đã chuyển vào FreeRTOS tasks
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
