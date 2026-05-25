#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <esp_task_wdt.h> // [THÊM] Thư viện Hardware Watchdog Timer của ESP32
// Thư viện quản lý năng lượng của ESP32
#include "esp_pm.h"
#include "esp_wifi.h"

// ================= CẤU HÌNH WIFI =================
const char* ssid     = "WIFI";
const char* password = "abc";

// ================= CẤU HÌNH RADAR LD2450 =================
#define RADAR_SERIAL    Serial2
#define RADAR_BAUD      115200
#define RADAR_RX_PIN    16
#define RADAR_TX_PIN    17

// ================= CẤU HÌNH ĐÈN LED VÀ NÚT BẤM CỨNG =================
#define LED_BUILTIN_PIN  2    // Đèn LED nhỏ màu xanh có sẵn trên board ESP32
#define BUTTON_BOOT_PIN  0    // Nút nhấn BOOT mặc định trên mạch ESP32

// ================= GIỚI HẠN TẦM QUÉT MẶT TRƯỚC LD2450 =================
#define MIN_Y       200    // mm – bỏ qua mục tiêu quá sát (< 20cm)
#define MAX_Y       6000   // mm – tầm tối đa phía trước (6m)
#define MAX_X_ABS   3500   // mm – giới hạn ngang (±3.5m)

// ================= CẤU HÌNH HỆ THỐNG ĐA PHÒNG GIA LẬP =================
#define NUM_ROOMS   3
const char* roomNames[NUM_ROOMS] = {"Phòng Khách", "Phòng Ngủ", "Phòng Tắm"};
uint8_t currentSelectedRoom = 0; // Phòng hiện tại hệ thống đang chọn theo dõi

// ================= CẤU HÌNH WATCHDOG & DIAGNOSTICS =================
#define WDT_TIMEOUT_SECONDS   4   // Reboot chip nếu bất kỳ Task nào bị kẹt quá 4 giây
uint32_t lastRadarPacketTime = 0; // Lưu mốc thời gian cuối cùng nhận được byte từ Radar
bool isSensorConnected = true;    // Trạng thái kết nối của cảm biến LD2450

// ================= CẤU HÌNH TIẾT KIỆM NĂNG LƯỢNG (ECO) =================
#define ECO_TIMEOUT_MS        20000 // 20s (20 * 1000 ms) phòng trống để vào Eco mode
uint32_t lastTargetDetectedTime = 0; // Mốc thời gian cuối cùng phát hiện có người
bool isEcoModeActive = false;         // Trạng thái chế độ tiết kiệm điện hiện tại

// ================= FREERTOS: QUEUE & MUTEX =================
struct RadarRaw {
  int16_t x, y, speed;
};
QueueHandle_t xRadarQueue;   
SemaphoreHandle_t xResultMutex;

// ================= DỮ LIỆU KẾT QUẢ CHIA SẺ GIỮA TASK =================
struct TargetResult {
  int16_t  x, y;
  int      speed;
  int      dist;
  String   action;
  bool     valid;          
} sharedResults[NUM_ROOMS];  // Mảng lưu kết quả độc lập cho từng phòng

// ================= VÙNG AN TOÀN =================
#define MAX_ZONES 5
struct Zone {
  int16_t x_min, y_min, x_max, y_max;
  bool active = false;
};
Zone safeZones[NUM_ROOMS][MAX_ZONES]; // Vùng an toàn riêng biệt cho từng phòng
int zoneCount[NUM_ROOMS] = {0, 0, 0};
SemaphoreHandle_t xZoneMutex;  

// ================= TRẠNG THÁI THEO DÕI NGƯỜI CỦA TỪNG PHÒNG =================
struct TargetState {
  int16_t  x_history[10];
  int16_t  y_history[10];
  uint8_t  h_idx;
  bool     potential_fall;
  uint32_t fall_timer;
  float    smooth_doppler;
  String   action;
  uint32_t last_seen;      
} tsRooms[NUM_ROOMS]; // Mỗi phòng giữ một FSM trạng thái riêng biệt

// Các biến phục vụ việc sinh dữ liệu giả cho Phòng 1 và Phòng 2
int16_t simX[NUM_ROOMS] = {0, -1000, 1200};
int16_t simY[NUM_ROOMS] = {0, 2500, 3000};
int16_t simDirX[NUM_ROOMS] = {0, 40, -50};
int16_t simDirY[NUM_ROOMS] = {0, 60, -30};

// ================= WEB SERVER & WEBSOCKET =================
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

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
  
  .room-selector { display: flex; justify-content: space-around; margin-bottom: 15px; }
  .btn-room { padding: 10px; flex: 1; margin: 0 4px; border: 1px solid #ccc; background: #e9ecef; cursor: pointer; border-radius: 6px; font-weight: bold; }
  .btn-room.selected { background: #007bff; color: white; border-color: #007bff; }

  .card { background:var(--card); padding:15px; border-radius:8px; border-left:5px solid #ccc; text-align:left; font-size:14px; max-width:300px; margin:10px auto; }
  .card.active { border-left-color:var(--safe); }
  .card.danger { border-left-color:var(--danger); background:#fff5f5; animation:blink 1s infinite; }
  .card.empty  { border-left-color:#ccc; }
  @keyframes blink { 50%{ opacity:0.7; } }
  .t-name { font-weight:bold; font-size:16px; border-bottom:1px solid #eee; margin-bottom:8px; }
  .zone-info { font-size:12px; color:#666; margin-top:6px; }

  #alert-overlay { display:none; position:fixed; bottom:20px; right:20px; z-index:999; background:none; }
  #alert-overlay.show { display:block; }
  #alert-box { background:#fff; border-radius:12px; padding:16px 18px; width:260px; text-align:left; box-shadow:0 6px 20px rgba(0,0,0,0.2); border-left:6px solid #dc3545; }
  #alert-icon { font-size:32px; margin-bottom:6px; }
  #alert-title { font-size:16px; font-weight:bold; color:#dc3545; margin-bottom:4px; }
  #alert-msg { font-size:13px; color:#555; margin-bottom:6px; }
  #alert-time { font-size:11px; color:#999; margin-bottom:10px; }
  #alert-ok { padding:6px 14px; background:#dc3545; color:#fff; border:none; border-radius:6px; font-size:13px; cursor:pointer; }
  #alert-ok:hover { background:#b02a37; }
  #alert-overlay.show #alert-box { animation:slideIn 0.3s ease; }
  @keyframes slideIn { from { transform:translateY(20px); opacity:0; } to { transform:translateY(0); opacity:1; } }
  .sys-status-bar { display: flex; justify-content: space-between; font-size: 12px; padding: 6px 10px; background: #e9ecef; border-radius: 6px; margin-bottom: 10px; font-weight: bold; }
  .status-ok { color: #28a745; }
  .status-err { color: #dc3545; animation: blink 1s infinite; }
</style></head><body>

<div class="panel">
  <h3>HỆ THỐNG GIÁM SÁT AN TOÀN TRỰC TUYẾN</h3>

  <!-- [THÊM] Thanh giám sát phần cứng và tài nguyên hệ thống -->
  <div class="sys-status-bar">
    <span>Phần cứng: <span id="hw-status" class="status-ok">🟢 ĐỒNG BỘ</span></span>
    <!-- [THÊM DÒNG DƯỚI] -->
    <span>Chế độ: <span id="pwr-status" style="color:#28a745;">⚡ HIGH PERF</span></span>
    <span>Bộ nhớ RAM: <span id="hw-heap" style="color:#007bff;">-- KB</span></span>
  </div>
  
  <div class="room-selector">
    <button class="btn-room selected" id="btn-r0" onclick="selectRoom(0)">Phòng Khách</button>
    <button class="btn-room" id="btn-r1" onclick="selectRoom(1)">Phòng Ngủ (Sim)</button>
    <button class="btn-room" id="btn-r2" onclick="selectRoom(2)">Phòng Tắm (Sim)</button>
  </div>

  <button class="btn" style="background:#28a745; width:100%; margin-bottom:15px; font-size:15px;" onclick="nextRoomWeb()">🔄 Đổi Phòng (Kế Tiếp)</button>

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

<div id="alert-overlay">
  <div id="alert-box">
    <div id="alert-icon">🚨</div>
    <div id="alert-title">CẢNH BÁO</div>
    <div id="alert-msg">Phát hiện bất thường!</div>
    <div id="alert-time"></div>
    <button id="alert-ok" onclick="dismissAlert()">Đã biết</button>
  </div>
</div>

<script>
let cv = document.getElementById("rd"), ctx = cv.getContext("2d");
let drawing=false, isDragging=false, startX, startY, currentX, currentY;
let currentRoom = 0;
let roomZones = [[], [], []]; 
let lastTarget = null;

let socket = null;
let reconnectTimer = null;
let reconnectDelay = 2000;
const MAX_DELAY = 15000;

function wsConnect() {
  if (socket && (socket.readyState === WebSocket.OPEN || socket.readyState === WebSocket.CONNECTING)) return;
  socket = new WebSocket(`ws://${location.host}/ws`);
  setStatus("connecting");

  socket.onopen = () => {
    setStatus("online");
    reconnectDelay = 2000;
    clearTimeout(reconnectTimer);
  };

  socket.onmessage = (e) => {
    try {
      let data = JSON.parse(e.data);

      // [THÊM] Cập nhật chế độ nguồn điện động
      if (data.power_mode === "ECO") {
        document.getElementById("pwr-status").innerText = "🌱 ECO MODE (80MHz)";
        document.getElementById("pwr-status").style.color = "#198754";
      } else {
        document.getElementById("pwr-status").innerText = "⚡ HIGH PERF (240MHz)";
        document.getElementById("pwr-status").style.color = "#ffc107";
      }

      // [THÊM] Cập nhật chẩn đoán lỗi phần cứng từ ESP32 lên giao diện
      if (data.hw_status === "OK") {
        document.getElementById("hw-status").innerText = "🟢 ĐỒNG BỘ";
        document.getElementById("hw-status").className = "status-ok";
      } else {
        document.getElementById("hw-status").innerText = "🚨 LỖI PHẦN CỨNG (UART LOST)";
        document.getElementById("hw-status").className = "status-err";
      }
      if (data.free_heap) {
        document.getElementById("hw-heap").innerText = Math.round(data.free_heap / 1024) + " KB";
      }
      
      if (data.sync && data.room !== currentRoom) {
        currentRoom = data.room;
        updateRoomTabUI();
      }

      if (data.room === currentRoom) {
        lastTarget = data.valid ? data : null;
        render();
        updateUI(data);
      }
    } catch(_) {}
  };

  socket.onclose = () => { setStatus("offline"); scheduleReconnect(); };
  socket.onerror = () => { socket.close(); };
}

function scheduleReconnect() {
  clearTimeout(reconnectTimer);
  reconnectTimer = setTimeout(() => {
    wsConnect();
    reconnectDelay = Math.min(reconnectDelay * 1.5, MAX_DELAY);
  }, reconnectDelay);
}

function wsSend(msg) {
  if (socket && socket.readyState === WebSocket.OPEN) socket.send(msg);
}

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
    lastTarget = null;
    render();
    updateUI({ valid: false });
  }
}

wsConnect();

function nextRoomWeb() {
  let nextId = (currentRoom + 1) % 3;
  selectRoom(nextId);
}

function selectRoom(roomId) {
  currentRoom = roomId;
  updateRoomTabUI();
  wsSend(`SELECT_ROOM:${roomId}`);
  lastTarget = null;
  render();
  updateUI({ valid: false });
}

function updateRoomTabUI() {
  for (let i = 0; i < 3; i++) {
    let btn = document.getElementById(`btn-r${i}`);
    if (i === currentRoom) btn.classList.add("selected");
    else btn.classList.remove("selected");
  }
  let zText = roomZones[currentRoom].length > 0 ? `${roomZones[currentRoom].length} vùng an toàn đang hoạt động` : "Chưa có vùng an toàn";
  document.getElementById("zi").innerText = zText;
}

function startDraw(){ drawing=true; document.getElementById("bz").classList.add("active"); }

function clearZone(){
  roomZones[currentRoom] = [];
  wsSend(`CLEAR_ROOM:${currentRoom}`);
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
  y1=Math.max(y1,0); y2=Math.max(y2,0);
  let z={x_min:Math.min(x1,x2), y_min:Math.min(y1,y2), x_max:Math.max(x1,x2), y_max:Math.max(y1,y2)};
  
  if (roomZones[currentRoom].length < 5) {
    roomZones[currentRoom].push(z);
    wsSend(`ADD_ZONE_ROOM:${currentRoom},${z.x_min},${z.y_min},${z.x_max},${z.y_max}`);
    document.getElementById("zi").innerText=`${roomZones[currentRoom].length} vùng an toàn đang hoạt động`;
  }
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
    hideAlert();
    return;
  }
  status.innerHTML=`<b>${t.a}</b><br>X: ${t.x} mm &nbsp; Y: ${t.y} mm<br><small>Tốc độ: ${t.spd} mm/s &nbsp; Di chuyển: ${t.dist} mm</small>`;

  let isDanger = (t.a === "FALL" || t.a === "IMMOBILE");
  if(isDanger){
    card.className="card danger";
    showAlert(t.a);
  } else {
    card.className="card active";
    hideAlert();
  }
}

let alertDismissed = false;   
let lastAlertState = "";      
let alertTimerInterval = null;
let alertStartTime = null;

function showAlert(state) {
  if (state !== lastAlertState) {
    lastAlertState = state;
    alertDismissed = false;
    alertStartTime = Date.now();
    startAlertTimer();
  }
  if (alertDismissed) return;

  const overlay = document.getElementById("alert-overlay");
  const box     = document.getElementById("alert-box");
  const icon    = document.getElementById("alert-icon");
  const title   = document.getElementById("alert-title");
  const msg     = document.getElementById("alert-msg");

  if (state.includes("IMMOBILE")) {
    icon.textContent  = "🛑";
    title.textContent = "PHÁT HIỆN NGẤT";
    msg.textContent   = "Người theo dõi không di chuyển trong thời gian dài!";
  } else {
    icon.textContent  = "🚨";
    title.textContent = "PHÁT HIỆN NGÃ";
    msg.textContent   = "Người theo dõi có thể đã bị ngã!";
  }
  overlay.classList.add("show");
}

function hideAlert() {
  const overlay = document.getElementById("alert-overlay");
  overlay.classList.remove("show");
  lastAlertState = "";
  alertDismissed = false;
  stopAlertTimer();
}

function dismissAlert() {
  alertDismissed = true;
  document.getElementById("alert-overlay").classList.remove("show");
}

function startAlertTimer() {
  stopAlertTimer();
  alertTimerInterval = setInterval(() => {
    if (!alertStartTime) return;
    let secs = Math.floor((Date.now() - alertStartTime) / 1000);
    let m = String(Math.floor(secs/60)).padStart(2,"0");
    let s = String(secs % 60).padStart(2,"0");
    document.getElementById("alert-time").textContent = `Thời gian cảnh báo: ${m}:${s}`;
  }, 1000);
}

function stopAlertTimer() {
  if (alertTimerInterval) { clearInterval(alertTimerInterval); alertTimerInterval = null; }
  document.getElementById("alert-time").textContent = "";
  alertStartTime = null;
}

function render(){
  ctx.clearRect(0,0,400,400);
  ctx.strokeStyle="#eee"; ctx.lineWidth=1;
  for(let i=0;i<=400;i+=50){
    ctx.beginPath(); ctx.moveTo(i,0); ctx.lineTo(i,400); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(0,i); ctx.lineTo(400,i); ctx.stroke();
  }
  ctx.fillStyle="rgba(200,200,200,0.25)";
  ctx.fillRect(0,200,400,200);
  ctx.fillStyle="#999"; ctx.font="11px Arial"; ctx.textAlign="center";
  ctx.fillText("PHÍA SAU – KHÔNG THEO DÕI", 200, 370);

  ctx.fillStyle="#ffaaaa"; ctx.font="bold 13px Arial"; ctx.textAlign="left";
  ctx.fillText("← ", 10, 210);
  ctx.textAlign="right"; ctx.fillText(" →", 390, 210);
  ctx.textAlign="center"; ctx.fillStyle="#aaa";
  ctx.fillText("↑ PHÍA TRƯỚC ↑", 200, 20);

  ctx.fillStyle="#007bff"; ctx.beginPath(); ctx.arc(200,200,12,0,7); ctx.fill();
  ctx.strokeStyle="#007bff"; ctx.beginPath(); ctx.moveTo(200,200); ctx.lineTo(172,172); ctx.lineTo(228,172); ctx.closePath(); ctx.stroke();

  roomZones[currentRoom].forEach((z,i)=>{
    ctx.fillStyle="rgba(40,167,69,0.15)"; ctx.strokeStyle="#28a745"; ctx.lineWidth=2;
    let rx=200+z.x_min/30, ry=200-z.y_max/30;
    let rw=(z.x_max-z.x_min)/30, rh=(z.y_max-z.y_min)/30;
    ctx.fillRect(rx,ry,rw,rh); ctx.strokeRect(rx,ry,rw,rh);
    ctx.fillStyle="#28a745"; ctx.font="10px Arial"; ctx.textAlign="left";
    ctx.fillText("Vùng "+(i+1), rx+4, ry+12);
  });

  if(isDragging){
    ctx.strokeStyle="#007bff"; ctx.setLineDash([5,5]); ctx.lineWidth=1;
    ctx.strokeRect(startX,startY,currentX-startX,currentY-startY);
    ctx.setLineDash([]);
  }

  if(lastTarget && lastTarget.valid){
    let px=200+lastTarget.x/30, py=200-lastTarget.y/30;
    px=Math.max(6,Math.min(394,px)); py=Math.max(6,Math.min(394,py));
    let isDanger=lastTarget.a.includes("FALL")||lastTarget.a.includes("IMMOBILE");
    ctx.fillStyle=isDanger?"red":"#28a745";
    ctx.beginPath(); ctx.arc(px,py,10,0,7); ctx.fill();
    ctx.fillStyle="black"; ctx.font="bold 11px Arial"; ctx.textAlign="left";
    ctx.fillText(lastTarget.a, px+14, py-8);
  }
}
</script>
</body></html>
)rawliteral";

// ============================================================
//            HÀM TIỆN ÍCH KIỂM TRA VÙNG AN TOÀN ĐA PHÒNG
// ============================================================
bool checkIfInAnySafeZone(uint8_t r_id, int16_t x, int16_t y) {
  for (int i = 0; i < MAX_ZONES; i++) {
    if (safeZones[r_id][i].active &&
        x >= safeZones[r_id][i].x_min && x <= safeZones[r_id][i].x_max &&
        y >= safeZones[r_id][i].y_min && y <= safeZones[r_id][i].y_max) {
      return true;
    }
  }
  return false;
}

// ============================================================
//   TASK 1 – Task_Radar (Core 0, Priority 3) – ĐỌC UART RADAR
// ============================================================
void Task_Radar(void* pvParam) {
  esp_task_wdt_add(NULL); // [THÊM] Đăng ký Task_Radar vào Hardware Watchdog
  static uint8_t buf[30];
  static uint8_t idx = 0;

  for (;;) {
    esp_task_wdt_reset(); // [THÊM] Reset Watchdog ở mỗi chu kỳ lặp
    while (RADAR_SERIAL.available()) {
      uint8_t b = RADAR_SERIAL.read();
      if (idx == 0 && b != 0xAA) continue;
      buf[idx++] = b;

      if (idx == 30) {
        idx = 0;
        if (buf[1] != 0xFF || buf[28] != 0x55) continue;

        // [THÊM] Nhận đúng cấu trúc gói tin của LD2450 -> Cập nhật nhịp tim phần cứng ngay
        lastRadarPacketTime = millis();
        isSensorConnected = true;

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

          if (rx == 0 && ry == 0) continue; 

          int16_t fy = -ry;  
          if (fy < MIN_Y || fy > MAX_Y)       continue; 
          if (abs(rx) > MAX_X_ABS)            continue; 

          RadarRaw raw = { rx, ry, rs };
          xQueueSend(xRadarQueue, &raw, 0);
          break; 
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5)); 
  }
}

// ============================================================
//   TASK 2 – Task_Logic (Core 0, Priority 2) – FSM ĐA PHÒNG GIA LẬP
// ============================================================
void Task_Logic(void* pvParam) {
  esp_task_wdt_add(NULL); // [THÊM] Đăng ký Task_Logic vào Hardware Watchdog
  RadarRaw raw;

  for (;;) {
    esp_task_wdt_reset(); // [THÊM] Reset Watchdog
    uint8_t r_id = currentSelectedRoom; 
    uint32_t now = millis();

    // [THÊM] Chẩn đoán liên tục: Nếu quá 3000ms không có dữ liệu UART -> Cảm biến hỏng/lỏng dây
    if (now - lastRadarPacketTime > 3000) {
      isSensorConnected = false;
    }
    // Khai báo một biến cờ để kiểm tra trạng thái phòng
    bool roomHasPeople = false;

    // ─── TRƯỜNG HỢP PHÒNG 0: ĐỌC DỮ LIỆU THẬT TỪ RADAR ───
    if (r_id == 0) {
      // [THÊM TRƯỜNG HỢP LỖI PHẦN CỨNG] Nếu đang xem phòng thật mà cảm biến chết
      if (!isSensorConnected) {
        if (xSemaphoreTake(xResultMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          sharedResults[0].valid = true; 
          sharedResults[0].action = "⚠️ LỖI PHẦN CỨNG: MẤT RADAR";
          sharedResults[0].x = 0; sharedResults[0].y = 0; sharedResults[0].speed = 0;
          xSemaphoreGive(xResultMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
        continue; // Bỏ qua phân tích FSM phía dưới vì radar đã mất kết nối
      }

      if (xQueueReceive(xRadarQueue, &raw, pdMS_TO_TICKS(60)) == pdTRUE) {
        TargetState* ts = &tsRooms[0];
        int16_t x = raw.x;
        int16_t y = -raw.y; 
        int16_t s = raw.speed;

        roomHasPeople = true;
        

        int16_t x_old = ts->x_history[ts->h_idx];
        int16_t y_old = ts->y_history[ts->h_idx];
        ts->x_history[ts->h_idx] = x;
        ts->y_history[ts->h_idx] = y;
        ts->h_idx = (ts->h_idx + 1) % 10;

        float distMoved  = sqrtf(powf(x - x_old, 2) + powf(y - y_old, 2));
        float vY         = (float)(y - y_old);
        ts->smooth_doppler = ts->smooth_doppler * 0.7f + fabsf(s) * 0.3f;
        ts->last_seen    = now;

        bool inSafe = false;
        if (xSemaphoreTake(xZoneMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          inSafe = checkIfInAnySafeZone(0, x, y);
          xSemaphoreGive(xZoneMutex);
        }

        if (!ts->potential_fall && !inSafe) {
          if (vY < -250 && (ts->smooth_doppler > 100 || distMoved > 250)) {
            ts->potential_fall = true;
            ts->fall_timer     = now;
          }
        }

        if (ts->potential_fall) {
          uint32_t elapsed = now - ts->fall_timer;
          if (elapsed > 1000 && (ts->smooth_doppler > 150 || distMoved > 350 || inSafe)) {
            ts->potential_fall = false;
            ts->action = "MOVING";
          } else if (elapsed > 10000) {
            ts->action = "IMMOBILE";
          } else if (elapsed > 2500) {
            ts->action = (ts->smooth_doppler < 80 && distMoved < 200) ? "FALL" : "FALLING ???";
          } else {
            ts->action = "FALLING ???";
          }
        } else {
          ts->action = (ts->smooth_doppler > 35 || distMoved > 120) ? "MOVING" : "STILL";
        }

        if (xSemaphoreTake(xResultMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          sharedResults[0] = { x, y, (int)ts->smooth_doppler, (int)distMoved, ts->action, true };
          xSemaphoreGive(xResultMutex);
        }
      } else {
        // Mất mục tiêu thật phòng 0
        if (xSemaphoreTake(xResultMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          sharedResults[0].valid = false;
          xSemaphoreGive(xResultMutex);
        }
        if (millis() - tsRooms[0].last_seen > 3000) {
          tsRooms[0].potential_fall = false;
          tsRooms[0].smooth_doppler = 0;
        }
      }
    } 
    // ─── TRƯỜNG HỢP PHÒNG 1 HOẶC 2: TỰ SINH DỮ LIỆU GIẢ LẬP ───
    else {
      vTaskDelay(pdMS_TO_TICKS(100)); // Chu kỳ sinh dữ liệu mô phỏng 100ms
      
      // Cho điểm di chuyển tịnh tiến tuần hoàn tự động
      simX[r_id] += simDirX[r_id];
      simY[r_id] += simDirY[r_id];

      // Đảo hướng nếu chạm biên vùng quét ảo để tạo hiệu ứng di chuyển qua lại
      if (abs(simX[r_id]) > 2500) simDirX[r_id] = -simDirX[r_id];
      if (simY[r_id] < 1000 || simY[r_id] > 5000) simDirY[r_id] = -simDirY[r_id];

      TargetState* ts = &tsRooms[r_id];
      int16_t x = simX[r_id];
      int16_t y = simY[r_id];

      // Giả lập trạng thái phòng 1 khác phòng 2 (Ví dụ: Phòng 2 thi thoảng giả ngã)
      if (r_id == 2 && random(0, 150) == 25 && !ts->potential_fall) {
         ts->potential_fall = true;
         ts->fall_timer = millis();
         ts->action = "FALL";
      }

      if (ts->potential_fall) {
         if (millis() - ts->fall_timer > 6000) { // Sau 6 giây đứng dậy đi tiếp
            ts->potential_fall = false;
            ts->action = "MOVING";
         }
      } else {
         ts->action = "MOVING";
      }

      if (xSemaphoreTake(xResultMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        sharedResults[r_id] = { x, y, (int)random(40, 90), (int)random(100, 200), ts->action, true };
        xSemaphoreGive(xResultMutex);
        if (sharedResults[r_id].valid) {
          roomHasPeople = true;
        }
      }
    }

    if (roomHasPeople) {
      lastTargetDetectedTime = now; // Cập nhật mốc thời gian có người gần nhất
      
      // Nếu đang ở chế độ Eco mà có chuyển động -> ĐẨY HIỆU NĂNG LÊN NGAY (Thức giấc)
      if (isEcoModeActive) {
        setCpuFrequencyMhz(240); // Đẩy xung nhịp lên tối đa 240MHz để xử lý mượt mà
        esp_wifi_set_ps(WIFI_PS_NONE); // Tắt tiết kiệm điện WiFi để Web phản hồi tức thì
        isEcoModeActive = false;
        Serial.println("[POWER] Phát hiện chuyển động! Đã khôi phục hiệu năng cao (240MHz).");
      }
    } 
    else {
      // Nếu phòng trống liên tục vượt quá thời gian cấu hình (5 phút) -> KÍCH HOẠT CHẾ ĐỘ ECO
      if (!isEcoModeActive && (now - lastTargetDetectedTime > ECO_TIMEOUT_MS)) {
        setCpuFrequencyMhz(80); // Hạ xung nhịp xuống 80MHz (Mức tối thiểu để duy trì WiFi/UART ổn định)
        esp_wifi_set_ps(WIFI_PS_MIN_MODEM); // Bật chế độ Modem-Sleep cho bộ thu phát WiFi
        isEcoModeActive = true;
        Serial.println("[POWER] DEMO Phòng trống không có ai > 20s. Kích hoạt chế độ ECO (80MHz + Modem-Sleep) thành công.");
      }
    }

  }
  
}

// ============================================================
//   TASK 3 – Task_WebServer (Core 1) – CHỐP LED + LIÊN KẾT ĐỒNG BỘ
// ============================================================
void onWsEvent(AsyncWebSocket* s, AsyncWebSocketClient* c,
               AwsEventType t, void* arg, uint8_t* d, size_t l) {
  if (t != WS_EVT_DATA) return;
  String m = "";
  for (size_t i = 0; i < l; i++) m += (char)d[i];

  if (m.startsWith("SELECT_ROOM:")) {
    uint8_t targetRoom = m.substring(12).toInt();
    if (targetRoom < NUM_ROOMS) {
      currentSelectedRoom = targetRoom;
      Serial.printf("[WEB EVENT] Đã nhận lệnh chuyển màn hình sang: %s\n", roomNames[currentSelectedRoom]);
      
      digitalWrite(LED_BUILTIN_PIN, HIGH);
      vTaskDelay(pdMS_TO_TICKS(70));
      digitalWrite(LED_BUILTIN_PIN, LOW);
    }
  }
  else if (m.startsWith("ADD_ZONE_ROOM:")) {
    if (xSemaphoreTake(xZoneMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      uint8_t r_id;
      int16_t x1, y1, x2, y2;
      sscanf(m.substring(14).c_str(), "%hhu,%hd,%hd,%hd,%hd", &r_id, &x1, &y1, &x2, &y2);
      if (r_id < NUM_ROOMS && zoneCount[r_id] < MAX_ZONES) {
        safeZones[r_id][zoneCount[r_id]++] = { x1, y1, x2, y2, true };
        Serial.printf("Đã thêm vùng cho %s #%d\n", roomNames[r_id], zoneCount[r_id]);
      }
      xSemaphoreGive(xZoneMutex);
    }
  } else if (m.startsWith("CLEAR_ROOM:")) {
    uint8_t r_id = m.substring(11).toInt();
    if (r_id < NUM_ROOMS) {
      if (xSemaphoreTake(xZoneMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        for (int i = 0; i < MAX_ZONES; i++) safeZones[r_id][i].active = false;
        zoneCount[r_id] = 0;
        xSemaphoreGive(xZoneMutex);
      }
      Serial.printf("Đã xóa vùng an toàn của %s\n", roomNames[r_id]);
    }
  }
}

void Task_WebServer(void* pvParam) {
  WiFi.begin(ssid, password);
  Serial.print("Đang kết nối WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.print(".");
  }
  Serial.print("\nIP: "); Serial.println(WiFi.localIP());

  pinMode(LED_BUILTIN_PIN, OUTPUT);
  pinMode(BUTTON_BOOT_PIN, INPUT_PULLUP);
  digitalWrite(LED_BUILTIN_PIN, LOW); 

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* r) {
    r->send_P(200, "text/html", index_html);
  });
  server.begin();
  Serial.println("Web server đã khởi động");

  esp_task_wdt_add(NULL); // [THÊM] Đăng ký Task Web vào Hardware Watchdog

  unsigned long lastDebounceTime = 0;
  const unsigned long debounceDelay = 350; 
  uint8_t lastRoomStatus = currentSelectedRoom; 

  for (;;) {
    esp_task_wdt_reset(); // [THÊM] Reset Watchdog

    if (digitalRead(BUTTON_BOOT_PIN) == LOW) {
      if ((millis() - lastDebounceTime) > debounceDelay) {
        lastDebounceTime = millis();
        
        currentSelectedRoom = (currentSelectedRoom + 1) % NUM_ROOMS;
        Serial.printf("[NÚT BOOT ESP32] Bấm chuyển mạch sang: %s\n", roomNames[currentSelectedRoom]);

        digitalWrite(LED_BUILTIN_PIN, HIGH); 
        vTaskDelay(pdMS_TO_TICKS(70));       
        digitalWrite(LED_BUILTIN_PIN, LOW);  
      }
    }

    if (ws.count() > 0) { 
      uint8_t r_id = currentSelectedRoom; 
      TargetResult res;

      if (xSemaphoreTake(xResultMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        res = sharedResults[r_id];
        xSemaphoreGive(xResultMutex);
      }

      StaticJsonDocument<256> doc;
      doc["room"]  = r_id; 
      doc["valid"] = res.valid;

      // [THÊM] Đóng gói trạng thái chẩn đoán hệ thống thực tế vào JSON gửi đi
      doc["hw_status"] = isSensorConnected ? "OK" : "ERROR";
      // [THÊM] Gửi trạng thái chế độ nguồn điện hiện tại lên giao diện
      doc["power_mode"] = isEcoModeActive ? "ECO" : "HIGH";
      doc["free_heap"] = ESP.getFreeHeap(); // Trả về số byte RAM còn trống thực tế của vi xử lý
      
      if (r_id != lastRoomStatus) {
        doc["sync"] = true;
        lastRoomStatus = r_id;
      } else {
        doc["sync"] = false;
      }

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
    vTaskDelay(pdMS_TO_TICKS(50)); 
  }
}

// ============================================================
//                                SETUP
// ============================================================
void setup() {
  // [THÊM] Cấu hình và kích hoạt lõi Hardware Task Watchdog Timer cho hệ thống
  // [SỬA TẠI ĐÂY]: Khởi tạo Watchdog phần cứng theo chuẩn ESP32 Core v3.x
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = WDT_TIMEOUT_SECONDS * 1000, // Đổi từ giây sang mili-giây
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1, // Giám sát lỗi trên tất cả các lõi CPU
    .trigger_panic = true // Tự động Hard Reset (Reboot) chip khi bị treo kẹt
  };
  
  // Truyền con trỏ cấu hình vào hàm khởi tạo
  esp_task_wdt_init(&wdt_config); 
  
  Serial.println("Kích hoạt Hardware Task Watchdog Timer v3.x thành công!");

  Serial.begin(115200);
  RADAR_SERIAL.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);

  xRadarQueue   = xQueueCreate(20, sizeof(RadarRaw));
  xResultMutex  = xSemaphoreCreateMutex();
  xZoneMutex    = xSemaphoreCreateMutex();

  memset(&tsRooms,       0, sizeof(tsRooms));
  memset(&sharedResults, 0, sizeof(sharedResults));
  
  for(int i=0; i<NUM_ROOMS; i++){
    sharedResults[i].valid = false;
  }

  xTaskCreatePinnedToCore(Task_Radar,     "Task_Radar",     4096, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(Task_Logic,     "Task_Logic",     8192, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(Task_WebServer, "Task_WebServer", 8192, NULL, 1, NULL, 1);

  Serial.println("Hệ thống đa phòng tích hợp dữ liệu mô phỏng đã sẵn sàng");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
