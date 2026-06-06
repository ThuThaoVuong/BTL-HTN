#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <esp_task_wdt.h>
#include "esp_pm.h"
#include "esp_wifi.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Update.h>   // ← OTA Firmware Update

// ================= CẤU HÌNH WIFI =================
const char* ssid     = "hải hiền";
const char* password = "88888888";

// ================= CẤU HÌNH RADAR LD2450 =================
HardwareSerial radarSerial(2);
#define RADAR_SERIAL    radarSerial
#define RADAR_BAUD      115200
#define RADAR_RX_PIN    16
#define RADAR_TX_PIN    17

// ================= CẤU HÌNH ĐÈN LED, NÚT BẤM VÀ BUZZER =================
#define LED_BUILTIN_PIN  2
#define BUTTON_BOOT_PIN  0
#define BUZZER_PIN       25

// ================= GIỚI HẠN TẦM QUÉT LD2450 =================
#define MIN_Y       200
#define MAX_Y       6000
#define MAX_X_ABS   3500

// ================= CẤU HÌNH HỆ THỐNG ĐA PHÒNG =================
#define NUM_ROOMS   3
const char* roomNames[NUM_ROOMS]    = {"Phong Khach", "Phong Ngu", "Phong Tam"};
const char* roomNamesLCD[NUM_ROOMS] = {"PHONG KHACH", "PHONG NGU", "PHONG TAM"};
volatile uint8_t currentSelectedRoom = 0;

// ================= CẤU HÌNH LCD I2C =================
#define LCD_ADDR  0x27
#define LCD_COLS   16
#define LCD_ROWS   2
#define LCD_SDA    21
#define LCD_SCL    22
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);
SemaphoreHandle_t xLcdMutex;

uint8_t iconPerson[8] = { 0x04,0x0E,0x04,0x0E,0x15,0x04,0x0A,0x11 };
uint8_t iconWarn[8]   = { 0x04,0x0E,0x0E,0x1F,0x1B,0x04,0x0E,0x00 };
uint8_t iconHeart[8]  = { 0x00,0x0A,0x1F,0x1F,0x0E,0x04,0x00,0x00 };

// ================= CẤU HÌNH WATCHDOG =================
#define WDT_TIMEOUT_SECONDS   8
uint32_t lastRadarPacketTime = 0;
volatile bool isSensorConnected = true;

// ================= CẤU HÌNH TIẾT KIỆM NĂNG LƯỢNG =================
#define ECO_TIMEOUT_MS        20000
uint32_t lastTargetDetectedTime = 0;
volatile bool isEcoModeActive = false;

// ================= OTA STATE =================
volatile bool otaInProgress = false;
volatile bool otaSuccess    = false;

// ================= FREERTOS: QUEUE & MUTEX =================
struct RadarRaw { int16_t x, y, speed; };
QueueHandle_t     xRadarQueue;
SemaphoreHandle_t xResultMutex;

// ================= DỮ LIỆU KẾT QUẢ CHIA SẺ =================
struct TargetResult {
  int16_t x, y;
  int     speed;
  int     dist;
  String  action;
  bool    valid;
} sharedResults[NUM_ROOMS];

// ================= VÙNG AN TOÀN =================
#define MAX_ZONES 5
struct Zone { int16_t x_min, y_min, x_max, y_max; bool active = false; };
Zone safeZones[NUM_ROOMS][MAX_ZONES];
int  zoneCount[NUM_ROOMS] = {0, 0, 0};
SemaphoreHandle_t xZoneMutex;

// ================= TRẠNG THÁI THEO DÕI =================
struct TargetState {
  int16_t  x_history[10];
  int16_t  y_history[10];
  uint8_t  h_idx;
  bool     potential_fall;
  uint32_t fall_timer;
  float    smooth_doppler;
  float    smooth_dist;
  String   action;
  uint32_t last_seen;
  int16_t  prev_x;
  int16_t  prev_y;
  bool     prev_valid;
  uint32_t action_stable_since;
} tsRooms[NUM_ROOMS];

int16_t simX[NUM_ROOMS]    = {0, -1000, 1200};
int16_t simY[NUM_ROOMS]    = {0,  2500, 3000};
int16_t simDirX[NUM_ROOMS] = {0,    40,  -50};
int16_t simDirY[NUM_ROOMS] = {0,    60,  -30};

// ================= WEB SERVER & WEBSOCKET =================
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ================= LED BLINK (non-blocking) =================
volatile uint32_t ledOffTime = 0;
void ledBlink() { digitalWrite(LED_BUILTIN_PIN, HIGH); ledOffTime = millis() + 70; }

// ================= BUZZER (non-blocking) =================
volatile uint32_t buzzerOffTime  = 0;
volatile uint8_t  buzzerBeepLeft = 0;
volatile uint32_t buzzerNextBeep = 0;
#define BUZZER_ON_MS   120
#define BUZZER_OFF_MS  180

void buzzerTick() {
  uint32_t now = millis();
  if (buzzerBeepLeft > 0 && now >= buzzerNextBeep) {
    if (digitalRead(BUZZER_PIN) == LOW) {
      digitalWrite(BUZZER_PIN, HIGH);
      buzzerOffTime = now + BUZZER_ON_MS;
    } else if (now >= buzzerOffTime) {
      digitalWrite(BUZZER_PIN, LOW);
      buzzerBeepLeft--;
      buzzerNextBeep = now + BUZZER_OFF_MS;
    }
  }
}
void buzzerAlert(uint8_t beeps = 3) {
  if (buzzerBeepLeft == 0) { buzzerBeepLeft = beeps; buzzerNextBeep = millis(); }
}

// ============================================================
//   LCD TASK
// ============================================================
char lcdBuf[2][17];
char lcdNew[2][17];

void lcdWriteIfChanged() {
  for (int row = 0; row < 2; row++) {
    if (strncmp(lcdBuf[row], lcdNew[row], 16) != 0) {
      strncpy(lcdBuf[row], lcdNew[row], 16);
      lcdBuf[row][16] = '\0';
      if (xSemaphoreTake(xLcdMutex, pdMS_TO_TICKS(30)) == pdTRUE) {
        lcd.setCursor(0, row);
        char padded[17];
        snprintf(padded, sizeof(padded), "%-16s", lcdNew[row]);
        lcd.print(padded);
        xSemaphoreGive(xLcdMutex);
      }
    }
  }
}

void Task_LCD(void* pvParam) {
  esp_task_wdt_add(NULL);

  if (xSemaphoreTake(xLcdMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
    lcd.createChar(0, iconPerson);
    lcd.createChar(1, iconWarn);
    lcd.createChar(2, iconHeart);
    xSemaphoreGive(xLcdMutex);
  }
  vTaskDelay(pdMS_TO_TICKS(2000));
  if (xSemaphoreTake(xLcdMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    lcd.clear(); xSemaphoreGive(xLcdMutex);
  }
  for (int i = 0; i < 2; i++) {
    memset(lcdBuf[i], ' ', 16); lcdBuf[i][16] = '\0';
    memset(lcdNew[i], 0, 17);
  }

  uint32_t lastLcdUpdate = 0;

  for (;;) {
    esp_task_wdt_reset();
    uint32_t now = millis();

    // ── Hiển thị trạng thái OTA trên LCD ───────────────────
    if (otaInProgress) {
      if (xSemaphoreTake(xLcdMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        lcd.setCursor(0, 0); lcd.print("  CAP NHAT OTA  ");
        lcd.setCursor(0, 1); lcd.print("  Dang xu ly... ");
        xSemaphoreGive(xLcdMutex);
      }
      vTaskDelay(pdMS_TO_TICKS(300)); continue;
    }
    if (otaSuccess) {
      if (xSemaphoreTake(xLcdMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        lcd.setCursor(0, 0); lcd.print(" UPDATE THANH   ");
        lcd.setCursor(0, 1); lcd.print(" CONG! Restart..");
        xSemaphoreGive(xLcdMutex);
      }
      vTaskDelay(pdMS_TO_TICKS(300)); continue;
    }
    // ───────────────────────────────────────────────────────

    if (now - lastLcdUpdate < 500) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }
    lastLcdUpdate = now;

    uint8_t r = currentSelectedRoom;
    TargetResult res;
    if (xSemaphoreTake(xResultMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      res = sharedResults[r]; xSemaphoreGive(xResultMutex);
    }

    char line0[17], line1[17];
    snprintf(line0, sizeof(line0), "%-16s", roomNamesLCD[r]);

    if (!isSensorConnected && r == 0)        snprintf(line1,sizeof(line1),"! SENSOR OFFLINE");
    else if (isEcoModeActive)                snprintf(line1,sizeof(line1),"Status:ECO      ");
    else if (!res.valid)                     snprintf(line1,sizeof(line1),"Status:NO TARGET");
    else if (res.action == "FALL")           snprintf(line1,sizeof(line1),"Status:FALL     ");
    else if (res.action == "IMMOBILE")       snprintf(line1,sizeof(line1),"Status:IMMOBILE ");
    else if (res.action == "MOVING")         snprintf(line1,sizeof(line1),"Status:MOVING   ");
    else if (res.action == "STILL")          snprintf(line1,sizeof(line1),"Status:STILL    ");
    else if (res.action.startsWith("FALLING"))snprintf(line1,sizeof(line1),"Status:FALLING? ");
    else                                     snprintf(line1,sizeof(line1),"Status:%-9s",res.action.c_str());

    strncpy(lcdNew[0], line0, 16); lcdNew[0][16]='\0';
    strncpy(lcdNew[1], line1, 16); lcdNew[1][16]='\0';
    lcdWriteIfChanged();
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ============================================================
//                     GIAO DIỆN WEB
// ============================================================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head><meta charset="UTF-8"><title>Radar Safe Guard</title>
<style>
  :root{--bg:#f0f2f5;--card:#ffffff;--text:#333;--safe:#28a745;--danger:#dc3545}
  body{background:var(--bg);color:var(--text);font-family:'Segoe UI',sans-serif;text-align:center;margin:0;padding:10px}
  .panel{background:var(--card);max-width:500px;margin:10px auto;padding:15px;border-radius:12px;box-shadow:0 4px 10px rgba(0,0,0,.1)}
  canvas{background:#fafafa;border:1px solid #ddd;border-radius:8px;cursor:crosshair;max-width:100%}
  .btn{padding:10px 15px;margin:5px;cursor:pointer;background:#007bff;color:#fff;border:none;border-radius:5px;font-weight:bold}
  .btn.active{background:#ffc107;color:#000}
  .room-selector{display:flex;justify-content:space-around;margin-bottom:15px}
  .btn-room{padding:10px;flex:1;margin:0 4px;border:1px solid #ccc;background:#e9ecef;cursor:pointer;border-radius:6px;font-weight:bold}
  .btn-room.selected{background:#007bff;color:#fff;border-color:#007bff}
  .card{background:var(--card);padding:15px;border-radius:8px;border-left:5px solid #ccc;text-align:left;font-size:14px;max-width:500px;margin:10px auto}
  .card.active{border-left-color:var(--safe)}
  .card.danger{border-left-color:var(--danger);background:#fff5f5;animation:blink 1s infinite}
  .card.empty{border-left-color:#ccc}
  @keyframes blink{50%{opacity:.7}}
  .t-name{font-weight:bold;font-size:16px;border-bottom:1px solid #eee;margin-bottom:8px}
  .zone-info{font-size:12px;color:#666;margin-top:6px}
  #alert-overlay{display:none;position:fixed;bottom:20px;right:20px;z-index:999}
  #alert-overlay.show{display:block}
  #alert-box{background:#fff;border-radius:12px;padding:16px 18px;width:260px;text-align:left;box-shadow:0 6px 20px rgba(0,0,0,.2);border-left:6px solid #dc3545}
  #alert-icon{font-size:32px;margin-bottom:6px}
  #alert-title{font-size:16px;font-weight:bold;color:#dc3545;margin-bottom:4px}
  #alert-msg{font-size:13px;color:#555;margin-bottom:6px}
  #alert-time{font-size:11px;color:#999;margin-bottom:10px}
  #alert-ok{padding:6px 14px;background:#dc3545;color:#fff;border:none;border-radius:6px;font-size:13px;cursor:pointer}
  #alert-ok:hover{background:#b02a37}
  #alert-overlay.show #alert-box{animation:slideIn .3s ease}
  @keyframes slideIn{from{transform:translateY(20px);opacity:0}to{transform:translateY(0);opacity:1}}
  .sys-status-bar{display:flex;justify-content:space-between;font-size:12px;padding:6px 10px;background:#e9ecef;border-radius:6px;margin-bottom:10px;font-weight:bold}
  .status-ok{color:#28a745}
  .status-err{color:#dc3545;animation:blink 1s infinite}

  /* ── OTA PANEL ── */
  #ota-panel{background:#fff;max-width:500px;margin:10px auto;padding:15px;border-radius:12px;box-shadow:0 4px 10px rgba(0,0,0,.1);text-align:left}
  #ota-panel h4{margin:0 0 10px;font-size:15px;color:#333}
  #ota-drop{border:2px dashed #adb5bd;border-radius:8px;padding:20px;text-align:center;color:#888;font-size:13px;cursor:pointer;transition:border-color .2s,background .2s}
  #ota-drop.dragover{border-color:#007bff;background:#f0f7ff;color:#007bff}
  #ota-file-input{display:none}
  #ota-filename{font-size:12px;color:#555;margin:8px 0 4px;min-height:16px}
  #ota-progress-wrap{display:none;margin-top:8px}
  #ota-bar-bg{background:#e9ecef;border-radius:20px;height:10px;overflow:hidden}
  #ota-bar{height:10px;width:0%;background:#007bff;border-radius:20px;transition:width .3s}
  #ota-pct{font-size:12px;color:#555;margin-top:4px;text-align:center}
  #ota-btn{margin-top:10px;width:100%;padding:10px;background:#28a745;color:#fff;border:none;border-radius:6px;font-weight:bold;font-size:14px;cursor:pointer;display:none}
  #ota-btn:hover{background:#218838}
  #ota-btn:disabled{background:#adb5bd;cursor:not-allowed}

  /* ── Thông báo thành công OTA ── */
  #ota-ok-overlay{display:none;position:fixed;inset:0;background:rgba(0,0,0,.55);z-index:1000;align-items:center;justify-content:center}
  #ota-ok-overlay.show{display:flex}
  #ota-ok-box{background:#fff;border-radius:16px;padding:32px 28px;max-width:340px;width:90%;text-align:center;box-shadow:0 8px 32px rgba(0,0,0,.25);animation:popIn .35s cubic-bezier(.34,1.56,.64,1)}
  @keyframes popIn{from{transform:scale(.7);opacity:0}to{transform:scale(1);opacity:1}}
  .ota-ok-icon{font-size:56px;margin-bottom:10px}
  .ota-ok-title{font-size:22px;font-weight:bold;color:#28a745;margin-bottom:8px}
  .ota-ok-msg{font-size:14px;color:#555;margin-bottom:4px}
  .ota-ok-sub{font-size:12px;color:#999;margin-bottom:12px}
  #ota-countdown{font-size:40px;font-weight:bold;color:#007bff;margin-bottom:16px}
  #ota-ok-close{padding:9px 26px;background:#28a745;color:#fff;border:none;border-radius:8px;font-size:14px;font-weight:bold;cursor:pointer}
  #ota-ok-close:hover{background:#218838}
</style></head><body>

<div class="panel">
  <h3>HỆ THỐNG GIÁM SÁT AN TOÀN TRỰC TUYẾN</h3>
  <div class="sys-status-bar">
    <span>Phần cứng: <span id="hw-status" class="status-ok">🟢 ĐỒNG BỘ</span></span>
    <span>Chế độ: <span id="pwr-status" style="color:#28a745">⚡ HIGH PERF</span></span>
    <span>RAM: <span id="hw-heap" style="color:#007bff">-- KB</span></span>
  </div>
  <div class="room-selector">
    <button class="btn-room selected" id="btn-r0" onclick="selectRoom(0)">Phòng Khách</button>
    <button class="btn-room" id="btn-r1" onclick="selectRoom(1)">Phòng Ngủ (Sim)</button>
    <button class="btn-room" id="btn-r2" onclick="selectRoom(2)">Phòng Tắm (Sim)</button>
  </div>
  <button class="btn" style="background:#28a745;width:100%;margin-bottom:15px;font-size:15px" onclick="nextRoomWeb()">🔄 Đổi Phòng (Kế Tiếp)</button>
  <button class="btn" id="bz" onclick="startDraw()">Vẽ Vùng An Toàn</button>
  <button class="btn" style="background:#6c757d" onclick="clearZone()">Xóa Vùng</button>
  <br><small id="ws-status" style="font-size:12px;color:#ffc107">🟡 Đang kết nối…</small>
  <br><canvas id="rd" width="400" height="400"></canvas>
  <div class="zone-info" id="zi">Chưa có vùng an toàn</div>
</div>

<div id="card-1" class="card empty">
  <div class="t-name">👤 Người theo dõi</div>
  <div id="s-1">Không phát hiện</div>
</div>

<!-- ── OTA Firmware Update Panel ── -->
<div id="ota-panel">
  <h4>🔧 Cập Nhật Firmware OTA</h4>
  <div id="ota-drop"
       onclick="document.getElementById('ota-file-input').click()"
       ondragover="otaDragOver(event)" ondragleave="otaDragLeave(event)" ondrop="otaDrop(event)">
    📂 Kéo thả file <b>.bin</b> vào đây hoặc <u>bấm để chọn</u>
  </div>
  <input type="file" id="ota-file-input" accept=".bin" onchange="otaFileSelected(this.files[0])">
  <div id="ota-filename"></div>
  <div id="ota-progress-wrap">
    <div id="ota-bar-bg"><div id="ota-bar"></div></div>
    <div id="ota-pct">0%</div>
  </div>
  <button id="ota-btn" onclick="otaUpload()">⬆️ Tải Firmware Lên ESP32</button>
</div>

<!-- Alert cảnh báo ngã / ngất -->
<div id="alert-overlay">
  <div id="alert-box">
    <div id="alert-icon">🚨</div>
    <div id="alert-title">CẢNH BÁO</div>
    <div id="alert-msg">Phát hiện bất thường!</div>
    <div id="alert-time"></div>
    <button id="alert-ok" onclick="dismissAlert()">Đã biết</button>
  </div>
</div>

<!-- ── OTA Thành Công Overlay ── -->
<div id="ota-ok-overlay">
  <div id="ota-ok-box">
    <div class="ota-ok-icon">✅</div>
    <div class="ota-ok-title">CẬP NHẬT THÀNH CÔNG!</div>
    <div class="ota-ok-msg">Firmware đã được nạp thành công.</div>
    <div class="ota-ok-sub">ESP32 tự khởi động lại sau:</div>
    <div id="ota-countdown">5</div>
    <button id="ota-ok-close" onclick="otaOkClose()">✔ Đã hiểu</button>
  </div>
</div>

<script>
/* ─── Canvas & radar ─── */
let cv=document.getElementById("rd"),ctx=cv.getContext("2d");
let drawing=false,isDragging=false,startX,startY,currentX,currentY;
let currentRoom=0,roomZones=[[],[],[]],lastTarget=null;
let renderPending=false;
function scheduleRender(){if(!renderPending){renderPending=true;requestAnimationFrame(()=>{renderPending=false;render();});}}

/* ─── WebSocket ─── */
let socket=null,reconnectTimer=null,reconnectDelay=2000;
const MAX_DELAY=15000;
function wsConnect(){
  if(socket&&(socket.readyState===WebSocket.OPEN||socket.readyState===WebSocket.CONNECTING))return;
  socket=new WebSocket(`ws://${location.host}/ws`);
  setStatus("connecting");
  socket.onopen=()=>{setStatus("online");reconnectDelay=2000;clearTimeout(reconnectTimer);};
  const WS_DZ=200;let lastHash="",lastRenderTime=0;
  socket.onmessage=(e)=>{
    try{
      const d=JSON.parse(e.data);updateSysBar(d);
      if(d.sync&&d.room!==currentRoom){currentRoom=d.room;updateRoomTabUI();}
      if(d.room!==currentRoom)return;
      const qx=d.valid?Math.round(d.x/WS_DZ):0,qy=d.valid?Math.round(d.y/WS_DZ):0;
      const hash=`${d.valid}|${qx}|${qy}|${d.a}`;
      const changed=hash!==lastHash;lastHash=hash;lastTarget=d.valid?d:null;
      const now=Date.now();
      if(changed&&now-lastRenderTime>200){lastRenderTime=now;scheduleRender();updateUI(d);}
    }catch(_){}
  };
  socket.onclose=()=>{setStatus("offline");clearTimeout(reconnectTimer);reconnectTimer=setTimeout(()=>{wsConnect();reconnectDelay=Math.min(reconnectDelay*1.5,MAX_DELAY);},reconnectDelay);};
  socket.onerror=()=>socket.close();
}
function wsSend(m){if(socket&&socket.readyState===WebSocket.OPEN)socket.send(m);}
function setStatus(s){
  const el=document.getElementById("ws-status");if(!el)return;
  const map={online:{t:"🟢 Đã kết nối",c:"#28a745"},offline:{t:"🔴 Mất kết nối – thử lại…",c:"#dc3545"},connecting:{t:"🟡 Đang kết nối…",c:"#ffc107"}};
  const v=map[s]||map.offline;el.innerText=v.t;el.style.color=v.c;
  if(s==="offline"){lastTarget=null;scheduleRender();updateUI({valid:false});}
}
function updateSysBar(d){
  const hw=document.getElementById("hw-status"),pwr=document.getElementById("pwr-status"),hp=document.getElementById("hw-heap");
  if(d.hw_status==="OK"){hw.innerText="🟢 ĐỒNG BỘ";hw.className="status-ok";}
  else{hw.innerText="🚨 LỖI PHẦN CỨNG (UART LOST)";hw.className="status-err";}
  pwr.innerText=d.power_mode==="ECO"?"🌱 ECO MODE (80MHz)":"⚡ HIGH PERF (240MHz)";
  pwr.style.color=d.power_mode==="ECO"?"#198754":"#ffc107";
  if(d.free_heap)hp.innerText=Math.round(d.free_heap/1024)+" KB";
}
wsConnect();

/* ─── Phòng & vùng ─── */
function nextRoomWeb(){selectRoom((currentRoom+1)%3);}
function selectRoom(id){currentRoom=id;updateRoomTabUI();wsSend(`SELECT_ROOM:${id}`);lastTarget=null;scheduleRender();updateUI({valid:false});}
function updateRoomTabUI(){
  for(let i=0;i<3;i++)document.getElementById(`btn-r${i}`).classList.toggle("selected",i===currentRoom);
  const z=roomZones[currentRoom].length;
  document.getElementById("zi").innerText=z>0?`${z} vùng an toàn đang hoạt động`:"Chưa có vùng an toàn";
}
function startDraw(){drawing=true;document.getElementById("bz").classList.add("active");}
function clearZone(){roomZones[currentRoom]=[];wsSend(`CLEAR_ROOM:${currentRoom}`);document.getElementById("zi").innerText="Chưa có vùng an toàn";scheduleRender();}

cv.onmousedown=(e)=>{if(!drawing)return;startX=e.offsetX;startY=e.offsetY;isDragging=true;};
cv.onmousemove=(e)=>{if(isDragging){currentX=e.offsetX;currentY=e.offsetY;scheduleRender();}};
cv.onmouseup=(e)=>{
  if(!isDragging)return;isDragging=false;
  let x1=(startX-200)*30,y1=(200-startY)*30,x2=(e.offsetX-200)*30,y2=(200-e.offsetY)*30;
  y1=Math.max(y1,0);y2=Math.max(y2,0);
  const z={x_min:Math.min(x1,x2),y_min:Math.min(y1,y2),x_max:Math.max(x1,x2),y_max:Math.max(y1,y2)};
  if(roomZones[currentRoom].length<5){
    roomZones[currentRoom].push(z);
    wsSend(`ADD_ZONE_ROOM:${currentRoom},${z.x_min},${z.y_min},${z.x_max},${z.y_max}`);
    document.getElementById("zi").innerText=`${roomZones[currentRoom].length} vùng an toàn đang hoạt động`;
  }
  drawing=false;document.getElementById("bz").classList.remove("active");scheduleRender();
};

/* ─── UI update ─── */
function updateUI(t){
  const card=document.getElementById("card-1"),st=document.getElementById("s-1");
  if(!t.valid){st.innerText="Không phát hiện";card.className="card empty";hideAlert();return;}
  st.innerHTML=`<b>${t.a}</b><br>X: ${t.x} mm &nbsp; Y: ${t.y} mm<br><small>Tốc độ: ${t.spd} mm/s &nbsp; Di chuyển: ${t.dist} mm</small>`;
  const danger=t.a==="FALL"||t.a==="IMMOBILE";
  card.className=danger?"card danger":"card active";
  danger?showAlert(t.a):hideAlert();
}

/* ─── Alert ─── */
let alertDismissed=false,lastAlertState="",alertTimerInterval=null,alertStartTime=null;
function showAlert(s){
  if(s!==lastAlertState){lastAlertState=s;alertDismissed=false;alertStartTime=Date.now();startAlertTimer();}
  if(alertDismissed)return;
  document.getElementById("alert-icon").textContent=s.includes("IMMOBILE")?"🛑":"🚨";
  document.getElementById("alert-title").textContent=s.includes("IMMOBILE")?"PHÁT HIỆN NGẤT":"PHÁT HIỆN NGÃ";
  document.getElementById("alert-msg").textContent=s.includes("IMMOBILE")?"Người theo dõi không di chuyển trong thời gian dài!":"Người theo dõi có thể đã bị ngã!";
  document.getElementById("alert-overlay").classList.add("show");
}
function hideAlert(){document.getElementById("alert-overlay").classList.remove("show");lastAlertState="";alertDismissed=false;stopAlertTimer();}
function dismissAlert(){alertDismissed=true;document.getElementById("alert-overlay").classList.remove("show");}
function startAlertTimer(){
  stopAlertTimer();
  alertTimerInterval=setInterval(()=>{
    if(!alertStartTime)return;
    const s=Math.floor((Date.now()-alertStartTime)/1000);
    document.getElementById("alert-time").textContent=`Thời gian cảnh báo: ${String(Math.floor(s/60)).padStart(2,"0")}:${String(s%60).padStart(2,"0")}`;
  },1000);
}
function stopAlertTimer(){if(alertTimerInterval){clearInterval(alertTimerInterval);alertTimerInterval=null;}document.getElementById("alert-time").textContent="";alertStartTime=null;}

/* ─── Canvas render ─── */
function render(){
  ctx.clearRect(0,0,400,400);
  ctx.strokeStyle="#eee";ctx.lineWidth=1;
  for(let i=0;i<=400;i+=50){ctx.beginPath();ctx.moveTo(i,0);ctx.lineTo(i,400);ctx.stroke();ctx.beginPath();ctx.moveTo(0,i);ctx.lineTo(400,i);ctx.stroke();}
  ctx.fillStyle="rgba(200,200,200,.25)";ctx.fillRect(0,200,400,200);
  ctx.fillStyle="#999";ctx.font="11px Arial";ctx.textAlign="center";
  ctx.fillText("PHÍA SAU – KHÔNG THEO DÕI",200,370);
  ctx.fillStyle="#aaa";ctx.fillText("↑ PHÍA TRƯỚC ↑",200,20);
  ctx.fillStyle="#007bff";ctx.beginPath();ctx.arc(200,200,12,0,Math.PI*2);ctx.fill();
  ctx.strokeStyle="#007bff";ctx.lineWidth=2;
  ctx.beginPath();ctx.moveTo(200,200);ctx.lineTo(172,172);ctx.lineTo(228,172);ctx.closePath();ctx.stroke();
  roomZones[currentRoom].forEach((z,i)=>{
    ctx.fillStyle="rgba(40,167,69,.15)";ctx.strokeStyle="#28a745";ctx.lineWidth=2;
    const rx=200+z.x_min/30,ry=200-z.y_max/30,rw=(z.x_max-z.x_min)/30,rh=(z.y_max-z.y_min)/30;
    ctx.fillRect(rx,ry,rw,rh);ctx.strokeRect(rx,ry,rw,rh);
    ctx.fillStyle="#28a745";ctx.font="10px Arial";ctx.textAlign="left";ctx.fillText("Vùng "+(i+1),rx+4,ry+12);
  });
  if(isDragging){ctx.strokeStyle="#007bff";ctx.setLineDash([5,5]);ctx.lineWidth=1;ctx.strokeRect(startX,startY,currentX-startX,currentY-startY);ctx.setLineDash([]);}
  if(lastTarget&&lastTarget.valid){
    let px=200+lastTarget.x/30,py=200-lastTarget.y/30;
    px=Math.max(6,Math.min(394,px));py=Math.max(6,Math.min(394,py));
    const danger=lastTarget.a.includes("FALL")||lastTarget.a.includes("IMMOBILE");
    ctx.fillStyle=danger?"red":"#28a745";ctx.beginPath();ctx.arc(px,py,10,0,Math.PI*2);ctx.fill();
    ctx.fillStyle="#333";ctx.font="bold 11px Arial";ctx.textAlign="left";ctx.fillText(lastTarget.a,px+14,py-8);
    ctx.fillStyle="#666";ctx.font="9px Arial";ctx.fillText(`(${lastTarget.x},${lastTarget.y})`,px+14,py+4);
  }
}
scheduleRender();

/* ════════════════════════════════════════════════════
   OTA FIRMWARE UPDATE
   ════════════════════════════════════════════════════ */
let otaFile=null;

function otaDragOver(e){e.preventDefault();document.getElementById("ota-drop").classList.add("dragover");}
function otaDragLeave(){document.getElementById("ota-drop").classList.remove("dragover");}
function otaDrop(e){
  e.preventDefault();document.getElementById("ota-drop").classList.remove("dragover");
  const f=e.dataTransfer.files[0];if(f)otaFileSelected(f);
}
function otaFileSelected(f){
  if(!f||!f.name.endsWith(".bin")){alert("Vui lòng chọn file firmware .bin hợp lệ!");return;}
  otaFile=f;
  document.getElementById("ota-filename").textContent="📄 "+f.name+" ("+(f.size/1024).toFixed(1)+" KB)";
  document.getElementById("ota-btn").style.display="block";
  document.getElementById("ota-progress-wrap").style.display="none";
  document.getElementById("ota-bar").style.width="0%";
  document.getElementById("ota-pct").textContent="0%";
}
function otaUpload(){
  if(!otaFile)return;
  const btn=document.getElementById("ota-btn");
  btn.disabled=true;btn.textContent="⏳ Đang tải lên...";
  document.getElementById("ota-progress-wrap").style.display="block";
  const xhr=new XMLHttpRequest();
  xhr.open("POST","/update",true);
  let successShown = false;
  xhr.upload.onprogress=(e)=>{
    if(e.lengthComputable){
      const p=Math.round(e.loaded/e.total*100);
      document.getElementById("ota-bar").style.width=p+"%";
      document.getElementById("ota-pct").textContent=p+"%";
      // Hiện popup ngay khi upload đạt 100% — không chờ response
      if(p===100 && !successShown){
        successShown=true;
        btn.textContent="✅ Hoàn tất!";
        showOtaSuccess();
      }
    }
  };
  xhr.upload.onload=()=>{
    // Upload hoàn tất (dự phòng nếu onprogress không fire đúng 100%)
    if(!successShown){
      successShown=true;
      document.getElementById("ota-bar").style.width="100%";
      document.getElementById("ota-pct").textContent="100%";
      btn.textContent="✅ Hoàn tất!";
      showOtaSuccess();
    }
  };
  xhr.onload=()=>{ /* response đến sau restart – đã xử lý ở upload.onload */ };
  xhr.onerror=()=>{
    // ERR_CONNECTION_RESET sau upload 100% = ESP32 đang restart, bình thường
    if(!successShown){
      btn.disabled=false;
      btn.textContent="⬆️ Tải Firmware Lên ESP32";
      alert("❌ Không thể kết nối thiết bị! Kiểm tra lại WiFi hoặc IP.");
    }
  };
  xhr.send(otaFile);
}

let _otaTimer=null;
function showOtaSuccess(){
  const overlay=document.getElementById("ota-ok-overlay");
  overlay.classList.add("show");
  let sec=5;
  document.getElementById("ota-countdown").textContent=sec;
  _otaTimer=setInterval(()=>{
    sec--;document.getElementById("ota-countdown").textContent=sec;
    if(sec<=0){clearInterval(_otaTimer);location.reload();}
  },1000);
}
function otaOkClose(){
  if(_otaTimer)clearInterval(_otaTimer);
  location.reload();
}
</script>
</body></html>
)rawliteral";

// ============================================================
//            KIỂM TRA VÙNG AN TOÀN
// ============================================================
bool checkIfInAnySafeZone(uint8_t r_id, int16_t x, int16_t y) {
  for (int i = 0; i < MAX_ZONES; i++) {
    if (safeZones[r_id][i].active &&
        x >= safeZones[r_id][i].x_min && x <= safeZones[r_id][i].x_max &&
        y >= safeZones[r_id][i].y_min && y <= safeZones[r_id][i].y_max)
      return true;
  }
  return false;
}

// ============================================================
//   TASK 1 – Task_Radar (Core 0, Priority 3)
// ============================================================
void Task_Radar(void* pvParam) {
  esp_task_wdt_add(NULL);
  static uint8_t buf[30];
  static uint8_t idx = 0;
  for (;;) {
    esp_task_wdt_reset();
    while (RADAR_SERIAL.available()) {
      uint8_t b = RADAR_SERIAL.read();
      if (idx == 0 && b != 0xAA) continue;
      buf[idx++] = b;
      if (idx == 30) {
        idx = 0;
        if (buf[1] != 0xFF || buf[28] != 0x55) continue;
        lastRadarPacketTime = millis();
        isSensorConnected   = true;
        for (int j = 0; j < 3; j++) {
          int off = 4 + j * 8;
          int16_t rx = (buf[off+1]&0x80) ? -((int16_t)(buf[off+1]&0x7F)<<8|buf[off])   : ((int16_t)(buf[off+1]&0x7F)<<8|buf[off]);
          int16_t ry = (buf[off+3]&0x80) ? -((int16_t)(buf[off+3]&0x7F)<<8|buf[off+2]) : ((int16_t)(buf[off+3]&0x7F)<<8|buf[off+2]);
          int16_t rs = (buf[off+5]&0x80) ? -((int16_t)(buf[off+5]&0x7F)<<8|buf[off+4]) : ((int16_t)(buf[off+5]&0x7F)<<8|buf[off+4]);
          if (rx==0&&ry==0) continue;
          int16_t fy=-ry;
          if (fy<MIN_Y||fy>MAX_Y||abs(rx)>MAX_X_ABS) continue;
          RadarRaw raw={rx,ry,rs};
          xQueueSend(xRadarQueue,&raw,0);
          break;
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ============================================================
//   TASK 2 – Task_Logic (Core 0, Priority 2)
// ============================================================
void Task_Logic(void* pvParam) {
  esp_task_wdt_add(NULL);
  RadarRaw raw;
  for (;;) {
    esp_task_wdt_reset();
    uint8_t  r_id = currentSelectedRoom;
    uint32_t now  = millis();
    bool roomHasPeople = false;

    if (now - lastRadarPacketTime > 3000) isSensorConnected = false;

    if (r_id == 0) {
      if (!isSensorConnected) {
        if (xSemaphoreTake(xResultMutex,pdMS_TO_TICKS(10))==pdTRUE) {
          sharedResults[0]={0,0,0,0,"HW ERROR: RADAR LOST",true};
          xSemaphoreGive(xResultMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(100)); continue;
      }
      if (xQueueReceive(xRadarQueue,&raw,pdMS_TO_TICKS(60))==pdTRUE) {
        TargetState* ts=&tsRooms[0];
        int16_t x=raw.x,y=-raw.y,s=raw.speed;
        roomHasPeople=true;
        float distMoved=0,vY=0;
        if (ts->prev_valid) {
          distMoved=sqrtf(powf((float)(x-ts->prev_x),2)+powf((float)(y-ts->prev_y),2));
          vY=(float)(y-ts->prev_y);
        }
        ts->prev_x=x; ts->prev_y=y; ts->prev_valid=true;
        ts->smooth_dist=ts->smooth_dist*0.85f+distMoved*0.15f;
        float ed=ts->smooth_dist;
        ts->x_history[ts->h_idx]=x; ts->y_history[ts->h_idx]=y;
        ts->h_idx=(ts->h_idx+1)%10;
        ts->smooth_doppler=ts->smooth_doppler*0.85f+fabsf((float)s)*0.15f;
        ts->last_seen=now;
        bool inSafe=false;
        if (xSemaphoreTake(xZoneMutex,pdMS_TO_TICKS(10))==pdTRUE) {
          inSafe=checkIfInAnySafeZone(0,x,y); xSemaphoreGive(xZoneMutex);
        }
        if (!ts->potential_fall&&!inSafe&&vY<-250&&(ts->smooth_doppler>100||ed>250)) {
          ts->potential_fall=true; ts->fall_timer=now;
        }
        if (ts->potential_fall) {
          uint32_t el=now-ts->fall_timer;
          if (el>1000&&(ts->smooth_doppler>150||ed>350||inSafe)) {
            ts->potential_fall=false; ts->action="MOVING"; ts->action_stable_since=0;
          } else if (el>10000) { ts->action="IMMOBILE";
          } else if (el>2500)  { ts->action=(ts->smooth_doppler<80&&ed<200)?"FALL":"FALLING ???";
          } else               { ts->action="FALLING ???"; }
        } else {
          if (ts->action!="MOVING") {
            if (ts->smooth_doppler>150&&ed>200) {
              if (!ts->action_stable_since) ts->action_stable_since=now;
              if (now-ts->action_stable_since>800){ts->action="MOVING";ts->action_stable_since=0;}
            } else ts->action_stable_since=0;
          } else {
            if (ts->smooth_doppler<30&&ed<80) {
              if (!ts->action_stable_since) ts->action_stable_since=now;
              if (now-ts->action_stable_since>1200){ts->action="STILL";ts->action_stable_since=0;}
            } else ts->action_stable_since=0;
          }
        }
        if (xSemaphoreTake(xResultMutex,pdMS_TO_TICKS(10))==pdTRUE) {
          sharedResults[0]={x,y,(int)ts->smooth_doppler,(int)ed,ts->action,true};
          xSemaphoreGive(xResultMutex);
        }
      } else {
        if (millis()-tsRooms[0].last_seen>1500) {
          if (xSemaphoreTake(xResultMutex,pdMS_TO_TICKS(10))==pdTRUE) {
            sharedResults[0].valid=false; xSemaphoreGive(xResultMutex);
          }
        }
        if (millis()-tsRooms[0].last_seen>3000) {
          tsRooms[0].potential_fall=false; tsRooms[0].smooth_doppler=0;
          tsRooms[0].smooth_dist=0; tsRooms[0].prev_valid=false;
          tsRooms[0].action_stable_since=0;
        }
      }
    } else {
      vTaskDelay(pdMS_TO_TICKS(100));
      simX[r_id]+=simDirX[r_id]; simY[r_id]+=simDirY[r_id];
      if (abs(simX[r_id])>2500)              simDirX[r_id]=-simDirX[r_id];
      if (simY[r_id]<1000||simY[r_id]>5000) simDirY[r_id]=-simDirY[r_id];
      TargetState* ts=&tsRooms[r_id];
      if (r_id==2&&random(0,150)==25&&!ts->potential_fall) {
        ts->potential_fall=true; ts->fall_timer=millis(); ts->action="FALL";
      }
      if (ts->potential_fall) {
        if (millis()-ts->fall_timer>6000){ts->potential_fall=false;ts->action="MOVING";}
      } else ts->action="MOVING";
      if (xSemaphoreTake(xResultMutex,pdMS_TO_TICKS(10))==pdTRUE) {
        sharedResults[r_id]={simX[r_id],simY[r_id],(int)random(40,90),(int)random(100,200),ts->action,true};
        if (sharedResults[r_id].valid) roomHasPeople=true;
        xSemaphoreGive(xResultMutex);
      }
    }

    if (roomHasPeople) {
      lastTargetDetectedTime=now;
      if (isEcoModeActive) {
        setCpuFrequencyMhz(240); esp_wifi_set_ps(WIFI_PS_NONE);
        isEcoModeActive=false; Serial.println("[POWER] Khoi phuc 240MHz");
      }
    } else {
      if (!isEcoModeActive&&now-lastTargetDetectedTime>ECO_TIMEOUT_MS) {
        setCpuFrequencyMhz(80); esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
        isEcoModeActive=true; Serial.println("[POWER] ECO mode 80MHz");
      }
    }
  }
}

// ============================================================
//   TASK 3 – Task_WebServer (Core 1, Priority 1)
// ============================================================
void onWsEvent(AsyncWebSocket* s, AsyncWebSocketClient* c,
               AwsEventType t, void* arg, uint8_t* d, size_t l) {
  if (t != WS_EVT_DATA) return;
  String m=""; for(size_t i=0;i<l;i++) m+=(char)d[i];

  if (m.startsWith("SELECT_ROOM:")) {
    uint8_t r=m.substring(12).toInt();
    if (r<NUM_ROOMS){currentSelectedRoom=r;Serial.printf("[WS] %s\n",roomNames[r]);ledBlink();}
  } else if (m.startsWith("ADD_ZONE_ROOM:")) {
    if (xSemaphoreTake(xZoneMutex,pdMS_TO_TICKS(50))==pdTRUE) {
      uint8_t r;int16_t x1,y1,x2,y2;
      sscanf(m.substring(14).c_str(),"%hhu,%hd,%hd,%hd,%hd",&r,&x1,&y1,&x2,&y2);
      if (r<NUM_ROOMS&&zoneCount[r]<MAX_ZONES)
        safeZones[r][zoneCount[r]++]={x1,y1,x2,y2,true};
      xSemaphoreGive(xZoneMutex);
    }
  } else if (m.startsWith("CLEAR_ROOM:")) {
    uint8_t r=m.substring(11).toInt();
    if (r<NUM_ROOMS&&xSemaphoreTake(xZoneMutex,pdMS_TO_TICKS(50))==pdTRUE) {
      for(int i=0;i<MAX_ZONES;i++) safeZones[r][i].active=false;
      zoneCount[r]=0; xSemaphoreGive(xZoneMutex);
    }
  }
}

void Task_WebServer(void* pvParam) {
  WiFi.begin(ssid,password);
  Serial.print("Ket noi WiFi");
  while(WiFi.status()!=WL_CONNECTED){vTaskDelay(pdMS_TO_TICKS(500));Serial.print(".");}
  Serial.printf("\nIP: %s\n",WiFi.localIP().toString().c_str());

  pinMode(LED_BUILTIN_PIN,OUTPUT); pinMode(BUTTON_BOOT_PIN,INPUT_PULLUP); pinMode(BUZZER_PIN,OUTPUT);
  digitalWrite(LED_BUILTIN_PIN,LOW); digitalWrite(BUZZER_PIN,LOW);

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  // ── Trang chủ ────────────────────────────────────────────
  server.on("/",HTTP_GET,[](AsyncWebServerRequest* r){r->send_P(200,"text/html",index_html);});

  // ── OTA: POST /update ────────────────────────────────────
  server.on("/update", HTTP_POST,
    // [1] Phản hồi sau khi nhận xong toàn bộ file
    [](AsyncWebServerRequest* request) {
      bool ok = !Update.hasError();
      otaSuccess    = ok;
      otaInProgress = false;
      if (ok) {
        Serial.println("[OTA] Thanh cong! Restart...");
        request->send(200, "text/plain", "OK");
        vTaskDelay(pdMS_TO_TICKS(800));
        ESP.restart();
      } else {
        Serial.printf("[OTA] Loi: %s\n", Update.errorString());
        request->send(400, "text/plain", Update.errorString());
      }
    },
    // [2] Nhận từng chunk dữ liệu .bin
    [](AsyncWebServerRequest* request, String filename,
       size_t index, uint8_t* data, size_t len, bool final) {
      if (!index) {
        otaInProgress = true;
        otaSuccess    = false;
        buzzerBeepLeft = 0;
        digitalWrite(BUZZER_PIN, LOW);
        Serial.printf("[OTA] Bat dau: %s\n", filename.c_str());
        size_t sz = request->contentLength();
        if (!Update.begin(sz > 0 ? sz : UPDATE_SIZE_UNKNOWN)) {
          Serial.printf("[OTA] begin() loi: %s\n", Update.errorString());
          return;
        }
        // Hiển thị trên LCD ngay khi bắt đầu
        if (xSemaphoreTake(xLcdMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
          lcd.clear();
          lcd.setCursor(0,0); lcd.print("  CAP NHAT OTA  ");
          lcd.setCursor(0,1); lcd.print("  Dang xu ly... ");
          xSemaphoreGive(xLcdMutex);
        }
      }
      if (Update.write(data, len) != len) {
        Serial.printf("[OTA] Ghi loi: %s\n", Update.errorString());
        Update.abort(); otaInProgress = false; return;
      }
      if (final) {
        if (Update.end(true)) {
          Serial.printf("[OTA] Hoan tat %u bytes\n", (unsigned)(index+len));
          // LCD thông báo thành công
          if (xSemaphoreTake(xLcdMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            lcd.clear();
            lcd.setCursor(0,0); lcd.print(" UPDATE THANH   ");
            lcd.setCursor(0,1); lcd.print(" CONG! Restart..");
            xSemaphoreGive(xLcdMutex);
          }
          buzzerAlert(2);   // 2 tiếng bíp báo thành công
        } else {
          Serial.printf("[OTA] end() loi: %s\n", Update.errorString());
          otaInProgress = false;
        }
      }
    }
  );

  server.begin();
  Serial.println("[WEB] Server OK – OTA endpoint: POST /update");
  esp_task_wdt_add(NULL);

  unsigned long lastDebounce=0;
  const unsigned long DEBOUNCE=350;
  uint8_t lastRoomStatus=currentSelectedRoom;
  #define WS_DZ 200
  int16_t lastSentQx=32767,lastSentQy=32767,lastSentQspd=32767,lastSentQdist=32767;
  bool lastSentValid=false;
  String lastSentAction="";
  uint32_t lastForceSend=0,lastBuzzerCheck=0;

  for (;;) {
    esp_task_wdt_reset();

    // Tạm dừng khi đang OTA
    if (otaInProgress) { vTaskDelay(pdMS_TO_TICKS(200)); continue; }

    if (ledOffTime>0&&millis()>=ledOffTime) { digitalWrite(LED_BUILTIN_PIN,LOW); ledOffTime=0; }
    buzzerTick();

    if (millis()-lastBuzzerCheck>2000) {
      lastBuzzerCheck=millis();
      bool danger=false;
      if (xSemaphoreTake(xResultMutex,pdMS_TO_TICKS(20))==pdTRUE) {
        for(int ri=0;ri<NUM_ROOMS;ri++)
          if(sharedResults[ri].valid&&(sharedResults[ri].action=="FALL"||sharedResults[ri].action=="IMMOBILE"))
            {danger=true;break;}
        xSemaphoreGive(xResultMutex);
      }
      if (danger) { buzzerAlert(3); Serial.println("[BUZZER] Canh bao!"); }
    }

    if (digitalRead(BUTTON_BOOT_PIN)==LOW&&millis()-lastDebounce>DEBOUNCE) {
      lastDebounce=millis();
      currentSelectedRoom=(currentSelectedRoom+1)%NUM_ROOMS;
      lastSentQx=32767;lastSentQy=32767;lastSentQspd=32767;lastSentQdist=32767;
      lastSentAction="";lastSentValid=false;
      ledBlink();
    }

    if (ws.count()>0) {
      uint8_t r_id=currentSelectedRoom;
      TargetResult res;
      if (xSemaphoreTake(xResultMutex,pdMS_TO_TICKS(20))==pdTRUE){res=sharedResults[r_id];xSemaphoreGive(xResultMutex);}
      uint32_t now32=millis();
      bool roomChanged=(r_id!=lastRoomStatus);
      int16_t qx=res.valid?(res.x/WS_DZ):0,qy=res.valid?(res.y/WS_DZ):0;
      int16_t qspd=res.valid?(res.speed/50):0,qdist=res.valid?(res.dist/20):0;
      bool changed=roomChanged||(qx!=lastSentQx)||(qy!=lastSentQy)||(qspd!=lastSentQspd)||
                   (qdist!=lastSentQdist)||(res.action!=lastSentAction)||(res.valid!=lastSentValid)||
                   (now32-lastForceSend>2000);
      if (changed) {
        lastSentQx=qx;lastSentQy=qy;lastSentQspd=qspd;lastSentQdist=qdist;
        lastSentAction=res.action;lastSentValid=res.valid;lastForceSend=now32;lastRoomStatus=r_id;
        StaticJsonDocument<384> doc;
        doc["room"]=r_id;doc["valid"]=res.valid;
        doc["hw_status"]=isSensorConnected?"OK":"ERROR";
        doc["power_mode"]=isEcoModeActive?"ECO":"HIGH";
        doc["free_heap"]=ESP.getFreeHeap();doc["sync"]=roomChanged;
        if(res.valid){doc["x"]=res.x;doc["y"]=res.y;doc["spd"]=res.speed;doc["dist"]=res.dist;doc["a"]=res.action;}
        String out;serializeJson(doc,out);ws.textAll(out);
      }
    }
    ws.cleanupClients();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ============================================================
//                        SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  randomSeed(esp_random());

  esp_task_wdt_config_t wdt_cfg = {
    .timeout_ms      = WDT_TIMEOUT_SECONDS * 1000,
    .idle_core_mask  = (1 << portNUM_PROCESSORS) - 1,
    .trigger_panic   = true
  };
  esp_task_wdt_init(&wdt_cfg);
  Serial.println("[WDT] Watchdog OK");

  RADAR_SERIAL.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);

  xRadarQueue  = xQueueCreate(20, sizeof(RadarRaw));
  xResultMutex = xSemaphoreCreateMutex();
  xZoneMutex   = xSemaphoreCreateMutex();
  xLcdMutex    = xSemaphoreCreateMutex();

  memset(&tsRooms, 0, sizeof(tsRooms));
  memset(&sharedResults, 0, sizeof(sharedResults));
  for (int i=0;i<NUM_ROOMS;i++) {
    sharedResults[i].valid=false;
    tsRooms[i].prev_valid=false;
    tsRooms[i].action="STILL";
    tsRooms[i].smooth_dist=0;
    tsRooms[i].action_stable_since=0;
  }

  Wire.begin(LCD_SDA, LCD_SCL);
  delay(50);
  lcd.init(); lcd.backlight(); lcd.clear();
  lcd.setCursor(0,0); lcd.print("  RADAR SAFEGUARD");
  lcd.setCursor(0,1); lcd.print("   Khoi dong...");
  Serial.println("[LCD] OK tai 0x27");

  xTaskCreatePinnedToCore(Task_Radar,     "Radar",  4096, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(Task_Logic,     "Logic",  8192, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(Task_WebServer, "WebSrv", 8192, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(Task_LCD,       "LCD",    4096, NULL, 1, NULL, 1);

  Serial.println("[BOOT] San sang | LCD SDA=21 SCL=22 | Buzzer GPIO25 | OTA POST /update");
}

void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }
