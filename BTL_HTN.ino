#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

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

/*
struct Zone {
  int16_t x_min, y_min, x_max, y_max;
  bool active = false;
} safeZone; // Đã đổi tên thành SafeZone
*/

#define MAX_T 3
struct TargetState {
  int16_t  x_history[10];
  int16_t  y_history[10];
  uint8_t  h_idx;
  bool     potential_fall; 
  uint32_t fall_timer;     
  float    smooth_doppler;
  String   action;
} ts[MAX_T + 1];

// ================= GIAO DIỆN WEB (HTML/CSS/JS) =================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head><meta charset="UTF-8"><title>Radar Safe Guard</title>
<style>
  :root { --bg: #f0f2f5; --card: #ffffff; --text: #333; --safe: #28a745; --danger: #dc3545; }
  body { background: var(--bg); color: var(--text); font-family: 'Segoe UI', sans-serif; text-align: center; margin: 0; padding: 10px; }
  .panel { background: var(--card); max-width: 500px; margin: 10px auto; padding: 15px; border-radius: 12px; box-shadow: 0 4px 10px rgba(0,0,0,0.1); }
  canvas { background: #fafafa; border: 1px solid #ddd; border-radius: 8px; cursor: crosshair; max-width: 100%; }
  .btn { padding: 10px 15px; margin: 5px; cursor: pointer; background: #007bff; color: white; border: none; border-radius: 5px; font-weight: bold; }
  .btn.active { background: #ffc107; color: black; }
  .tracking-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); gap: 10px; max-width: 800px; margin: auto; }
  .card { background: var(--card); padding: 10px; border-radius: 8px; border-left: 5px solid #ccc; text-align: left; font-size: 13px; }
  .card.active { border-left-color: var(--safe); }
  .card.danger { border-left-color: var(--danger); background: #fff5f5; animation: blink 1s infinite; }
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
    <div id="card-3" class="card"><div class="t-name">Người 3</div><div id="s-3">Trống</div></div>
  </div>

<script>
  let socket = new WebSocket(`ws://${location.host}/ws`);
  let cv = document.getElementById("rd"), ctx = cv.getContext("2d");
  let drawing = false, isDragging = false, startX, startY, currentX, currentY;
  let zones = []; // Mảng lưu nhiều vùng
  let lastTargets = [];

  socket.onmessage = (e) => {
    lastTargets = JSON.parse(e.data);
    render();
    updateUI(lastTargets);
  };

  function startDraw() { drawing = true; document.getElementById("bz").classList.add("active"); }
  
  // Xóa vùng cuối cùng hoặc xóa hết
  function clearZone() { 
    zones = []; 
    socket.send("CLEAR_ALL"); 
    render(); 
  }

  cv.onmousedown = (e) => { if(!drawing) return; startX = e.offsetX; startY = e.offsetY; isDragging = true; };
  cv.onmousemove = (e) => { if(isDragging) { currentX = e.offsetX; currentY = e.offsetY; render(); } };

  cv.onmouseup = (e) => {
    if(!isDragging) return;
    isDragging = false;
    let x1 = (startX-200)*30, y1 = (200-startY)*30;
    let x2 = (e.offsetX-200)*30, y2 = (200-e.offsetY)*30;
    
    let newZone = {x_min: Math.min(x1,x2), y_min: Math.min(y1,y2), x_max: Math.max(x1,x2), y_max: Math.max(y1,y2)};
    zones.push(newZone);
    
    // Gửi vùng mới nhất lên ESP32
    socket.send(`ADD_ZONE:${newZone.x_min},${newZone.y_min},${newZone.x_max},${newZone.y_max}`);
    drawing = false; 
    document.getElementById("bz").classList.remove("active");
    render();
  };

  function updateUI(targets) {
    // 1. Reset tất cả các thẻ về trạng thái "Trống" trước khi cập nhật dữ liệu mới
    for(let i=1; i<=3; i++) {
      let card = document.getElementById(`card-${i}`);
      let status = document.getElementById(`s-${i}`);
      if (card && status) {
        status.innerText = "Trống";
        card.className = "card"; // Xóa bỏ các class danger/active cũ
      }
    }

    // 2. Cập nhật dữ liệu từ các Target đang hoạt động
    targets.forEach(t => {
      let card = document.getElementById(`card-${t.id}`);
      let status = document.getElementById(`s-${t.id}`);
      
      if(card && status) {
        // Hiển thị Hành động, Tọa độ và Chỉ số di chuyển
        status.innerHTML = `<b>${t.a}</b><br>X: ${t.x}, Y: ${t.y}<br><small>Move: ${t.dist}mm</small>`;
        
        // Kiểm tra trạng thái để đổi màu thẻ
        if (t.a.includes("FALL") || t.a.includes("IMMOBILE")) {
          card.className = "card danger"; // Màu đỏ nhấp nháy cho trường hợp nguy hiểm
        } else if (t.a.includes("FALLING")) {
          card.className = "card danger"; // Có thể đổi thành màu vàng nếu bạn muốn phân loại riêng
        } else {
          card.className = "card active"; // Màu xanh cho trạng thái bình thường/di chuyển
        }
      }
    });
  }

  function render() {
    ctx.clearRect(0,0,400,400);
    
    // --- 1. VẼ LƯỚI & CHỈ HƯỚNG ---
    ctx.strokeStyle="#eee"; ctx.lineWidth=1;
    for(let i=0; i<=400; i+=50) { // Lưới mỗi 1.5m
       ctx.beginPath(); ctx.moveTo(i,0); ctx.lineTo(i,400); ctx.stroke();
       ctx.beginPath(); ctx.moveTo(0,i); ctx.lineTo(400,i); ctx.stroke();
    }

    // Nhãn hướng cực rõ
    ctx.fillStyle = "#ffaaaa"; ctx.font = "bold 14px Arial";
    ctx.fillText("← BÊN TRÁI", 10, 210);
    ctx.textAlign = "right"; ctx.fillText("BÊN PHẢI →", 390, 210);
    ctx.textAlign = "center"; ctx.fillStyle = "#aaa";
    ctx.fillText("↑ PHÍA TRƯỚC (XA) ↑", 200, 20);

    // --- 2. VẼ RADAR ---
    ctx.fillStyle = "#007bff"; ctx.beginPath(); ctx.arc(200,200,12,0,7); ctx.fill();
    ctx.strokeStyle="#007bff"; ctx.beginPath(); ctx.moveTo(200,200); ctx.lineTo(170,170); ctx.lineTo(230,170); ctx.closePath(); ctx.stroke();

    // --- 3. VẼ TẤT CẢ CÁC VÙNG AN TOÀN ---
    zones.forEach((z, index) => {
      ctx.fillStyle="rgba(40,167,69,0.15)"; ctx.strokeStyle="#28a745";
      let rx = 200 + z.x_min/30, ry = 200 - z.y_max/30;
      let rw = (z.x_max - z.x_min)/30, rh = (z.y_max - z.y_min)/30;
      ctx.fillRect(rx, ry, rw, rh);
      ctx.strokeRect(rx, ry, rw, rh);
      ctx.fillStyle="#28a745"; ctx.font="10px Arial";
      ctx.fillText("Vùng "+(index+1), rx + 5, ry + 12);
    });

    // Vẽ preview khi đang kéo
    if(isDragging) {
      ctx.strokeStyle = "#007bff"; ctx.setLineDash([5,5]);
      ctx.strokeRect(startX, startY, currentX-startX, currentY-startY);
      ctx.setLineDash([]);
    }

    // --- 4. VẼ NGƯỜI ---
    lastTargets.forEach(t => {
      let px = 200 + t.x/30, py = 200 - t.y/30;
      ctx.fillStyle = (t.a.includes("FALL") || t.a.includes("IMMOBILE")) ? "red" : "#28a745";
      ctx.beginPath(); ctx.arc(px,py,10,0,7); ctx.fill();
      ctx.fillStyle="black"; ctx.font="bold 12px Arial";
      ctx.fillText("T"+t.id, px+14, py-10);
    });
  }
</script></body></html>
)rawliteral";

// ================= LOGIC XỬ LÝ (PHẦN C++) =================

void processTarget(int16_t x, int16_t y, int16_t s, int id, JsonArray& root) {
  TargetState& t = ts[id];
  uint32_t now = millis();
  y = -y; 

  int16_t x_old = t.x_history[t.h_idx];
  int16_t y_old = t.y_history[t.h_idx];
  t.x_history[t.h_idx] = x; t.y_history[t.h_idx] = y;
  t.h_idx = (t.h_idx + 1) % 10;

  float distanceMoved = sqrt(pow(x - x_old, 2) + pow(y - y_old, 2));
  float vY = (y - y_old) * 1.0; 
  t.smooth_doppler = (t.smooth_doppler * 0.7) + (abs(s) * 0.3);

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
      t.action = "DI CHUYỂN";
    } 
    else if (elapsed > 10000) t.action = "IMMOBILE";
    else if (elapsed > 2500) {
      if (t.smooth_doppler < 80 && distanceMoved < 200) t.action = "FALL";
      else t.action = "WAITING";
    }
    else t.action = "FALLING ???";
  } else {
    t.action = (t.smooth_doppler > 35 || distanceMoved > 120) ? "MOVING" : "STILL";
  }

  JsonObject obj = root.createNestedObject();
  obj["id"] = id; obj["x"] = x; obj["y"] = y; 
  obj["spd"] = (int)t.smooth_doppler; obj["dist"] = (int)distanceMoved; obj["a"] = t.action;
}

/*
void handleRadar() {
  static uint8_t buf[30]; static uint8_t idx = 0;
  while (RADAR_SERIAL.available()) {
    uint8_t b = RADAR_SERIAL.read();
    if (idx == 0 && b != 0xAA) continue;
    buf[idx++] = b;
    if (idx == 30) {
      if (buf[1] == 0xFF && buf[28] == 0x55) {
        StaticJsonDocument<1024> doc; JsonArray arr = doc.to<JsonArray>();
        for(int j=0; j<3; j++) {
          int off = 4 + (j*8);
          int16_t x = (buf[off+1]&0x80) ? -((buf[off+1]&0x7F)<<8|buf[off]) : ((buf[off+1]&0x7F)<<8|buf[off]);
          int16_t y = (buf[off+3]&0x80) ? -((buf[off+3]&0x7F)<<8|buf[off+2]) : ((buf[off+3]&0x7F)<<8|buf[off+2]);
          int16_t s = (buf[off+5]&0x80) ? -((buf[off+5]&0x7F)<<8|buf[off+4]) : ((buf[off+5]&0x7F)<<8|buf[off+4]);
          if(x != 0 || y != 0) processTarget(x, y, s, j+1, arr);
        }
        String out; serializeJson(doc, out); ws.textAll(out);
      }
      idx = 0;
    }
  }
}*/

// Thêm cấu trúc lưu vết người dùng ở trên cùng file C++
struct Track {
    int16_t x, y;
    uint32_t last_seen;
    bool active = false;
} tracks[4]; // Lưu cho 3 ID (1, 2, 3)

void handleRadar() {
  static uint8_t buf[30]; static uint8_t idx = 0;
  while (RADAR_SERIAL.available()) {
    uint8_t b = RADAR_SERIAL.read();
    if (idx == 0 && b != 0xAA) continue;
    buf[idx++] = b;
    if (idx == 30) {
      if (buf[1] == 0xFF && buf[28] == 0x55) {
        StaticJsonDocument<1024> doc; JsonArray arr = doc.to<JsonArray>();
        
        // Mảng tạm chứa dữ liệu thô từ Radar trong frame này
        for(int j=0; j<3; j++) {
          int off = 4 + (j*8);
          int16_t raw_x = (buf[off+1]&0x80) ? -((buf[off+1]&0x7F)<<8|buf[off]) : ((buf[off+1]&0x7F)<<8|buf[off]);
          int16_t raw_y = (buf[off+3]&0x80) ? -((buf[off+3]&0x7F)<<8|buf[off+2]) : ((buf[off+3]&0x7F)<<8|buf[off+2]);
          int16_t raw_s = (buf[off+5]&0x80) ? -((buf[off+5]&0x7F)<<8|buf[off+4]) : ((buf[off+5]&0x7F)<<8|buf[off+4]);

          if(raw_x == 0 && raw_y == 0) continue;

          // THUẬT TOÁN GÁN ID (GNN)
          int assigned_id = -1;
          float min_dist = 800.0; // Chỉ gán nếu cách vị trí cũ < 80cm

          for(int i=1; i<=3; i++) {
            if(tracks[i].active) {
                float d = sqrt(pow(raw_x - tracks[i].x, 2) + pow(raw_y - tracks[i].y, 2));
                if(d < min_dist) { min_dist = d; assigned_id = i; }
            }
          }

          // Nếu không gần ai, tìm slot trống
          if(assigned_id == -1) {
            for(int i=1; i<=3; i++) {
                if(!tracks[i].active || (millis() - tracks[i].last_seen > 3000)) {
                    assigned_id = i; break;
                }
            }
          }

          if(assigned_id != -1) {
            tracks[assigned_id].x = raw_x;
            tracks[assigned_id].y = raw_y;
            tracks[assigned_id].last_seen = millis();
            tracks[assigned_id].active = true;
            // Gọi hàm xử lý logic ngã với ID đã được khóa
            processTarget(raw_x, raw_y, raw_s, assigned_id, arr);
          }
        }
        String out; serializeJson(doc, out); ws.textAll(out);
      }
      idx = 0;
    }
  }
}

/*
void onWsEvent(AsyncWebSocket *s, AsyncWebSocketClient *c, AwsEventType t, void *arg, uint8_t *d, size_t l) {
  if (t == WS_EVT_DATA) {
    String m = ""; for(size_t i=0; i<l; i++) m += (char)d[i];
    if (m.startsWith("ZONE:")) {
      sscanf(m.substring(5).c_str(), "%hd,%hd,%hd,%hd", &safeZone.x_min, &safeZone.y_min, &safeZone.x_max, &safeZone.y_max);
      safeZone.active = true;
    } else if (m == "CLEAR") safeZone.active = false;
  }
}*/

#define MAX_ZONES 5
struct Zone {
  int16_t x_min, y_min, x_max, y_max;
  bool active = false;
} safeZones[MAX_ZONES];

int zoneCount = 0;

// Trong hàm processTarget, sửa logic kiểm tra vùng an toàn:
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

// Trong hàm onWsEvent, cập nhật logic nhận nhiều vùng:
void onWsEvent(AsyncWebSocket *s, AsyncWebSocketClient *c, AwsEventType t, void *arg, uint8_t *d, size_t l) {
  if (t == WS_EVT_DATA) {
    String m = ""; for(size_t i=0; i<l; i++) m += (char)d[i];
    
    if (m.startsWith("ADD_ZONE:")) {
      if (zoneCount < MAX_ZONES) {
        int16_t x1, y1, x2, y2;
        sscanf(m.substring(9).c_str(), "%hd,%hd,%hd,%hd", &x1, &y1, &x2, &y2);
        safeZones[zoneCount] = {x1, y1, x2, y2, true};
        zoneCount++;
        Serial.println("Đã thêm 1 vùng an toàn mới");
      }
    } 
    else if (m == "CLEAR_ALL") {
      for(int i=0; i<MAX_ZONES; i++) safeZones[i].active = false;
      zoneCount = 0;
      Serial.println("Đã xóa tất cả các vùng");
    }
  }
}

void setup() {
  Serial.begin(115200);
  RADAR_SERIAL.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);
  Serial.print("IP: "); Serial.println(WiFi.localIP());
  ws.onEvent(onWsEvent); server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *r){ r->send_P(200, "text/html", index_html); });
  server.begin();
}

void loop() { handleRadar(); ws.cleanupClients(); }
