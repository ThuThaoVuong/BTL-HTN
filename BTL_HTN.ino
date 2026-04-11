#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <math.h>

// ================= CẤU HÌNH WIFI =================
const char* ssid = "esp32";
const char* password = "12345678";

// ================= PHẦN CỨNG RADAR =================
#define RADAR_SERIAL Serial2
#define RADAR_BAUD   115200 // Đã sửa thành 115200 theo đúng tần số Radar của bạn
#define RADAR_RX_PIN 16     // Nối với dây TX của Radar
#define RADAR_TX_PIN 17     // Nối với dây RX của Radar

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ================= GIAO DIỆN HTML & WEB APP =================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta charset="UTF-8">
  <title>ESP32 Radar Dual Tracker</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: 'Segoe UI', Arial, sans-serif; text-align: center; background: #0e0e0e; color: white; margin: 0; padding: 20px; }
    .container { max-width: 600px; margin: auto; }
    canvas { background: #000; border: 2px solid #333; border-radius: 50%; box-shadow: 0 0 20px rgba(0,255,0,0.1); width: 100%; max-width: 400px; }
    .legend { display: flex; justify-content: center; gap: 20px; margin: 15px 0; font-weight: bold; }
    .dot { width: 12px; height: 12px; border-radius: 50%; display: inline-block; }
    .p1 { background: #ff4757; } .p2 { background: #1e90ff; }
    #status { font-family: monospace; color: #00ff00; background: #1a1a1a; padding: 10px; border-radius: 5px; margin-top: 15px; font-size: 14px;}
  </style>
</head>
<body>
  <div class="container">
    <h2>Bản đồ Radar Trực tiếp (2 Mục Tiêu)</h2>
    <div class="legend">
        <span><span class="dot p1"></span> Người 1</span>
        <span><span class="dot p2"></span> Người 2</span>
    </div>
    <canvas id="radarCanvas" width="400" height="400"></canvas>
    <div id="status">Đang đợi dữ liệu Radar...</div>
  </div>

  <script>
    var canvas = document.getElementById('radarCanvas');
    var ctx = canvas.getContext('2d');
    var gateway = `ws://${window.location.hostname}/ws`;
    var websocket;

    window.onload = () => { 
        websocket = new WebSocket(gateway); 
        websocket.onmessage = (event) => {
            var allTargets = JSON.parse(event.data);
            
            // Chỉ lấy những người có tọa độ hợp lệ (khác 0,0)
            var validTargets = allTargets.filter(t => t.x !== 0 || t.y !== 0);
            updateUI(validTargets);
            drawRadar(validTargets);
        };
    };

    function updateUI(targets) {
        let text = "";
        targets.forEach(t => {
            text += `P${t.id}: [X: ${t.x}, Y: ${t.y}] | `;
        });
        document.getElementById('status').innerHTML = text || "Không có mục tiêu nào";
    }

    function drawRadar(targets) {
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        
        // Vẽ Lưới Radar
        ctx.strokeStyle = "#1a331a";
        for(let i=1; i<=4; i++) {
            ctx.beginPath(); ctx.arc(200, 200, i*50, 0, 2*Math.PI); ctx.stroke();
        }
        ctx.beginPath(); ctx.moveTo(200, 0); ctx.lineTo(200, 400); ctx.stroke();
        ctx.beginPath(); ctx.moveTo(0, 200); ctx.lineTo(400, 200); ctx.stroke();

        // Vẽ các mục tiêu
        targets.forEach(t => {
            let drawX = 200 + (t.x * 200 / 6000);
            let drawY = 200 - (t.y * 200 / 6000); 

            // Cài đặt màu: Người 1 Đỏ, Người 2 Xanh dương
            ctx.fillStyle = (t.id === 1) ? "#ff4757" : "#1e90ff";
            
            ctx.beginPath();
            ctx.arc(drawX, drawY, 9, 0, 2*Math.PI);
            ctx.fill();
            
            ctx.fillStyle = "white";
            ctx.fillText("P" + t.id, drawX + 12, drawY - 12);
        });

        // Vẽ Tâm Radar
        ctx.fillStyle = "#00ff00";
        ctx.beginPath(); ctx.arc(200, 200, 4, 0, 2*Math.PI); ctx.fill();
    }
  </script>
</body>
</html>)rawliteral";

// ================= XỬ LÝ DỮ LIỆU RADAR =================

// Hàm bóc tách dữ liệu Radar
void parseTarget(uint8_t* buf, int offset, int id, String &json) {
    int16_t x = (buf[offset+1] & 0x80) ? -(int16_t)(((buf[offset+1] & 0x7F) << 8) | buf[offset]) : (int16_t)(((buf[offset+1] & 0x7F) << 8) | buf[offset]);
    int16_t y = (buf[offset+3] & 0x80) ? -(int16_t)(((buf[offset+3] & 0x7F) << 8) | buf[offset+2]) : (int16_t)(((buf[offset+3] & 0x7F) << 8) | buf[offset+2]);
    y = -y; // Đảo chiều Y cho đúng hướng nhìn

    if (json != "[") json += ",";
    json += "{\"id\":" + String(id) + ",\"x\":" + String(x) + ",\"y\":" + String(y) + "}";
}

void handleRadar() {
    static uint8_t buf[30];
    static uint8_t idx = 0;

    while (RADAR_SERIAL.available()) {
        uint8_t b = RADAR_SERIAL.read();
        if (idx == 0 && b != 0xAA) continue;
        buf[idx++] = b;

        if (idx >= 30) {
            if (buf[1] == 0xFF && buf[28] == 0x55) {
                String json = "[";
                parseTarget(buf, 4, 1, json);  // Đọc người 1
                parseTarget(buf, 12, 2, json); // Đọc người 2
                json += "]";

                ws.textAll(json); // Đẩy dữ liệu lên web
            }
            idx = 0;
        }
    }
}

// ================= SETUP VÀ LOOP =================
void setup() {
    Serial.begin(115200);
    // Lưu ý: Đã đổi RADAR_BAUD thành 115200 để khớp với tín hiệu mạch của bạn
    RADAR_SERIAL.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
    Serial.print("\nIP: "); Serial.println(WiFi.localIP());

    server.addHandler(&ws);
    
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/html; charset=utf-8", index_html);
    });

    server.begin();
}

void loop() {
    handleRadar();
    ws.cleanupClients();
}