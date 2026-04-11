#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// ================= WIFI =================
const char* ssid     = "esp32";
const char* password = "12345678";

// ================= RADAR =================
#define RADAR_SERIAL Serial2
#define RADAR_BAUD   115200
#define RADAR_RX_PIN 16
#define RADAR_TX_PIN 17

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ================= RTOS =================
QueueHandle_t xFrameQueue;   // RadarFrame
QueueHandle_t xJsonQueue;    // String*
SemaphoreHandle_t xStateMutex; // bảo vệ tstate[]
TaskHandle_t hRadarTask, hProcessTask, hWebTask;

// ================= CẤU TRÚC DỮ LIỆU =================
#define MAX_T 3

struct RawTarget {
  int16_t  x, y, s;
  bool     valid;
};

struct RadarFrame {
  RawTarget targets[MAX_T];
  uint32_t  timestamp;
};

// ================= TARGET STATE (UPDATE MỚI) =================
struct TargetState {
  int16_t  y_history[5];
  uint32_t y_hist_time[5];
  uint8_t  y_hist_idx;

  float    smooth_speed;
  int16_t  prev_Y;
  int16_t  prev_X;

  // Máy trạng thái phát hiện ngã
  enum FallState {
    NORMAL,
    SPEED_SPIKE,   // Phát hiện chuyển động nhanh (rơi vọt)
    WAIT_CONFIRM,  // Tiếp đất, nằm im chờ xác nhận
    FALLEN         // Đã ngã
  } fall_state;

  uint32_t fall_timer;
} tstate[MAX_T + 1]; // index 1..3

void initTargetStates() {
  memset(tstate, 0, sizeof(tstate));
}

// Tính vận tốc Y (mm/s) từ circular buffer lịch sử
float calcYVelocity(int id) {
  TargetState& ts = tstate[id];
  uint8_t  oldest = (ts.y_hist_idx + 1) % 5;
  uint8_t  newest = ts.y_hist_idx;
  uint32_t dt_ms  = ts.y_hist_time[newest] - ts.y_hist_time[oldest];
  if (dt_ms < 10) return 0;
  int16_t dy = ts.y_history[newest] - ts.y_history[oldest];
  return (float)dy / (float)dt_ms * 1000.0f;
}

// ================= HTML =================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
<meta charset="UTF-8">
<title>Radar Tracker</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  body {
    font-family: Arial;
    text-align: center;
    background: #0e0e0e;
    color: white;
    margin: 0;
    padding: 10px;
  }
  canvas {
    background: black;
    border-radius: 50%;
    margin-top: 10px;
  }
  .badge {
    padding: 10px 20px;
    margin: 8px;
    border-radius: 6px;
    display: inline-block;
    font-size: 16px;
    font-weight: bold;
  }
  .present { background: #1a7a1a; }
  .empty   { background: #555; }
  .fallen  { background: red; animation: blink 0.5s step-start infinite; }
  @keyframes blink { 50% { opacity: 0; } }
  #info p {
    background: #1a1a1a;
    border-radius: 6px;
    padding: 8px 16px;
    margin: 4px auto;
    max-width: 380px;
    font-size: 13px;
  }
  .fallen-text  { color: red;    font-weight: bold; }
  .falling-text { color: orange; font-weight: bold; }
</style>
</head>
<body>
<h2>Radar Tracking</h2>
<div id="presence" class="badge empty">Không có ai</div>
<br>
<canvas id="radar" width="400" height="400"></canvas>
<div id="info"></div>

<script>
let socket = new WebSocket(`ws://${location.host}/ws`);

socket.onmessage = (e) => {
  let targets = JSON.parse(e.data);
  update(targets);
  drawRadar(targets);
};

function update(targets) {
  let presence = document.getElementById("presence");
  let info = "";

  let anyFallen  = targets.some(t => t.action === "FALLEN");
  let anyFalling = targets.some(t => t.action.startsWith("FALLING?"));

  if (targets.length === 0) {
    presence.innerHTML = "Không có ai";
    presence.className = "badge empty";
  } else if (anyFallen) {
    presence.innerHTML = "⚠️ PHÁT HIỆN NGÃ!";
    presence.className = "badge fallen";
  } else {
    presence.innerHTML = "Có người (" + targets.length + ")";
    presence.className = "badge present";
  }

  targets.forEach(t => {
    let spd  = (t.s / 1000.0).toFixed(2);
    let vy   = t.vy || 0;
    let dist = Math.sqrt(t.x*t.x + t.y*t.y) / 1000.0;
    let side = t.x < -100 ? "Trái" : t.x > 100 ? "Phải" : "Giữa";
    let cls  = "";
    if      (t.action === "FALLEN")           cls = "fallen-text";
    else if (t.action.startsWith("FALLING?")) cls = "falling-text";

    info += `<p class="${cls}">
      <b>Người ${t.id}</b> | ${t.action}<br>
      Cách: ${dist.toFixed(2)}m (${side}) | Speed: ${spd} m/s | vY: ${vy} mm/s
    </p>`;
  });

  document.getElementById("info").innerHTML = info || "<i>Đang quét...</i>";
}

function drawRadar(targets) {
  let canvas = document.getElementById("radar");
  let ctx    = canvas.getContext("2d");
  ctx.clearRect(0, 0, 400, 400);

  // Vòng tròn — mỗi vòng 50px = 1.5m
  ctx.strokeStyle = "#1a331a";
  ctx.lineWidth = 1;
  for (let i = 1; i <= 4; i++) {
    ctx.beginPath();
    ctx.arc(200, 200, i * 50, 0, 2 * Math.PI);
    ctx.stroke();
    ctx.fillStyle = "#2a5c2a";
    ctx.font = "10px Arial";
    ctx.fillText((i * 1.5).toFixed(1) + "m", 203 + i * 50, 197);
  }

  // Trục
  ctx.strokeStyle = "#1a331a";
  ctx.beginPath(); ctx.moveTo(200, 0);   ctx.lineTo(200, 400); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(0,   200); ctx.lineTo(400, 200); ctx.stroke();

  // Nhãn
  ctx.fillStyle = "#3a7a3a";
  ctx.font = "11px Arial";
  ctx.fillText("Radar", 184, 216);
  ctx.fillText("Trước", 184, 18);

  let colors = ["#ff4444", "#4af", "#fa0"];

  targets.forEach(t => {
    let px = 200 + t.x * 200 / 6000;
    let py = 200 - t.y * 200 / 6000;

    let isFallen  = t.action === "FALLEN";
    let isFalling = t.action.startsWith("FALLING?");

    if (isFallen) {
      ctx.strokeStyle = "red";
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.arc(px, py, 20, 0, 2 * Math.PI);
      ctx.stroke();
    } else if (isFalling) {
      ctx.strokeStyle = "orange";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(px, py, 15, 0, 2 * Math.PI);
      ctx.stroke();
    }
    ctx.lineWidth = 1;

    ctx.fillStyle = isFallen ? "red" : colors[(t.id - 1) % 3];
    ctx.beginPath();
    ctx.arc(px, py, 8, 0, 2 * Math.PI);
    ctx.fill();

    ctx.fillStyle = "white";
    ctx.font = "bold 11px Arial";
    ctx.fillText("T" + t.id, px + 11, py - 5);
  });

  // Tâm radar
  ctx.fillStyle = "lime";
  ctx.beginPath();
  ctx.arc(200, 200, 5, 0, 2 * Math.PI);
  ctx.fill();
}
</script>
</body>
</html>
)rawliteral";

// ─────────────────────────────────────────────────────────
//  TASK 1: RADAR READ TASK
// ─────────────────────────────────────────────────────────
void RadarReadTask(void* pv) {
  uint8_t    buf[30];
  uint8_t    idx      = 0;
  bool       in_frame = false;
  RadarFrame frame;

  Serial.println("[Task1:RadarRead] Core " + String(xPortGetCoreID()) + " | Pri 3");

  for (;;) {
    if (RADAR_SERIAL.available()) {
      uint8_t b = RADAR_SERIAL.read();

      if (!in_frame) {
        if      (idx == 0 && b == 0xAA) { buf[idx++] = b; }
        else if (idx == 1 && b == 0xFF) { buf[idx++] = b; in_frame = true; }
        else                            { idx = 0; }
      } else {
        buf[idx++] = b;

        if (idx >= 30) {
          if (buf[28] == 0x55 && buf[29] == 0xCC) {
            frame.timestamp = millis();

            for (int t = 0; t < MAX_T; t++) {
              uint8_t base = 4 + t * 8;

              int16_t raw_x = (buf[base+1] & 0x80)
                ? -(int16_t)(((buf[base+1] & 0x7F) << 8) | buf[base])
                :  (int16_t)(((buf[base+1] & 0x7F) << 8) | buf[base]);

              int16_t raw_y = (buf[base+3] & 0x80)
                ? -(int16_t)(((buf[base+3] & 0x7F) << 8) | buf[base+2])
                :  (int16_t)(((buf[base+3] & 0x7F) << 8) | buf[base+2]);

              raw_y = -raw_y; // Đảo chiều Y

              int16_t raw_s = (buf[base+5] & 0x80)
                ? -(int16_t)(((buf[base+5] & 0x7F) << 8) | buf[base+4])
                :  (int16_t)(((buf[base+5] & 0x7F) << 8) | buf[base+4]);

              bool valid = !(raw_x == 0 && raw_y == 0);
              frame.targets[t] = { raw_x, raw_y, raw_s, valid };
            }

            xQueueSend(xFrameQueue, &frame, 0);
          }
          idx      = 0;
          in_frame = false;
        }
      }
    } else {
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
}

// ─────────────────────────────────────────────────────────
//  TASK 2: PROCESS TASK (UPDATE MỚI: MÁY TRẠNG THÁI CHỐNG NGÃ)
// ─────────────────────────────────────────────────────────
void ProcessTask(void* pv) {
  RadarFrame frame;
  Serial.println("[Task2:Process] Core " + String(xPortGetCoreID()) + " | Pri 2");

  for (;;) {
    if (xQueueReceive(xFrameQueue, &frame, pdMS_TO_TICKS(200)) != pdTRUE) continue;

    uint32_t now = frame.timestamp;
    String   json = "[";
    bool     any  = false;

    xSemaphoreTake(xStateMutex, portMAX_DELAY);

    for (int i = 0; i < MAX_T; i++) {
      RawTarget&  tgt = frame.targets[i];
      TargetState& ts  = tstate[i + 1];

      if (!tgt.valid) {
        // Reset trạng thái ngã nếu mất dấu (ra khỏi vùng radar)
        ts.fall_state = TargetState::NORMAL;
        continue;
      }

      // ── Cập nhật lịch sử ──
      ts.y_hist_idx = (ts.y_hist_idx + 1) % 5;
      ts.y_history[ts.y_hist_idx]   = tgt.y;
      ts.y_hist_time[ts.y_hist_idx] = now;

      float vy = calcYVelocity(i + 1);
      float spd_raw = (float)abs(tgt.s);
      
      // Khử nhiễu nhẹ cho tốc độ
      if (spd_raw < 20) spd_raw = 0;
      ts.smooth_speed = 0.7f * ts.smooth_speed + 0.3f * spd_raw;

      // ════════════════════════════════════════════════
      //  LOGIC PHÁT HIỆN NGÃ MỚI (STATE MACHINE)
      // ════════════════════════════════════════════════

      if (ts.fall_state == TargetState::NORMAL) {
        // Giai đoạn 1: Phát hiện gia tốc đột ngột (tốc độ > 800 mm/s)
        // Lưu ý: Nếu test không nhạy, sửa 800.0f thành 600.0f
        if (spd_raw > 800.0f || abs(vy) > 800.0f) {
          ts.fall_state = TargetState::SPEED_SPIKE;
          ts.fall_timer = now;
          Serial.printf("[T%d] CANH BAO ROI: spd=%.0f\n", i+1, spd_raw);
        }
      } 
      else if (ts.fall_state == TargetState::SPEED_SPIKE) {
        // Giai đoạn 2: Cho 1.5 giây để tiếp đất. Lúc này tốc độ phải tụt về gần 0.
        if (now - ts.fall_timer > 1500) {
          if (ts.smooth_speed < 40.0f) {
            ts.fall_state = TargetState::WAIT_CONFIRM;
            ts.fall_timer = now; // Bắt đầu đếm giờ nằm im
            Serial.printf("[T%d] TIEP DAT, DANG CHO XAC NHAN...\n", i+1);
          } else {
            // Vẫn di chuyển nhanh -> Chỉ là đang chạy bộ hoặc đi nhanh
            ts.fall_state = TargetState::NORMAL;
          }
        }
      } 
      else if (ts.fall_state == TargetState::WAIT_CONFIRM) {
        // Giai đoạn 3: Nằm im bất động trong 4 giây
        uint32_t elapsed = now - ts.fall_timer;
        if (ts.smooth_speed > 100.0f) {
          // Người cử động tay chân mạnh hoặc tự đứng lên được
          ts.fall_state = TargetState::NORMAL;
          Serial.printf("[T%d] HUY NGA (Co cu dong: %.0f)\n", i+1, ts.smooth_speed);
        } else if (elapsed > 4000) {
          // Hoàn toàn bất động 4 giây
          ts.fall_state = TargetState::FALLEN;
          Serial.printf("[T%d] *** XAC NHAN DA NGA! ***\n", i+1);
        }
      } 
      else if (ts.fall_state == TargetState::FALLEN) {
        // Giai đoạn 4: Đã ngã. Nếu tốc độ tăng lại > 150mm/s tức là đã đứng dậy.
        if (ts.smooth_speed > 150.0f) {
          ts.fall_state = TargetState::NORMAL;
          Serial.printf("[T%d] DA DUNG DAY AN TOAN.\n", i+1);
        }
      }

      // ── Xuất chuỗi trạng thái cho Web ──
      String action_str = "";
      if (ts.fall_state == TargetState::FALLEN) {
        action_str = "FALLEN";
      } else if (ts.fall_state == TargetState::WAIT_CONFIRM) {
        uint32_t elapsed = now - ts.fall_timer;
        action_str = "FALLING? (" + String(elapsed / 1000) + "s)";
      } else {
        int16_t deltaY = tgt.y - ts.prev_Y;
        if      (ts.smooth_speed < 30) action_str = "Dung yen";
        else if (deltaY >  50)         action_str = "Di ra xa";
        else if (deltaY < -50)         action_str = "Tien lai gan";
        else                           action_str = "Di lai";
      }

      ts.prev_Y = tgt.y;
      ts.prev_X = tgt.x;

      // Build JSON
      if (any) json += ",";
      json += "{\"id\":"   + String(i + 1)
           + ",\"x\":"     + String(tgt.x)
           + ",\"y\":"     + String(tgt.y)
           + ",\"s\":"     + String(abs(tgt.s))
           + ",\"vy\":"    + String((int)vy)
           + ",\"action\":\"" + action_str + "\""
           + "}";
      any = true;
    }

    xSemaphoreGive(xStateMutex);

    json += "]";

    String* pJson = new String(json);
    if (xQueueSend(xJsonQueue, &pJson, 0) != pdTRUE) {
      delete pJson;
    }
  }
}

// ─────────────────────────────────────────────────────────
//  TASK 3: WEB TASK
// ─────────────────────────────────────────────────────────
void WebTask(void* pv) {
  String* pJson = nullptr;

  Serial.println("[Task3:Web] Core " + String(xPortGetCoreID()) + " | Pri 1");

  for (;;) {
    if (xQueueReceive(xJsonQueue, &pJson, pdMS_TO_TICKS(100)) == pdTRUE) {
      if (pJson) {
        ws.textAll(*pJson);
        delete pJson;
        pJson = nullptr;
      }
    }

    ws.cleanupClients();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n====== Radar RTOS System ======");
  initTargetStates();

  RADAR_SERIAL.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);

  xFrameQueue = xQueueCreate(10, sizeof(RadarFrame));
  xJsonQueue  = xQueueCreate(5,  sizeof(String*));
  xStateMutex = xSemaphoreCreateMutex();

  if (!xFrameQueue || !xJsonQueue || !xStateMutex) {
    Serial.println("FATAL: Khong tao duoc RTOS primitives!");
    while (1);
  }

  WiFi.begin(ssid, password);
  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nIP: " + WiFi.localIP().toString());

  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/html", index_html);
  });
  server.begin();
  Serial.println("Server started");

  xTaskCreatePinnedToCore(RadarReadTask, "RadarRead", 4096, NULL, 3, &hRadarTask,   0);
  xTaskCreatePinnedToCore(ProcessTask,   "Process",   8192, NULL, 2, &hProcessTask, 1);
  xTaskCreatePinnedToCore(WebTask,       "Web",       4096, NULL, 1, &hWebTask,     1);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
