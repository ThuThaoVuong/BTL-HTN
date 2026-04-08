#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <vector>

// --- Cấu hình Radar ---
#define RXD2 16
#define TXD2 17
#define BAUD_RADAR 256000

// --- Cấu trúc dữ liệu ---
enum EntityState { NORMAL = 0, MONITORING = 1, FALLEN = 2, FAINTED = 3, IN_BED = 4 };
struct Person {
  int16_t x, y, speed;
  EntityState state;
  uint32_t lastMoveTime;
  uint32_t spikeTime;
  bool hasSpike;
  bool isPresent;
};

// Biến toàn cục
Person people[3];
AsyncWebServer server(80);

// --- TASK 1: Đọc Radar (Core 0) ---
void Task_Radar(void *pv) {
  uint8_t raw[29];
  while (1) {
    if (Serial2.available() >= 29) {
      if (Serial2.read() == 0xAA && Serial2.read() == 0xFF && Serial2.read() == 0x03 && Serial2.read() == 0x00) {
        Serial2.readBytes(raw, 25);
        for (int i = 0; i < 3; i++) {
          int b = i * 8;
          uint16_t x_raw = raw[b] | (raw[b+1] << 8);
          uint16_t y_raw = raw[b+2] | (raw[b+3] << 8);
          uint16_t s_raw = raw[b+4] | (raw[b+5] << 8);

          int16_t x_final = (x_raw & 0x7FFF);
          if (x_raw & 0x8000) x_final = -x_final;
          int16_t y_final = (y_raw & 0x7FFF);
          if (y_raw & 0x8000) y_final = -y_final;
          
          people[i].x = x_final;
          people[i].y = abs(y_final);
          people[i].speed = (int16_t)(s_raw & 0x7FFF);
          people[i].isPresent = (people[i].y > 50 && people[i].y < 6000);
        }
      }
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

// --- TASK 2: Logic xử lý trạng thái (Core 0) ---
void Task_Logic(void *pv) {
  while (1) {
    uint32_t now = millis();
    for (int i = 0; i < 3; i++) {
      if (!people[i].isPresent) continue;
      
      // Thuật toán đơn giản (Bạn có thể thêm Zone vào đây sau)
      if (abs(people[i].speed) > 750) { people[i].hasSpike = true; people[i].spikeTime = now; }
      if (abs(people[i].speed) > 150) {
        people[i].lastMoveTime = now;
        people[i].state = NORMAL;
      } else {
        uint32_t idle = now - people[i].lastMoveTime;
        if (people[i].hasSpike && (now - people[i].spikeTime > 2000) && idle > 15000) people[i].state = FALLEN;
        else if (idle > 60000) people[i].state = FAINTED;
        else people[i].state = MONITORING;
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// --- THIẾT LẬP WEB SERVER (BACKEND) ---
void setup() {
  Serial.begin(115200);
  Serial2.begin(BAUD_RADAR, SERIAL_8N1, RXD2, TXD2);

  // 1. Khởi tạo LittleFS (Lưu file Web)
  if (!LittleFS.begin(true)) { Serial.println("LittleFS Error!"); return; }

  // 2. Phát WiFi
  WiFi.softAP("MatThan_Radar", "88888888");
  Serial.println("IP Address: 192.168.4.1");

  // 3. API ENDPOINT: Trả dữ liệu JSON cho Frontend (Trưởng nhóm yêu cầu)
  server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request) {
    StaticJsonDocument<512> doc;
    JsonArray array = doc.createNestedArray("people");

    for (int i = 0; i < 3; i++) {
      JsonObject obj = array.createNestedObject();
      obj["id"] = i;
      obj["x"] = people[i].x;
      obj["y"] = people[i].y;
      obj["s"] = people[i].speed;
      obj["st"] = (int)people[i].state;
      obj["active"] = people[i].isPresent;
    }

    String jsonResponse;
    serializeJson(doc, jsonResponse);
    request->send(200, "application/json", jsonResponse);
  });

  // 4. API ENDPOINT: Reset cảnh báo
  server.on("/reset-alert", HTTP_GET, [](AsyncWebServerRequest *request) {
    for (int i = 0; i < 3; i++) people[i].hasSpike = false;
    request->send(200, "text/plain", "Đã reset cảnh báo");
  });

  // 5. API ENDPOINT: Lưu zones vào LittleFS
  server.on("/save-zones", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      StaticJsonDocument<1024> doc;
      deserializeJson(doc, (const char*)data);

      JsonArray zonesArray = doc["zones"];
      if (zonesArray.size() > 0) {
        File f = LittleFS.open("/zones.json", "w");
        if (f) {
          serializeJson(doc, f);
          f.close();
          request->send(200, "application/json", "{\"status\":\"ok\"}");
          Serial.println("Zones saved!");
          return;
        }
      }
      request->send(400, "application/json", "{\"error\":\"Save failed\"}");
    });

  // 6. API ENDPOINT: Tải zones từ LittleFS
  server.on("/load-zones", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (LittleFS.exists("/zones.json")) {
      File f = LittleFS.open("/zones.json", "r");
      String content = f.readString();
      f.close();
      request->send(200, "application/json", content);
    } else {
      request->send(200, "application/json", "{\"zones\":[]}");
    }
  });

  // 7. API ENDPOINT: Reset config (xóa zones)
  server.on("/reset-config", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (LittleFS.exists("/zones.json")) {
      LittleFS.remove("/zones.json");
    }
    request->send(200, "text/plain", "Config reset");
    Serial.println("Config reset!");
  });

  // 8. Trả file giao diện (Người B nạp vào thư mục data)
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  server.begin();

  // Chạy Task trên Core 0 để dành Core 1 cho WiFi/Web
  xTaskCreatePinnedToCore(Task_Radar, "Radar", 4096, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(Task_Logic, "Logic", 4096, NULL, 2, NULL, 0);
}

void loop() {}