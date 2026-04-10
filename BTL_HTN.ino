// ================== FIXED VERSION ==================
// ✔ Fix trục Y
// ✔ Fix hiển thị trước/sau
// ===================================================

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <math.h>

// ================= HARDWARE =================
#define RADAR_SERIAL Serial2
#define RADAR_BAUD   256000
#define RADAR_RX_PIN 16
#define RADAR_TX_PIN 17
#define DEBUG_SERIAL Serial

// ================= CONFIG =================
#define MAX_TARGETS 3
#define FRAME_HEADER_1 0xAA
#define FRAME_HEADER_2 0xFF
#define FRAME_TAIL_1   0x55
#define FRAME_TAIL_2   0xCC
#define FRAME_LENGTH   30

// ================= ZONE =================
struct Zone {
  int16_t x_min, x_max;
  int16_t y_min, y_max;
  const char* name;
};

const Zone ZONES[] = {
  { -1500, 1500,    0, 2000, "ZONE_NEAR" },
  { -2000, 2000, 2000, 4000, "ZONE_MID"  },
  { -2500, 2500, 4000, 6000, "ZONE_FAR"  },
};
#define NUM_ZONES (sizeof(ZONES)/sizeof(ZONES[0]))

// ================= DATA =================
struct Target {
  int16_t x, y, speed;
  bool valid;
};

struct RadarFrame {
  Target targets[MAX_TARGETS];
  uint32_t timestamp;
};

// ================= RTOS =================
QueueHandle_t xRadarQueue;

// ================= UTIL =================
uint8_t getZone(int16_t x, int16_t y) {
  for (uint8_t i = 0; i < NUM_ZONES; i++) {
    if (x >= ZONES[i].x_min && x <= ZONES[i].x_max &&
        y >= ZONES[i].y_min && y <= ZONES[i].y_max)
      return i;
  }
  return 0xFF;
}

// ================= TASK 1: READ =================
void RadarReadTask(void* pv) {
  uint8_t buf[FRAME_LENGTH];
  uint8_t idx = 0;
  bool in_frame = false;

  RadarFrame frame;

  for (;;) {
    if (RADAR_SERIAL.available()) {
      uint8_t b = RADAR_SERIAL.read();

      if (!in_frame) {
        if (idx == 0 && b == FRAME_HEADER_1) buf[idx++] = b;
        else if (idx == 1 && b == FRAME_HEADER_2) {
          buf[idx++] = b;
          in_frame = true;
        } else idx = 0;
      } else {
        buf[idx++] = b;

        if (idx >= FRAME_LENGTH) {
          if (buf[28] == FRAME_TAIL_1 && buf[29] == FRAME_TAIL_2) {

            frame.timestamp = millis();

            for (int t = 0; t < MAX_TARGETS; t++) {
              uint8_t base = 4 + t * 8;

              int16_t raw_x = (buf[base+1] & 0x80)
                ? -(int16_t)(((buf[base+1] & 0x7F) << 8) | buf[base])
                :  (int16_t)(((buf[base+1] & 0x7F) << 8) | buf[base]);

              int16_t raw_y = (buf[base+3] & 0x80)
                ? -(int16_t)(((buf[base+3] & 0x7F) << 8) | buf[base+2])
                :  (int16_t)(((buf[base+3] & 0x7F) << 8) | buf[base+2]);

              // 🔥 FIX QUAN TRỌNG
              raw_y = -raw_y;

              int16_t raw_s = (buf[base+5] & 0x80)
                ? -(int16_t)(((buf[base+5] & 0x7F) << 8) | buf[base+4])
                :  (int16_t)(((buf[base+5] & 0x7F) << 8) | buf[base+4]);

              bool valid = !(raw_x == 0 && raw_y == 0);

              frame.targets[t] = { raw_x, raw_y, raw_s, valid };
            }

            xQueueSend(xRadarQueue, &frame, 0);
          }
          idx = 0;
          in_frame = false;
        }
      }
    } else vTaskDelay(1);
  }
}

// ================= TASK 2: OUTPUT =================
void OutputTask(void* pv) {
  RadarFrame frame;

  for (;;) {
    if (xQueueReceive(xRadarQueue, &frame, portMAX_DELAY)) {

      Serial.println("\n==========================");

      for (int i = 0; i < MAX_TARGETS; i++) {
        auto& t = frame.targets[i];
        if (!t.valid) continue;

        float dist = sqrt(t.x * t.x + t.y * t.y) / 1000.0;

        const char* side =
          (t.x < -100) ? "Trai" :
          (t.x >  100) ? "Phai" : "Giua";

        // 🔥 FIX HIỂN THỊ
        const char* front_back =
          (t.y >= 0) ? "phia truoc" : "phia sau";

        uint8_t zone = getZone(t.x, t.y);
        const char* zone_name =
          (zone != 0xFF) ? ZONES[zone].name : "Ngoai vung";

        Serial.printf("Nguoi #%d\n", i+1);
        Serial.printf("  X=%d mm | Y=%d mm\n", t.x, t.y);
        Serial.printf("  Khoang cach: %.2fm %s (%s)\n",
                      dist, front_back, side);
        Serial.printf("  Zone: %s\n", zone_name);
      }
    }
  }
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  RADAR_SERIAL.begin(RADAR_BAUD, SERIAL_8N1,
                     RADAR_RX_PIN, RADAR_TX_PIN);

  xRadarQueue = xQueueCreate(10, sizeof(RadarFrame));

  xTaskCreatePinnedToCore(RadarReadTask, "Radar", 4096, NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(OutputTask,    "Output",4096, NULL, 1, NULL, 1);

  Serial.println("Radar started...");
}

void loop() {}