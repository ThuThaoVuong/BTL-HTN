#include <WiFi.h>
#include <ArduinoJson.h>
#include <vector>

// --- Cấu hình Radar & Chân cắm ---
#define RXD2 16
#define TXD2 17
#define BAUD_RADAR 256000

// --- Cấu trúc dữ liệu ---
enum EntityState { NORMAL = 0, MONITORING = 1, FALLEN = 2, FAINTED = 3, IN_BED = 4 };
const char* stateNames[] = {"DANG DI CHUYEN", "THEO DOI", "!! NGA !!", "!! NGAT !!", "NGHI NGOI"};

struct Zone {
  String name;
  int16_t xMin, xMax, yMin, yMax;
};

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
std::vector<Zone> zones;

// --- TASK 1: Đọc & Giải mã Radar (Core 0) ---
void Task_Radar(void *pv) {
  uint8_t raw[29];
  while (1) {
    if (Serial2.available() >= 29) {
      if (Serial2.read() == 0xAA && Serial2.read() == 0xFF && Serial2.read() == 0x03 && Serial2.read() == 0x00) {
        Serial2.readBytes(raw, 25); // Đọc phần còn lại của frame
        
        Serial.print(">>> RAW FRAME: "); // In dòng data thô để check
        
        for (int i = 0; i < 3; i++) {
          int b = i * 8;
          
          // 1. Giải mã thô từ các cặp byte (Little Endian)
          uint16_t x_raw = raw[b] | (raw[b+1] << 8);
          uint16_t y_raw = raw[b+2] | (raw[b+3] << 8);
          uint16_t s_raw = raw[b+4] | (raw[b+5] << 8);
          
          // 2. Xử lý bit dấu (Bit 15) theo tài liệu HLK-LD2450
          int16_t x_final = (x_raw & 0x7FFF);
          if (x_raw & 0x8000) x_final = -x_final;
          
          int16_t y_final = (y_raw & 0x7FFF);
          if (y_raw & 0x8000) y_final = -y_final;
          
          int16_t s_final = (s_raw & 0x7FFF);
          if (s_raw & 0x8000) s_final = -s_final;

          // Cập nhật dữ liệu vào biến global
          people[i].x = x_final;
          people[i].y = abs(y_final); // Y luôn là khoảng cách dương phía trước radar
          people[i].speed = s_final;
          people[i].isPresent = (people[i].y > 50 && people[i].y < 6000); 

          // Dòng in Frame Data cụ thể theo yêu cầu của bạn
          Serial.printf("T[%d]:(X:%d, Y:%d, S:%d) | ", i, people[i].x, people[i].y, people[i].speed);
        }
        Serial.println(); // Xuống dòng sau khi in xong 3 target
      }
    }
    vTaskDelay(5 / portTICK_PERIOD_MS); // Quét nhanh hơn để không lỡ frame
  }
}

// --- TASK 2: Logic xử lý trạng thái & Báo cáo (Core 0) ---
void Task_Logic(void *pv) {
  while (1) {
    uint32_t now = millis();
    static uint32_t lastPrint = 0;

    for (int i = 0; i < 3; i++) {
      if (!people[i].isPresent) continue;

      // Kiểm tra vùng an toàn
      bool inSafeZone = false;
      String zoneName = "";
      for (auto &z : zones) {
        if (people[i].x >= z.xMin && people[i].x <= z.xMax && people[i].y >= z.yMin && people[i].y <= z.yMax) {
          inSafeZone = true; zoneName = z.name; break;
        }
      }

      if (inSafeZone) {
        people[i].state = IN_BED;
        people[i].lastMoveTime = now;
      } else {
        // Thuật toán phát hiện Ngã/Ngất
        if (abs(people[i].speed) > 750) { // Ngưỡng vận tốc biến động mạnh
          people[i].hasSpike = true;
          people[i].spikeTime = now;
        }

        if (abs(people[i].speed) > 150) { // Có chuyển động thường
          people[i].lastMoveTime = now;
          people[i].state = NORMAL;
        } else {
          uint32_t idleTime = now - people[i].lastMoveTime;
          // Nếu có cú va chạm (spike) + bất động lâu = NGÃ
          if (people[i].hasSpike && (now - people[i].spikeTime > 2000) && idleTime > 15000) {
            people[i].state = FALLEN;
          } 
          // Nếu chỉ bất động cực lâu (> 60s) = NGẤT/QUÊN
          else if (idleTime > 60000) {
            people[i].state = FAINTED;
          } else {
            people[i].state = MONITORING;
          }
        }
      }
    }

    // In báo cáo tóm tắt mỗi 2 giây để không chiếm dụng Console
    if (now - lastPrint > 2000) {
      Serial.println("\n--- [ SUMMARY REPORT ] ---");
      for (int i = 0; i < 3; i++) {
        if (people[i].isPresent) {
          Serial.printf("Target %d -> Trang thai: %s \n", i, stateNames[people[i].state]);
        }
      }
      lastPrint = now;
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void setup() {
  // Console để debug
  Serial.begin(115200);
  // Cổng kết nối Radar LD2450
  Serial2.begin(BAUD_RADAR, SERIAL_8N1, RXD2, TXD2);

  // FIX CỨNG VÙNG AN TOÀN (Dựa trên thông số bạn đã test)
  // Ví dụ: Giường nằm ở khoảng cách 30-45cm trước mặt radar
  zones.push_back({"Giuong Cua Toi", -300, 300, 200, 600});

  // Chế độ phát Wifi để sau này dùng Web
  WiFi.softAP("MatThan_Radar_v2", "88888888");

  Serial.println("\n==============================");
  Serial.println("HE THONG MAT THAN KHOI DONG...");
  Serial.println("==============================");

  // Chạy Task trên Core 0
  xTaskCreatePinnedToCore(Task_Radar, "Task_Radar", 4096, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(Task_Logic, "Task_Logic", 4096, NULL, 2, NULL, 0);
}

void loop() {
  // Để trống vì đã dùng FreeRTOS Task
}