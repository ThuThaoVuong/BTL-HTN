/*
 * HLK-LD2450 Full-Feature Radar System
 * ESP32 + FreeRTOS — 3 Tasks Architecture
 *
 * Task 1: RadarReadTask   — Đọc & parse dữ liệu từ LD2450 qua UART
 * Task 2: ProcessingTask  — Xử lý tracking, zone, event detection, statistics
 * Task 3: OutputTask      — Log, LED, cảnh báo, MQTT/Serial output
 *
 * Phần cứng:
 *   ESP32 DevKit
 *   HLK-LD2450 kết nối UART2 (GPIO16=RX, GPIO17=TX)
 *   LED_BUILTIN hoặc GPIO2
 */

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include <math.h>

// ─────────────────────────────────────────────
//  CẤU HÌNH PHẦN CỨNG
// ─────────────────────────────────────────────
#define RADAR_SERIAL      Serial2
#define RADAR_BAUD        256000
#define RADAR_RX_PIN      16
#define RADAR_TX_PIN      17
#define DEBUG_SERIAL      Serial
#define LED_PIN           2
#define BUZZER_PIN        4

// ─────────────────────────────────────────────
//  CẤU HÌNH LD2450
// ─────────────────────────────────────────────
#define MAX_TARGETS       3
#define FRAME_HEADER_1    0xAA
#define FRAME_HEADER_2    0xFF
#define FRAME_TAIL_1      0x55
#define FRAME_TAIL_2      0xCC
#define FRAME_LENGTH      30

// ─────────────────────────────────────────────
//  CẤU HÌNH ZONE (mm)
// ─────────────────────────────────────────────
struct Zone {
  int16_t x_min, x_max;
  int16_t y_min, y_max;
  const char* name;
};

const Zone ZONES[] = {
  { -1500,  1500,    0, 2000, "ZONE_NEAR" },
  { -2000,  2000, 2000, 4000, "ZONE_MID"  },
  { -2500,  2500, 4000, 6000, "ZONE_FAR"  },
  {  -800,   800,  200, 1500, "ZONE_DESK" },
};
#define NUM_ZONES (sizeof(ZONES)/sizeof(ZONES[0]))

// ─────────────────────────────────────────────
//  CẤU TRÚC DỮ LIỆU
// ─────────────────────────────────────────────
struct Target {
  int16_t  x;
  int16_t  y;
  int16_t  speed;
  uint16_t resolution;
  bool     valid;
};

struct RadarFrame {
  Target   targets[MAX_TARGETS];
  uint32_t timestamp;
  uint8_t  target_count;
};

#define TRAJ_MAX_POINTS 50
struct TrackedObject {
  uint8_t  id;
  int16_t  x, y;
  int16_t  speed;
  float    velocity_x, velocity_y;
  float    acceleration;
  int16_t  traj_x[TRAJ_MAX_POINTS];
  int16_t  traj_y[TRAJ_MAX_POINTS];
  uint32_t traj_time[TRAJ_MAX_POINTS];
  uint8_t  traj_len;
  uint8_t  traj_head;
  enum Action { UNKNOWN, WALKING, RUNNING, STANDING, SITTING, FALLEN } action;
  uint32_t still_since;
  uint32_t last_seen;
  bool     active;
  uint32_t presence_start;
  float    total_distance;
};

struct AnomalyEvent {
  enum Type {
    FALL_DETECTED,
    LONG_STILL,
    FAST_MOVEMENT,
    ZONE_ENTER,
    ZONE_EXIT,
    NEW_TARGET,
    LOST_TARGET
  } type;
  uint8_t  target_id;
  uint8_t  zone_id;
  int16_t  x, y;
  float    value;
  uint32_t timestamp;
  char     message[64];
};

struct Statistics {
  uint32_t total_detections;
  uint32_t total_events;
  uint32_t zone_time[NUM_ZONES];
  uint8_t  max_simultaneous;
  float    avg_speed;
  uint32_t uptime_start;
};

// ─────────────────────────────────────────────
//  QUEUES & SEMAPHORES & SHARED STATE
// ─────────────────────────────────────────────
QueueHandle_t     xRadarQueue;
QueueHandle_t     xAnomalyQueue;
SemaphoreHandle_t xStatsMutex;
SemaphoreHandle_t xObjectMutex;

TrackedObject g_objects[MAX_TARGETS];
Statistics    g_stats;
bool          g_presence;
uint8_t       g_zone_mask;

TaskHandle_t hRadarTask, hProcessTask, hOutputTask;

// ─────────────────────────────────────────────
//  TIỆN ÍCH
// ─────────────────────────────────────────────
float distance(int16_t x1, int16_t y1, int16_t x2, int16_t y2) {
  float dx = x1 - x2, dy = y1 - y2;
  return sqrtf(dx*dx + dy*dy);
}

uint8_t getZone(int16_t x, int16_t y) {
  for (uint8_t i = 0; i < NUM_ZONES; i++) {
    if (x >= ZONES[i].x_min && x <= ZONES[i].x_max &&
        y >= ZONES[i].y_min && y <= ZONES[i].y_max) {
      return i;
    }
  }
  return 0xFF;
}

const char* actionName(TrackedObject::Action a) {
  switch (a) {
    case TrackedObject::WALKING:  return "WALKING";
    case TrackedObject::RUNNING:  return "RUNNING";
    case TrackedObject::STANDING: return "STANDING";
    case TrackedObject::SITTING:  return "SITTING";
    case TrackedObject::FALLEN:   return "FALLEN!";
    default:                      return "UNKNOWN";
  }
}

// ─────────────────────────────────────────────
//  TASK 1: RADAR READ TASK
//  Core 0 | Priority 3 — đọc UART, parse frame
// ─────────────────────────────────────────────
void RadarReadTask(void* pvParam) {
  uint8_t    buf[FRAME_LENGTH];
  uint8_t    idx = 0;
  bool       in_frame = false;
  RadarFrame frame;

  DEBUG_SERIAL.println("[RadarReadTask] Started on core " + String(xPortGetCoreID()));

  for (;;) {
    if (RADAR_SERIAL.available()) {
      uint8_t b = RADAR_SERIAL.read();

      if (!in_frame) {
        if (idx == 0 && b == FRAME_HEADER_1) {
          buf[idx++] = b;
        } else if (idx == 1 && b == FRAME_HEADER_2) {
          buf[idx++] = b;
          in_frame = true;
        } else {
          idx = 0;
        }
      } else {
        buf[idx++] = b;

        if (idx >= FRAME_LENGTH) {
          if (buf[FRAME_LENGTH-2] == FRAME_TAIL_1 &&
              buf[FRAME_LENGTH-1] == FRAME_TAIL_2) {

            frame.timestamp    = millis();
            frame.target_count = 0;

            for (uint8_t t = 0; t < MAX_TARGETS; t++) {
              uint8_t base = 4 + t * 8;

              // Decode đúng theo datasheet LD2450:
              // Byte cao bit7=1 → âm, lấy 15 bit còn lại làm magnitude
              int16_t raw_x = (buf[base+1] & 0x80)
                ? -(int16_t)(((buf[base+1] & 0x7F) << 8) | buf[base])
                :  (int16_t)(((buf[base+1] & 0x7F) << 8) | buf[base]);
              int16_t raw_y = (buf[base+3] & 0x80)
                ? -(int16_t)(((buf[base+3] & 0x7F) << 8) | buf[base+2])
                :  (int16_t)(((buf[base+3] & 0x7F) << 8) | buf[base+2]);
              int16_t raw_s = (buf[base+5] & 0x80)
                ? -(int16_t)(((buf[base+5] & 0x7F) << 8) | buf[base+4])
                :  (int16_t)(((buf[base+5] & 0x7F) << 8) | buf[base+4]);
              uint16_t res = (uint16_t)(buf[base+6] | (buf[base+7] << 8));

              bool valid = !(raw_x == 0 && raw_y == 0);
              frame.targets[t] = { raw_x, raw_y, raw_s, res, valid };
              if (valid) frame.target_count++;
            }

            xQueueSend(xRadarQueue, &frame, 0);
          }

          in_frame = false;
          idx = 0;
        }
      }
    } else {
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
}

// ─────────────────────────────────────────────
//  TASK 2: PROCESSING TASK
//  Core 1 | Priority 2 — tracking, zone, anomaly
// ─────────────────────────────────────────────
#define SPEED_WALKING_MIN   20
#define SPEED_RUNNING_MIN   100
#define SPEED_FALL_SPIKE    200
#define STILL_THRESHOLD     10
#define STILL_LONG_MS       30000UL
#define FALL_Y_DROP_MM      600

void sendAnomaly(AnomalyEvent::Type type, uint8_t tid, uint8_t zid,
                 int16_t x, int16_t y, float val, const char* msg) {
  AnomalyEvent ev;
  ev.type      = type;
  ev.target_id = tid;
  ev.zone_id   = zid;
  ev.x = x; ev.y = y;
  ev.value     = val;
  ev.timestamp = millis();
  strncpy(ev.message, msg, 63);
  xQueueSend(xAnomalyQueue, &ev, 0);
}

void updateTrajectory(TrackedObject& obj, int16_t x, int16_t y, uint32_t t) {
  obj.traj_head = (obj.traj_head + 1) % TRAJ_MAX_POINTS;
  obj.traj_x[obj.traj_head] = x;
  obj.traj_y[obj.traj_head] = y;
  obj.traj_time[obj.traj_head] = t;
  if (obj.traj_len < TRAJ_MAX_POINTS) obj.traj_len++;
}

void classifyAction(TrackedObject& obj, uint32_t now) {
  float spd = fabsf(obj.speed);
  int16_t prev_y = obj.traj_y[(obj.traj_head + TRAJ_MAX_POINTS - 1) % TRAJ_MAX_POINTS];

  if (obj.traj_len >= 2) {
    int16_t dy = prev_y - obj.y;
    if (dy > FALL_Y_DROP_MM && spd > SPEED_FALL_SPIKE/2) {
      obj.action = TrackedObject::FALLEN;
      return;
    }
  }

  if (spd < STILL_THRESHOLD) {
    if (obj.action != TrackedObject::STANDING &&
        obj.action != TrackedObject::SITTING  &&
        obj.action != TrackedObject::FALLEN) {
      obj.still_since = now;
    }
    obj.action = TrackedObject::STANDING;
  } else if (spd < SPEED_RUNNING_MIN) {
    obj.action = TrackedObject::WALKING;
  } else {
    obj.action = TrackedObject::RUNNING;
  }
}

void ProcessingTask(void* pvParam) {
  RadarFrame frame;
  uint8_t    prev_zone[MAX_TARGETS];

  memset(prev_zone, 0xFF, sizeof(prev_zone));

  DEBUG_SERIAL.println("[ProcessingTask] Started on core " + String(xPortGetCoreID()));

  for (;;) {
    if (xQueueReceive(xRadarQueue, &frame, pdMS_TO_TICKS(100)) == pdTRUE) {
      uint32_t now = frame.timestamp;

      xSemaphoreTake(xObjectMutex, portMAX_DELAY);

      for (uint8_t t = 0; t < MAX_TARGETS; t++) {
        TrackedObject& obj = g_objects[t];
        Target&        tgt = frame.targets[t];

        if (!tgt.valid) {
          if (obj.active) {
            obj.active = false;
            uint32_t dur = now - obj.presence_start;
            sendAnomaly(AnomalyEvent::LOST_TARGET, t, 0xFF, obj.x, obj.y,
                        dur / 1000.0f, "Target lost");
          }
          continue;
        }

        if (!obj.active) {
          obj.active         = true;
          obj.id             = t;
          obj.still_since    = now;
          obj.presence_start = now;
          obj.total_distance = 0;
          obj.traj_len       = 0;
          obj.traj_head      = 0;
          obj.action         = TrackedObject::UNKNOWN;

          xSemaphoreTake(xStatsMutex, portMAX_DELAY);
          g_stats.total_detections++;
          xSemaphoreGive(xStatsMutex);

          sendAnomaly(AnomalyEvent::NEW_TARGET, t, getZone(tgt.x, tgt.y),
                      tgt.x, tgt.y, 0, "New target detected");
        }

        // Velocity & acceleration (low-pass filter)
        float dt = 0.1f;
        if (obj.last_seen > 0) {
          dt = (now - obj.last_seen) / 1000.0f;
          if (dt < 0.001f) dt = 0.001f;
        }
        float vx = (tgt.x - obj.x) / dt;
        float vy = (tgt.y - obj.y) / dt;
        float v  = sqrtf(vx*vx + vy*vy);
        float prev_v = sqrtf(obj.velocity_x*obj.velocity_x + obj.velocity_y*obj.velocity_y);
        float raw_acc    = (v - prev_v) / dt;
        obj.acceleration = obj.acceleration * 0.9f + raw_acc * 0.1f;
        obj.velocity_x   = vx;
        obj.velocity_y   = vy;

        obj.total_distance += distance(tgt.x, tgt.y, obj.x, obj.y);
        obj.x     = tgt.x;
        obj.y     = tgt.y;
        obj.speed = tgt.speed;
        obj.last_seen = now;

        updateTrajectory(obj, tgt.x, tgt.y, now);
        classifyAction(obj, now);

        uint8_t cur_zone = getZone(tgt.x, tgt.y);
        if (cur_zone != prev_zone[t]) {
          if (prev_zone[t] != 0xFF)
            sendAnomaly(AnomalyEvent::ZONE_EXIT,  t, prev_zone[t], tgt.x, tgt.y, 0, "Zone exit");
          if (cur_zone != 0xFF)
            sendAnomaly(AnomalyEvent::ZONE_ENTER, t, cur_zone,     tgt.x, tgt.y, 0, "Zone enter");
          prev_zone[t] = cur_zone;
        }

        if (cur_zone != 0xFF) {
          xSemaphoreTake(xStatsMutex, portMAX_DELAY);
          g_stats.zone_time[cur_zone] += (uint32_t)(dt * 1000);
          xSemaphoreGive(xStatsMutex);
        }

        if (obj.action == TrackedObject::FALLEN) {
          sendAnomaly(AnomalyEvent::FALL_DETECTED, t, cur_zone,
                      tgt.x, tgt.y, fabsf(obj.speed), "FALL DETECTED!");
        }

        if (obj.action == TrackedObject::STANDING &&
            (now - obj.still_since) > STILL_LONG_MS) {
          static uint32_t last_still_warn[MAX_TARGETS] = {0};
          if (now - last_still_warn[t] > 10000) {
            float still_sec = (now - obj.still_since) / 1000.0f;
            sendAnomaly(AnomalyEvent::LONG_STILL, t, cur_zone,
                        tgt.x, tgt.y, still_sec, "Person standing still too long");
            last_still_warn[t] = now;
          }
        }

        if (fabsf(obj.speed) > SPEED_FALL_SPIKE) {
          sendAnomaly(AnomalyEvent::FAST_MOVEMENT, t, cur_zone,
                      tgt.x, tgt.y, obj.speed, "Abnormal fast movement");
        }
      }

      uint8_t cnt = 0, zmask = 0;
      for (uint8_t t = 0; t < MAX_TARGETS; t++) {
        if (g_objects[t].active) {
          cnt++;
          uint8_t z = getZone(g_objects[t].x, g_objects[t].y);
          if (z != 0xFF) zmask |= (1 << z);
        }
      }
      g_presence  = (cnt > 0);
      g_zone_mask = zmask;

      xSemaphoreTake(xStatsMutex, portMAX_DELAY);
      if (cnt > g_stats.max_simultaneous) g_stats.max_simultaneous = cnt;
      xSemaphoreGive(xStatsMutex);

      xSemaphoreGive(xObjectMutex);
    }
  }
}

// ─────────────────────────────────────────────
//  TASK 3: OUTPUT TASK
//  Core 1 | Priority 1 — dashboard, events, stats
// ─────────────────────────────────────────────
void OutputTask(void* pvParam) {
  AnomalyEvent ev;
  uint32_t last_status_print = 0;
  uint32_t last_stats_print  = 0;

  DEBUG_SERIAL.println("[OutputTask] Started on core " + String(xPortGetCoreID()));

  for (;;) {
    uint32_t now = millis();

    // ── Xử lý sự kiện bất thường ──
    while (xQueueReceive(xAnomalyQueue, &ev, 0) == pdTRUE) {
      const char* type_str = "EVENT";
      switch (ev.type) {
        case AnomalyEvent::FALL_DETECTED: type_str = "[!!!] FALL";    break;
        case AnomalyEvent::LONG_STILL:    type_str = "[ | ] STILL";   break;
        case AnomalyEvent::FAST_MOVEMENT: type_str = "[>>!] FAST";    break;
        case AnomalyEvent::ZONE_ENTER:    type_str = "[ -> ENTER";    break;
        case AnomalyEvent::ZONE_EXIT:     type_str = "[  <- EXIT";    break;
        case AnomalyEvent::NEW_TARGET:    type_str = "[ +] NEW";      break;
        case AnomalyEvent::LOST_TARGET:   type_str = "[ -] LOST";     break;
      }

      DEBUG_SERIAL.printf("[%8lu] %s | Nguoi#%d X:%5d Y:%5d | val:%.1f | %s\n",
        ev.timestamp, type_str, ev.target_id + 1,
        ev.x, ev.y, ev.value, ev.message);

      digitalWrite(LED_PIN, HIGH);
      vTaskDelay(pdMS_TO_TICKS(50));
      digitalWrite(LED_PIN, LOW);

      if (ev.type == AnomalyEvent::FALL_DETECTED) {
        for (int i = 0; i < 3; i++) {
          digitalWrite(BUZZER_PIN, HIGH); vTaskDelay(pdMS_TO_TICKS(100));
          digitalWrite(BUZZER_PIN, LOW);  vTaskDelay(pdMS_TO_TICKS(100));
        }
      }

      xSemaphoreTake(xStatsMutex, portMAX_DELAY);
      g_stats.total_events++;
      xSemaphoreGive(xStatsMutex);
    }

    // ── Dashboard mỗi 500ms ──
    if (now - last_status_print >= 500) {
      last_status_print = now;

      xSemaphoreTake(xObjectMutex, portMAX_DELAY);

      uint8_t person_count = 0;
      for (uint8_t t = 0; t < MAX_TARGETS; t++)
        if (g_objects[t].active) person_count++;

      DEBUG_SERIAL.println("+-----------------------------------------+");
      if (person_count == 0) {
        DEBUG_SERIAL.println("|  RADAR  --  Khong co nguoi             |");
        digitalWrite(LED_PIN, LOW);
      } else {
        DEBUG_SERIAL.printf( "|  RADAR  **  Phat hien %d nguoi          |\n", person_count);
      }
      DEBUG_SERIAL.printf(   "|  Uptime: %6lus                         |\n", now / 1000UL);
      DEBUG_SERIAL.println("+--+--------------------------------------+");

      if (person_count == 0) {
        DEBUG_SERIAL.println("|  (trong)                                |");
      }

      for (uint8_t t = 0; t < MAX_TARGETS; t++) {
        if (!g_objects[t].active) continue;
        TrackedObject& o = g_objects[t];

        uint8_t     zone      = getZone(o.x, o.y);
        const char* zone_name = (zone != 0xFF) ? ZONES[zone].name : "Ngoai vung";
        float       dist_r    = sqrtf((float)o.x*o.x + (float)o.y*o.y) / 1000.0f;
        const char* side      = (o.x < -100) ? "Trai" : (o.x > 100) ? "Phai" : "Giua";
        float       spd_abs   = fabsf(o.speed);
        const char* approach  = (o.speed > 5) ? "lai gan" : (o.speed < -5) ? "ra xa" : "dung yen";
        uint32_t    still_sec = (o.action == TrackedObject::STANDING)
                                ? (now - o.still_since) / 1000UL : 0;

        const char* icon = "~~";
        switch (o.action) {
          case TrackedObject::WALKING:  icon = ">>";  break;
          case TrackedObject::RUNNING:  icon = ">>!"; break;
          case TrackedObject::STANDING: icon = "|| "; break;
          case TrackedObject::SITTING:  icon = "_| "; break;
          case TrackedObject::FALLEN:   icon = "!!!"; break;
          default: break;
        }

        DEBUG_SERIAL.printf("|[%s] Nguoi #%d                          |\n", icon, t + 1);
        DEBUG_SERIAL.printf("|    Khoang cach : %.2fm phia truoc (%s) |\n", dist_r, side);
        DEBUG_SERIAL.printf("|    Toa do      : X=%5dmm  Y=%5dmm     |\n", o.x, o.y);
        DEBUG_SERIAL.printf("|    Trang thai  : %-8s | %-7s %.0fcm/s|\n",
                            actionName(o.action), approach, spd_abs);
        DEBUG_SERIAL.printf("|    Khu vuc     : %-10s               |\n", zone_name);
        DEBUG_SERIAL.printf("|    Co mat      : %lus", (now - o.presence_start) / 1000UL);

        if (o.action == TrackedObject::STANDING && still_sec > 5)
          DEBUG_SERIAL.printf(" | Dung yen: %lus", still_sec);
        if (o.action == TrackedObject::FALLEN)
          DEBUG_SERIAL.print(" | *** NGA ***");
        DEBUG_SERIAL.println();

        // Ngăn cách nếu còn người tiếp theo
        bool has_next = false;
        for (uint8_t k = t+1; k < MAX_TARGETS; k++)
          if (g_objects[k].active) { has_next = true; break; }
        if (has_next)
          DEBUG_SERIAL.println("|  - - - - - - - - - - - - - - - - - - -|");
      }

      DEBUG_SERIAL.println("+-----------------------------------------+");

      xSemaphoreGive(xObjectMutex);
    }

    // ── Thống kê mỗi 30 giây ──
    if (now - last_stats_print >= 30000) {
      last_stats_print = now;

      xSemaphoreTake(xStatsMutex, portMAX_DELAY);
      Statistics s = g_stats;
      xSemaphoreGive(xStatsMutex);

      DEBUG_SERIAL.println("\n======== THONG KE ========");
      DEBUG_SERIAL.printf("  Uptime           : %lu s\n",  (now - s.uptime_start) / 1000UL);
      DEBUG_SERIAL.printf("  Tong luot phat hien: %lu\n",  s.total_detections);
      DEBUG_SERIAL.printf("  Tong su kien     : %lu\n",    s.total_events);
      DEBUG_SERIAL.printf("  Nhieu nguoi nhat : %d\n",     s.max_simultaneous);
      for (uint8_t z = 0; z < NUM_ZONES; z++)
        DEBUG_SERIAL.printf("  %-10s : %lu s\n", ZONES[z].name, s.zone_time[z] / 1000UL);

      // Quỹ đạo 5 điểm gần nhất
      xSemaphoreTake(xObjectMutex, portMAX_DELAY);
      for (uint8_t t = 0; t < MAX_TARGETS; t++) {
        if (!g_objects[t].active || g_objects[t].traj_len < 2) continue;
        TrackedObject& o = g_objects[t];
        DEBUG_SERIAL.printf("  Quy dao Nguoi#%d (%d pts): ", t+1, o.traj_len);
        uint8_t pn = min((uint8_t)5, o.traj_len);
        for (uint8_t p = 0; p < pn; p++) {
          uint8_t idx = (o.traj_head + TRAJ_MAX_POINTS - pn + 1 + p) % TRAJ_MAX_POINTS;
          DEBUG_SERIAL.printf("(%d,%d)", o.traj_x[idx], o.traj_y[idx]);
          if (p < pn-1) DEBUG_SERIAL.print("->");
        }
        DEBUG_SERIAL.println();
      }
      xSemaphoreGive(xObjectMutex);
      DEBUG_SERIAL.println("==========================\n");
    }

    // LED heartbeat
    if (g_presence && (now % 1000 < 50))
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ─────────────────────────────────────────────
//  SETUP & LOOP
// ─────────────────────────────────────────────
void setup() {
  DEBUG_SERIAL.begin(115200);
  delay(500);
  DEBUG_SERIAL.println("\n\n====== HLK-LD2450 RTOS System ======");
  DEBUG_SERIAL.println("ESP32 + FreeRTOS | 3-Task Architecture");
  DEBUG_SERIAL.println("=====================================\n");

  pinMode(LED_PIN,    OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(LED_PIN,    LOW);
  digitalWrite(BUZZER_PIN, LOW);

  RADAR_SERIAL.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
  DEBUG_SERIAL.printf("Radar UART2: RX=GPIO%d TX=GPIO%d BAUD=%d\n",
                      RADAR_RX_PIN, RADAR_TX_PIN, RADAR_BAUD);

  memset(g_objects, 0, sizeof(g_objects));
  memset(&g_stats,  0, sizeof(g_stats));
  g_stats.uptime_start = millis();
  g_presence  = false;
  g_zone_mask = 0;

  xRadarQueue   = xQueueCreate(20, sizeof(RadarFrame));
  xAnomalyQueue = xQueueCreate(50, sizeof(AnomalyEvent));
  xObjectMutex  = xSemaphoreCreateMutex();
  xStatsMutex   = xSemaphoreCreateMutex();

  if (!xRadarQueue || !xAnomalyQueue || !xObjectMutex || !xStatsMutex) {
    DEBUG_SERIAL.println("FATAL: Khong tao duoc RTOS primitives!");
    while(1);
  }

  // Task 1: RadarReadTask — Core 0, Priority 3
  xTaskCreatePinnedToCore(RadarReadTask,  "RadarRead",  4096, NULL, 3, &hRadarTask,   0);
  // Task 2: ProcessingTask — Core 1, Priority 2
  xTaskCreatePinnedToCore(ProcessingTask, "Processing", 8192, NULL, 2, &hProcessTask, 1);
  // Task 3: OutputTask — Core 1, Priority 1
  xTaskCreatePinnedToCore(OutputTask,     "Output",     4096, NULL, 1, &hOutputTask,  1);

  DEBUG_SERIAL.println("3 RTOS tasks da khoi dong:");
  DEBUG_SERIAL.println("  Core0 | Pri3 | RadarReadTask   — UART parse");
  DEBUG_SERIAL.println("  Core1 | Pri2 | ProcessingTask  — Tracking/Zone/Event");
  DEBUG_SERIAL.println("  Core1 | Pri1 | OutputTask      — Dashboard/LED/Alert");
  DEBUG_SERIAL.println("\nDang cho du lieu radar...\n");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
