/*
 * ============================================================
 * HỆ THỐNG GIÁM SÁT SỨC KHỎE — ESP32 + HLK-LD2450
 * Chế độ: Serial Monitor only (không WiFi, không Web)
 * ============================================================
 * Đấu dây:
 *   LD2450 TX → ESP32 GPIO 16 (UART2 RX)
 *   LD2450 RX → ESP32 GPIO 17 (UART2 TX)
 *   Buzzer    → GPIO 25 + GND
 * ============================================================
 */

#define UART_RX_PIN      16
#define UART_TX_PIN      17
#define LD2450_BAUD      115200
#define BUZZER_PIN       25
#define LED_PIN          2

#define MAX_TARGETS      3

// Ngưỡng tốc độ (mm/s)
#define SPEED_STILL_MAX   200
#define SPEED_FALL_MIN    600
#define SPEED_FALL_MAX   1200

// Ngưỡng thời gian (ms)
#define FALL_STILL_MS    20000UL
#define IMMOB_MS         90000UL
#define TARGET_TIMEOUT_MS 3000UL

// Nearest-neighbor
#define NN_MAX_DIST_MM   1000

// ============================================================
//  ENUM & STRUCT
// ============================================================
typedef enum { ST_INACTIVE, ST_NORMAL, ST_RESTING,
               ST_STILL, ST_FALLEN, ST_IMMOBILE } MovState;
typedef enum { AL_NONE, AL_FALL, AL_IMMOBILE } AlertType;

struct TargetRaw {
  int16_t x, y, speed;
  bool    valid;
};

struct RadarFrame {
  TargetRaw targets[MAX_TARGETS];
  uint32_t  ts;
};

struct PersonState {
  int       id;
  float     x, y, speed;
  MovState  state;
  AlertType alert;
  bool      active;
  uint32_t  lastActiveMs;
  uint32_t  stillStartMs;
  uint32_t  spikeMs;
  bool      hadSpike;
};

// ============================================================
//  GLOBAL
// ============================================================
PersonState g_persons[MAX_TARGETS];

// ============================================================
//  HELPERS
// ============================================================
const char* stateStr(MovState s) {
  switch(s) {
    case ST_NORMAL:   return "NORMAL";
    case ST_RESTING:  return "RESTING";
    case ST_STILL:    return "STILL";
    case ST_FALLEN:   return "FALLEN";
    case ST_IMMOBILE: return "IMMOBILE";
    default:          return "INACTIVE";
  }
}
const char* alertStr(AlertType a) {
  if (a == AL_FALL)     return "*** NGÃ ***";
  if (a == AL_IMMOBILE) return "*** NGẤT ***";
  return "none";
}

void initPerson(PersonState* p, int id) {
  memset(p, 0, sizeof(PersonState));
  p->id    = id;
  p->state = ST_INACTIVE;
}

// ============================================================
//  LD2450 PARSER
// ============================================================
class LD2450Parser {
  uint8_t buf[30];
  uint8_t pos = 0;
public:
  bool feed(uint8_t byte, RadarFrame* out) {
    buf[pos++] = byte;
    if (pos <= 4) {
      const uint8_t HDR[4] = {0xAA, 0xFF, 0x03, 0x00};
      if (buf[pos-1] != HDR[pos-1]) {
        memmove(buf, buf+1, pos-1); pos--;
      }
      return false;
    }
    if (pos < 30) return false;
    if (buf[28] != 0x55 || buf[29] != 0xCC) {
      for (int i=1; i<(int)pos; i++) {
        if (buf[i] == 0xAA) { memmove(buf, buf+i, pos-i); pos -= i; return false; }
      }
      pos = 0; return false;
    }
    out->ts = millis();
    for (int i=0; i<MAX_TARGETS; i++) {
      uint8_t* d = buf + 4 + i*8;
      uint16_t rx = d[0] | ((uint16_t)d[1] << 8);
      uint16_t ry = d[2] | ((uint16_t)d[3] << 8);
      uint16_t rs = d[4] | ((uint16_t)d[5] << 8);
      out->targets[i].x     = (rx & 0x8000) ? -(int16_t)(rx & 0x7FFF) : (int16_t)(rx & 0x7FFF);
      out->targets[i].y     = (ry & 0x8000) ? -(int16_t)(ry & 0x7FFF) : (int16_t)(ry & 0x7FFF);
      out->targets[i].speed = (rs & 0x8000) ? -(int16_t)((rs & 0x7FFF)*10) : (int16_t)((rs & 0x7FFF)*10);
      out->targets[i].valid = (out->targets[i].x != 0 || out->targets[i].y != 0);
    }
    pos = 0; return true;
  }
};

// ============================================================
//  STATE MACHINE
// ============================================================
void runStateMachine(PersonState& p, uint32_t now) {
  if (!p.active) { p.state = ST_INACTIVE; return; }
  if (p.alert != AL_NONE) return;

  bool isStill = (p.speed <= SPEED_STILL_MAX);
  uint32_t stillDur = (p.stillStartMs > 0) ? (now - p.stillStartMs) : 0;

  if (p.hadSpike && isStill && stillDur >= FALL_STILL_MS) {
    p.alert = AL_FALL;
    p.state = ST_FALLEN;
    Serial.println("=====================================");
    Serial.printf("!!! CẢNH BÁO: Person%d NGÃ !!!\n", p.id);
    Serial.printf("    Vị trí: X=%.0f mm, Y=%.0f mm\n", p.x, p.y);
    Serial.println("=====================================");
    return;
  }
  if (!p.hadSpike && isStill && stillDur >= IMMOB_MS) {
    p.alert = AL_IMMOBILE;
    p.state = ST_IMMOBILE;
    Serial.println("=====================================");
    Serial.printf("!!! CẢNH BÁO: Person%d NGẤT !!!\n", p.id);
    Serial.printf("    Vị trí: X=%.0f mm, Y=%.0f mm\n", p.x, p.y);
    Serial.println("=====================================");
    return;
  }
  if (p.hadSpike && (now - p.spikeMs) > (FALL_STILL_MS + 5000UL))
    p.hadSpike = false;

  if (isStill && stillDur > 2000) { p.state = ST_STILL; return; }
  p.state = ST_NORMAL;
}

// ============================================================
//  PROCESS FRAME
// ============================================================
void processFrame(const RadarFrame& frame) {
  uint32_t now = frame.ts;
  bool usedSlot[MAX_TARGETS] = {};

  for (int t=0; t<MAX_TARGETS; t++) {
    if (!frame.targets[t].valid) continue;
    float tx = frame.targets[t].x;
    float ty = frame.targets[t].y;
    float spd = abs(frame.targets[t].speed);

    int   best = -1;
    float bestD = NN_MAX_DIST_MM;
    for (int s=0; s<MAX_TARGETS; s++) {
      if (usedSlot[s] || !g_persons[s].active) continue;
      float dx = tx - g_persons[s].x, dy = ty - g_persons[s].y;
      float d = sqrt(dx*dx + dy*dy);
      if (d < bestD) { bestD = d; best = s; }
    }
    if (best == -1) {
      for (int s=0; s<MAX_TARGETS; s++) {
        if (!usedSlot[s] && !g_persons[s].active) { best = s; break; }
      }
    }
    if (best == -1) continue;

    usedSlot[best] = true;
    PersonState& p = g_persons[best];
    p.x = tx; p.y = ty; p.speed = spd;
    p.active = true;
    p.lastActiveMs = now;

    if (spd >= SPEED_FALL_MIN && spd <= SPEED_FALL_MAX && !p.hadSpike) {
      p.hadSpike = true;
      p.spikeMs  = now;
      Serial.printf("[Spike] Person%d speed=%.0f mm/s\n", best, spd);
    }
    if (spd > SPEED_STILL_MAX) p.stillStartMs = 0;
    else if (p.stillStartMs == 0) p.stillStartMs = now;
  }

  for (int s=0; s<MAX_TARGETS; s++) {
    if (!usedSlot[s] && g_persons[s].active) {
      if ((now - g_persons[s].lastActiveMs) > TARGET_TIMEOUT_MS) {
        g_persons[s].active = false;
        g_persons[s].state  = ST_INACTIVE;
      }
    }
    runStateMachine(g_persons[s], now);
  }
}

// ============================================================
//  PRINT STATUS (mỗi 500ms)
// ============================================================
uint32_t lastPrint = 0;

void printStatus() {
  uint32_t now = millis();
  if (now - lastPrint < 500) return;
  lastPrint = now;

  bool anyActive = false;
  for (int i=0; i<MAX_TARGETS; i++)
    if (g_persons[i].active) { anyActive = true; break; }

  if (!anyActive) {
    Serial.println("[--] Không phát hiện đối tượng");
    return;
  }

  Serial.println("------------------------------------");
  for (int i=0; i<MAX_TARGETS; i++) {
    PersonState& p = g_persons[i];
    if (!p.active) continue;
    Serial.printf("Person%d | X:%6.0f Y:%6.0f | Speed:%5.0f mm/s | %s",
      i, p.x, p.y, p.speed, stateStr(p.state));
    if (p.alert != AL_NONE)
      Serial.printf(" | %s", alertStr(p.alert));
    Serial.println();
  }
}

// ============================================================
//  SETUP & LOOP
// ============================================================
LD2450Parser parser;
RadarFrame   frame;

void setup() {
  Serial.begin(115200);
  Serial2.begin(LD2450_BAUD, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  pinMode(BUZZER_PIN, OUTPUT); digitalWrite(BUZZER_PIN, LOW);
  pinMode(LED_PIN, OUTPUT);    digitalWrite(LED_PIN, HIGH);

  for (int i=0; i<MAX_TARGETS; i++) initPerson(&g_persons[i], i);

  Serial.println("====================================");
  Serial.println("  LD2450 Serial Monitor");
  Serial.println("  Baud: 115200");
  Serial.println("====================================");
}

void loop() {
  // Đọc từ LD2450
  while (Serial2.available()) {
    if (parser.feed(Serial2.read(), &frame)) {
      processFrame(frame);
    }
  }

  // Timeout
  uint32_t now = millis();
  for (int s=0; s<MAX_TARGETS; s++) {
    if (g_persons[s].active && (now - g_persons[s].lastActiveMs) > TARGET_TIMEOUT_MS) {
      g_persons[s].active = false;
      g_persons[s].state  = ST_INACTIVE;
    }
  }

  // Buzzer
  bool anyAlert = false;
  for (int i=0; i<MAX_TARGETS; i++)
    if (g_persons[i].alert != AL_NONE) { anyAlert = true; break; }
  digitalWrite(BUZZER_PIN, anyAlert ? HIGH : LOW);

  // In ra Serial
  printStatus();
}
