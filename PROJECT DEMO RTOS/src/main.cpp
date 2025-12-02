#include <Arduino.h>
#include <math.h>
#include <ESP32Servo.h>    // Library servo khusus ESP32

// ================== PIN SETUP (SESUIKAN DENGAN WOKWI-MU) ==================
// Stepper kiri (motor 1)
#define STEP1_PIN   14
#define DIR1_PIN    15

// Stepper kanan (motor 2)
#define STEP2_PIN   7
#define DIR2_PIN    9

// Stepper ke-3 (kipas pemadam api)
#define STEP3_PIN   42
#define DIR3_PIN    41

// Ultrasonic HC-SR04
#define TRIG_PIN    17
#define ECHO_PIN    18

// Servo pemutar sensor (satu mekanik dengan kipas)
#define SERVO_PIN   8

// LED
#define LED_STATUS  19    // status sistem
#define LED_FAN     20    // indikator kipas ON

// ================== PARAMETER LOGIKA ROBOT ==================
// LOGIKA BARU:
// > 300 cm  -> tidak ada api
// <= 300 cm -> deteksi api (jauh)
// <= 50 cm  -> sangat dekat, berhenti & kipas ON

const float FIRE_DETECT_CM   = 300.0f;  // batas deteksi api
const float STOP_DISTANCE_CM = 50.0f;   // jarak berhenti & nyalakan kipas

// Parameter differential drive
const float WHEEL_RADIUS   = 0.03f;   // jari-jari roda (m)
const float WHEEL_BASE     = 0.15f;   // jarak antar roda (m)
const int   STEPS_PER_REV  = 200;     // step per satu putaran stepper
const float PI_F           = 3.1415926f;

// Kecepatan robot (linear & angular)
const float V_PATROL    = 0.08f;   // m/s saat patroli/random
const float V_APPROACH  = 0.10f;   // m/s saat mendekati api
const float OMEGA_TURN  = 1.2f;    // rad/s untuk belok

// Durasi gerakan random
const uint32_t RANDOM_MOVE_MS      = 3000; // 3 detik tiap pola random
const uint32_t EXTINGUISH_MAX_MS   = 5000; // max kipas 5 detik

// Kecepatan kipas (stepper 3) – makin kecil makin kencang
const uint32_t FAN_STEP_INTERVAL_US  = 10;  // super cepat

// Perkiraan waktu belok 90° (pakai OMEGA_TURN)
const uint32_t TURN_TIME_90_MS = (uint32_t)((PI_F / 2.0f) / OMEGA_TURN * 1000.0f);

// ================== TIPE DATA & RTOS ==================

enum RobotMode {
  MODE_SCAN_RANDOM,      // >300 cm, robot gerak random + servo scanning
  MODE_DETECT_PAUSE,     // baru deteksi <=300, berhenti sebentar
  MODE_TURN_TO_FIRE,     // belok ke arah api
  MODE_APPROACH_FIRE,    // maju lurus menuju api sampai <=50 cm
  MODE_EXTINGUISH        // berhenti & kipas ON, tunggu api padam / 5 detik
};

struct MoveCommand {
  float v;      // kecepatan linear (m/s)
  float omega;  // kecepatan sudut (rad/s)
};

// State bersama (pakai mutex)
struct SharedState {
  float distanceCm;        // jarak terbaru
  int   servoAngle;        // sudut servo aktual
  int   desiredServoAngle; // target sudut servo (diatur TaskControl)
  bool  scanMode;          // true = servo sweeping, false = servo diam
  bool  fireDetected;      // dist <= 300
  bool  nearFire;          // dist <= 50
  bool  extinguish;        // kipas ON?
};

SharedState sharedState = { 999.0f, 90, 90, true, false, false, false };

QueueHandle_t moveQueue;
SemaphoreHandle_t stateMutex;

Servo scanServo;

// ================== FUNGSI BANTU ==================

// Baca jarak ultrasonic
float readUltrasonicCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(3);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // timeout 30 ms
  if (duration == 0) {
    return 400.0f; // tidak ada pantulan
  }
  float distance = duration * 0.0343f / 2.0f;
  return distance;
}

// Inverse kinematics differential drive
void inverseKinematics(float v, float omega, float &wl, float &wr) {
  wl = (2.0f * v - omega * WHEEL_BASE) / (2.0f * WHEEL_RADIUS);
  wr = (2.0f * v + omega * WHEEL_BASE) / (2.0f * WHEEL_RADIUS);
}

// ================== TASK: SERVO + SENSOR ==================
// Servo mengikuti mode:
//   - scanMode = true  -> sweep 0..180 derajat
//   - scanMode = false -> stay di desiredServoAngle
// Setiap loop baca ultrasonic & simpan ke sharedState.
void TaskServoSensor(void *pvParameters) {
  (void) pvParameters;

  int  servoAngle = 0;        // mulai dari 0°
  bool increasing = true;     // sweep 0 → 180 → 0 ...

  for (;;) {
    bool scan = true;
    int  desired = 90;

    // Baca perintah dari TaskControl
    if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
      scan    = sharedState.scanMode;
      desired = sharedState.desiredServoAngle;
      xSemaphoreGive(stateMutex);
    }

    if (scan) {
      // MODE SCAN: servo sweeping 0..180
      if (increasing) {
        servoAngle += 10;
        if (servoAngle >= 180) {
          servoAngle = 180;
          increasing = false;
        }
      } else {
        servoAngle -= 10;
        if (servoAngle <= 0) {
          servoAngle = 0;
          increasing = true;
        }
      }
    } else {
      // MODE HOLD: servo diam di desired angle
      servoAngle = desired;
    }

    scanServo.write(servoAngle);

    // Baca jarak
    float dist = readUltrasonicCm();

    // Simpan ke sharedState
    if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
      sharedState.distanceCm = dist;
      sharedState.servoAngle = servoAngle;
      // flag bantu
      sharedState.fireDetected = (dist <= FIRE_DETECT_CM);
      sharedState.nearFire     = (dist <= STOP_DISTANCE_CM);
      xSemaphoreGive(stateMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(80));  // kecepatan sweep / update sensor
  }
}

// ================== TASK: CONTROL / STATE MACHINE ==================
void TaskControl(void *pvParameters) {
  (void) pvParameters;

  RobotMode mode = MODE_SCAN_RANDOM;
  TickType_t randomEndTick       = 0;
  MoveCommand randomCmd          = {0.0f, 0.0f};

  TickType_t detectStartTick     = 0;
  TickType_t turnEndTick         = 0;
  TickType_t extinguishStartTick = 0;

  // Seed random
  randomSeed(analogRead(0));

  for (;;) {
    float dist = 999.0f;
    int   servoAngle = 90;

    // Ambil snapshot dari sharedState
    if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
      dist       = sharedState.distanceCm;
      servoAngle = sharedState.servoAngle;
      xSemaphoreGive(stateMutex);
    }

    TickType_t nowTick = xTaskGetTickCount();

    switch (mode) {

      // ================== MODE 0: SCAN + GERAK RANDOM ==================
      case MODE_SCAN_RANDOM: {
        // Servo mode scanning
        if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
          sharedState.scanMode          = true;
          sharedState.desiredServoAngle = sharedState.servoAngle;
          sharedState.extinguish        = false;
          xSemaphoreGive(stateMutex);
        }

        if (dist > FIRE_DETECT_CM) {
          // Tidak ada api → gerak random
          if (nowTick >= randomEndTick) {
            int r = random(0, 4); // 0..3
            switch (r) {
              case 0: // maju
                randomCmd.v     = V_PATROL;
                randomCmd.omega = 0.0f;
                Serial.println("[RANDOM] Gerak MAJU");
                break;
              case 1: // mundur
                randomCmd.v     = -V_PATROL;
                randomCmd.omega = 0.0f;
                Serial.println("[RANDOM] Gerak MUNDUR");
                break;
              case 2: // putar kiri
                randomCmd.v     = 0.0f;
                randomCmd.omega = OMEGA_TURN;
                Serial.println("[RANDOM] BELOK KIRI");
                break;
              case 3: // putar kanan
              default:
                randomCmd.v     = 0.0f;
                randomCmd.omega = -OMEGA_TURN;
                Serial.println("[RANDOM] BELOK KANAN");
                break;
            }
            randomEndTick = nowTick + pdMS_TO_TICKS(RANDOM_MOVE_MS);
          }
          xQueueOverwrite(moveQueue, &randomCmd);
        } else {
          // DETEKSI API (<=300) → berhenti & servo stop sementara
          MoveCommand stopCmd = {0.0f, 0.0f};
          xQueueOverwrite(moveQueue, &stopCmd);
          Serial.println("[DETECT] Stop, ada objek/api di depan (<=300 cm)");

          if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
            sharedState.scanMode          = false;      // servo berhenti
            sharedState.desiredServoAngle = servoAngle; // tahan di sudut saat deteksi
            xSemaphoreGive(stateMutex);
          }

          detectStartTick = nowTick;
          mode = MODE_DETECT_PAUSE; // 1
        }
        break;
      }

      // ================== MODE 1: PAUSE SETELAH DETEKSI ==================
      case MODE_DETECT_PAUSE: {
        // Kalau tiba-tiba target hilang (dist > 300) → balik ke scan
        if (dist > FIRE_DETECT_CM) {
          Serial.println("[DETECT] Target hilang, kembali ke SCAN_RANDOM");
          mode = MODE_SCAN_RANDOM;
          break;
        }

        if ((nowTick - detectStartTick) >= pdMS_TO_TICKS(300)) {
          // Setelah jeda sebentar, tentukan arah dari sudut servo
          if (servoAngle < 60) {
            // Api di kanan → belok kanan
            MoveCommand turnCmd = {0.0f, -OMEGA_TURN};
            xQueueOverwrite(moveQueue, &turnCmd);
            Serial.println("[DETECT] Api di KANAN -> BELOK KANAN");

            turnEndTick = nowTick + pdMS_TO_TICKS(TURN_TIME_90_MS);
            mode = MODE_TURN_TO_FIRE; // 2

          } else if (servoAngle > 120) {
            // Api di kiri → belok kiri
            MoveCommand turnCmd = {0.0f, OMEGA_TURN};
            xQueueOverwrite(moveQueue, &turnCmd);
            Serial.println("[DETECT] Api di KIRI -> BELOK KIRI");

            turnEndTick = nowTick + pdMS_TO_TICKS(TURN_TIME_90_MS);
            mode = MODE_TURN_TO_FIRE; // 2

          } else {
            // Api di depan → langsung maju
            Serial.println("[DETECT] Api di DEPAN -> lanjut APPROACH_FIRE (MAJU)");
            mode = MODE_APPROACH_FIRE; // 3
          }
        } else {
          // Tetap berhenti
          MoveCommand stopCmd = {0.0f, 0.0f};
          xQueueOverwrite(moveQueue, &stopCmd);
        }
        break;
      }

      // ================== MODE 2: BELOK KE ARAH API ==================
      case MODE_TURN_TO_FIRE: {
        // Kalau target hilang saat belok → balik scan
        if (dist > FIRE_DETECT_CM) {
          MoveCommand stopCmd = {0.0f, 0.0f};
          xQueueOverwrite(moveQueue, &stopCmd);
          Serial.println("[TURN] Target hilang saat belok, kembali ke SCAN_RANDOM");
          mode = MODE_SCAN_RANDOM;
          break;
        }

        if (nowTick >= turnEndTick) {
          // selesai belok → stop sebentar, lalu lanjut ke approach
          MoveCommand stopCmd = {0.0f, 0.0f};
          xQueueOverwrite(moveQueue, &stopCmd);
          Serial.println("[TURN] Selesai belok, lanjut APPROACH_FIRE");
          mode = MODE_APPROACH_FIRE; // 3
        }
        break;
      }

      // ================== MODE 3: MAJU KE ARAH API ==================
      case MODE_APPROACH_FIRE: {
        // Servo diarahkan ke depan (90°) dan diam
        if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
          sharedState.scanMode          = false;
          sharedState.desiredServoAngle = 90;
          xSemaphoreGive(stateMutex);
        }

        // Kalau jarak kembali >300 → target hilang → balik MODE 0
        if (dist > FIRE_DETECT_CM) {
          MoveCommand stopCmd = {0.0f, 0.0f};
          xQueueOverwrite(moveQueue, &stopCmd);

          if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
            sharedState.extinguish = false;
            sharedState.scanMode   = true;
            xSemaphoreGive(stateMutex);
          }

          Serial.println("[APPROACH] Target hilang, kembali ke SCAN_RANDOM");
          mode = MODE_SCAN_RANDOM; // 0
          break;
        }

        if (dist > STOP_DISTANCE_CM) {
          // Masih >50 cm → maju lurus
          MoveCommand cmd = {V_APPROACH, 0.0f};
          xQueueOverwrite(moveQueue, &cmd);
          Serial.println("[APPROACH] MAJU mendekati api");
        } else {
          // Sudah <= 50 → berhenti & mulai padamkan api
          MoveCommand stopCmd = {0.0f, 0.0f};
          xQueueOverwrite(moveQueue, &stopCmd);
          Serial.println("[APPROACH] Jarak <= 50 cm, STOP dan nyalakan kipas");

          if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
            sharedState.extinguish = true;  // nyalakan kipas
            xSemaphoreGive(stateMutex);
          }

          extinguishStartTick = nowTick;
          mode = MODE_EXTINGUISH; // 4
        }
        break;
      }

      // ================== MODE 4: PADAMKAN API (KIPAS ON) ==================
      case MODE_EXTINGUISH: {
        // Motor tetap diam
        MoveCommand stopCmd = {0.0f, 0.0f};
        xQueueOverwrite(moveQueue, &stopCmd);

        bool timeUp  = (nowTick - extinguishStartTick) >= pdMS_TO_TICKS(EXTINGUISH_MAX_MS);
        bool farNow  = (dist > FIRE_DETECT_CM);   // jarak > 300

        if (timeUp || farNow) {
          if (timeUp)  Serial.println("[EXTINGUISH] Waktu 5 detik habis, kembali SCAN_RANDOM");
          if (farNow)  Serial.println("[EXTINGUISH] Jarak >300 (api padam), kembali SCAN_RANDOM");

          // Matikan kipas, balik mode scan + random
          if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
            sharedState.extinguish        = false;
            sharedState.scanMode          = true;   // servo kembali scanning
            sharedState.desiredServoAngle = 0;      // mulai dari 0°
            xSemaphoreGive(stateMutex);
          }

          mode = MODE_SCAN_RANDOM;  // 0
        }
        break;
      }
    }

    // Flag bantu (optional)
    if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
      sharedState.fireDetected = (dist <= FIRE_DETECT_CM);
      sharedState.nearFire     = (dist <= STOP_DISTANCE_CM);
      xSemaphoreGive(stateMutex);
    }

    // LOG MODE & JARAK (biar kelihatan state-nya juga)
    Serial.print("Mode: ");
    Serial.print((int)mode);
    Serial.print("  Dist: ");
    Serial.println(dist);

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ================== TASK: MOTOR (CORE 1) ==================
void TaskMotor(void *pvParameters) {
  (void) pvParameters;

  MoveCommand currentCmd = {0.0f, 0.0f};

  float wl = 0.0f;
  float wr = 0.0f;

  uint32_t stepIntervalL = 0;
  uint32_t stepIntervalR = 0;

  uint32_t lastStepMicrosL = micros();
  uint32_t lastStepMicrosR = micros();

  for (;;) {
    MoveCommand newCmd;
    if (xQueueReceive(moveQueue, &newCmd, 0) == pdPASS) {
      currentCmd = newCmd;

      inverseKinematics(currentCmd.v, currentCmd.omega, wl, wr);

      digitalWrite(DIR1_PIN, (wl >= 0.0f) ? HIGH : LOW);
      digitalWrite(DIR2_PIN, (wr >= 0.0f) ? HIGH : LOW);

      float stepsPerSecL = fabsf(wl) / (2.0f * PI_F) * STEPS_PER_REV;
      float stepsPerSecR = fabsf(wr) / (2.0f * PI_F) * STEPS_PER_REV;

      stepIntervalL = (stepsPerSecL > 1.0f) ? (uint32_t)(1000000.0f / stepsPerSecL) : 0;
      stepIntervalR = (stepsPerSecR > 1.0f) ? (uint32_t)(1000000.0f / stepsPerSecR) : 0;
    }

    uint32_t now = micros();

    if (stepIntervalL > 0 && (now - lastStepMicrosL) >= stepIntervalL) {
      lastStepMicrosL = now;
      digitalWrite(STEP1_PIN, HIGH);
      delayMicroseconds(3);
      digitalWrite(STEP1_PIN, LOW);
    }

    if (stepIntervalR > 0 && (now - lastStepMicrosR) >= stepIntervalR) {
      lastStepMicrosR = now;
      digitalWrite(STEP2_PIN, HIGH);
      delayMicroseconds(3);
      digitalWrite(STEP2_PIN, LOW);
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ================== TASK: STATUS + KIPAS (CORE 0) ==================
void TaskStatusFan(void *pvParameters) {
  (void) pvParameters;

  pinMode(LED_STATUS, OUTPUT);
  pinMode(LED_FAN, OUTPUT);

  pinMode(STEP3_PIN, OUTPUT);
  pinMode(DIR3_PIN, OUTPUT);

  digitalWrite(DIR3_PIN, HIGH);  // arah putaran kipas

  uint32_t lastFanStepMicros = micros();
  TickType_t lastBlinkTick = xTaskGetTickCount();
  bool ledState = false;
  const TickType_t BLINK_INTERVAL = pdMS_TO_TICKS(200);

  for (;;) {
    bool ext = false;

    if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
      ext = sharedState.extinguish;
      xSemaphoreGive(stateMutex);
    }

    // Blink LED_STATUS tanpa blocking
    TickType_t nowTick = xTaskGetTickCount();
    if (nowTick - lastBlinkTick >= BLINK_INTERVAL) {
      lastBlinkTick = nowTick;
      ledState = !ledState;
      digitalWrite(LED_STATUS, ledState ? HIGH : LOW);
    }

    uint32_t now = micros();

    if (ext) {
      // MODE PADAMKAN API: KIPAS FULL SPEED
      digitalWrite(LED_FAN, HIGH);

      if (now - lastFanStepMicros >= FAN_STEP_INTERVAL_US) {
        lastFanStepMicros = now;
        digitalWrite(STEP3_PIN, HIGH);
        delayMicroseconds(3);
        digitalWrite(STEP3_PIN, LOW);
      }

      // Jangan delay ms, cukup yield
      taskYIELD();

    } else {
      // MODE NORMAL: KIPAS MATI
      digitalWrite(LED_FAN, LOW);
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
}

// ================== SETUP & LOOP ==================
void setup() {
  Serial.begin(115200);
  delay(1000);

  // Pin mode stepper penggerak
  pinMode(STEP1_PIN, OUTPUT);
  pinMode(DIR1_PIN, OUTPUT);
  pinMode(STEP2_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);

  // Pin mode stepper kipas
  pinMode(STEP3_PIN, OUTPUT);
  pinMode(DIR3_PIN, OUTPUT);

  // Ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Inisialisasi timer PWM untuk ESP32Servo
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Servo
  scanServo.setPeriodHertz(50);             // servo 50 Hz
  scanServo.attach(SERVO_PIN, 500, 2400);   // min/max pulse µs
  scanServo.write(90);

  // RTOS primitive
  moveQueue  = xQueueCreate(1, sizeof(MoveCommand));  // 1 elemen, di-overwrite
  stateMutex = xSemaphoreCreateMutex();

  if (moveQueue == NULL || stateMutex == NULL) {
    Serial.println("Gagal membuat queue atau mutex!");
    while (true) {
      delay(1000);
    }
  }

  // Task & core
  xTaskCreatePinnedToCore(
    TaskServoSensor,
    "ServoSensor",
    4096,
    NULL,
    2,
    NULL,
    0       // Core 0
  );

  xTaskCreatePinnedToCore(
    TaskControl,
    "Control",
    4096,
    NULL,
    2,
    NULL,
    0       // Core 0
  );

  xTaskCreatePinnedToCore(
    TaskMotor,
    "Motor",
    4096,
    NULL,
    2,
    NULL,
    1       // Core 1
  );

  xTaskCreatePinnedToCore(
    TaskStatusFan,
    "StatusFan",
    4096,
    NULL,
    1,
    NULL,
    0       // Core 0
  );
}

void loop() {
  // semua kerja di task RTOS
}
