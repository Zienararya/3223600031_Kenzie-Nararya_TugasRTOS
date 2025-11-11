#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>

// --- OLED setup ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// --- Pin setup ---
#define LED1 4
#define LED2 5
#define LED3 6
#define BTN1 12
#define BTN2 13
#define BUZZER 2
#define POT 1
#define ENCA 19
#define ENCB 20
#define SW 21
#define SERVO_PIN 18

// Stepper pins
#define STEP_PIN 16
#define DIR_PIN 15
#define EN_PIN 17

Servo myServo;

// --- Task handles ---
TaskHandle_t Task_LED;
TaskHandle_t Task_ButtonOLED;
TaskHandle_t Task_PotServo;
TaskHandle_t Task_Stepper;
TaskHandle_t Task_Buzzer;

// --- Function declarations ---
void taskLED(void *pvParameters);
void taskButtonOLED(void *pvParameters);
void taskPotServo(void *pvParameters);
void taskStepper(void *pvParameters);
void taskBuzzer(void *pvParameters);

void setup() {
  Serial.begin(115200);

  // Pin mode
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(BTN1, INPUT_PULLUP);
  pinMode(BTN2, INPUT_PULLUP);
  pinMode(BUZZER, OUTPUT);
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW);

  myServo.attach(SERVO_PIN);

  // OLED init
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)){
    Serial.println("OLED gagal!");
    for(;;);
  }
  display.clearDisplay();
  display.display();

  // --- Create tasks ---
  xTaskCreatePinnedToCore(taskLED, "TaskLED", 2048, NULL, 1, &Task_LED, 0);
  xTaskCreatePinnedToCore(taskButtonOLED, "TaskButtonOLED", 4096, NULL, 1, &Task_ButtonOLED, 1);
  xTaskCreatePinnedToCore(taskPotServo, "TaskPotServo", 2048, NULL, 1, &Task_PotServo, 1);
  xTaskCreatePinnedToCore(taskStepper, "TaskStepper", 2048, NULL, 1, &Task_Stepper, 1);
  xTaskCreatePinnedToCore(taskBuzzer, "TaskBuzzer", 2048, NULL, 1, &Task_Buzzer, 0);
}

void loop() {
  // Kosong, semua dijalankan oleh RTOS
}

// --- TASK DEFINITIONS ---

void taskLED(void *pvParameters) {
  while(1) {
    digitalWrite(LED1, HIGH); 
    vTaskDelay(200 / portTICK_PERIOD_MS);
    digitalWrite(LED1, LOW); 
    digitalWrite(LED2, HIGH); 
    vTaskDelay(200 / portTICK_PERIOD_MS);
    digitalWrite(LED2, LOW); 
    digitalWrite(LED3, HIGH); 
    vTaskDelay(200 / portTICK_PERIOD_MS);
    digitalWrite(LED3, LOW);
  }
}

void taskButtonOLED(void *pvParameters) {
  while(1) {
    int b1 = digitalRead(BTN1);
    int b2 = digitalRead(BTN2);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.print("Button1: "); display.println(b1 == LOW ? "Pressed" : "Released");
    display.print("Button2: "); display.println(b2 == LOW ? "Pressed" : "Released");
    display.display();
    vTaskDelay(300 / portTICK_PERIOD_MS);
  }
}

void taskPotServo(void *pvParameters) {
  while(1) {
    int potVal = analogRead(POT);
    int angle = map(potVal, 0, 4095, 0, 180);
    myServo.write(angle);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void taskStepper(void *pvParameters) {
  bool dir = true;
  while(1) {
    digitalWrite(DIR_PIN, dir); 
    for (int i = 0; i < 200; i++) { 
      digitalWrite(STEP_PIN, HIGH);
      vTaskDelay(2 / portTICK_PERIOD_MS);
      digitalWrite(STEP_PIN, LOW);
      vTaskDelay(2 / portTICK_PERIOD_MS);
    }
    dir = !dir; // ubah arah tiap putaran
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void taskBuzzer(void *pvParameters) {
  int lastA = LOW;
  int lastB = LOW;
  int freq = 1000;
  bool buzzerOn = false;
  while(1) {
  // ==== BACA ROTARY ENCODER ====
  int aState = digitalRead(ENCA);
  int bState = digitalRead(ENCB);

  if (aState != lastA) {
    if (bState != aState) {
      freq += 100;
    } else {
      freq -= 100;
    }

    // Batasi rentang frekuensi agar tidak terlalu rendah atau tinggi
    if (freq < 100) freq = 100;
    if (freq > 5000) freq = 5000;

    Serial.print("Frekuensi: ");
    Serial.println(freq);
  }

  lastA = aState;
  lastB = bState;

  // ==== SWITCH UNTUK ON/OFF BUZZER ====
  static bool lastButton = HIGH;
  bool buttonState = digitalRead(SW);

  if (lastButton == HIGH && buttonState == LOW) {
    buzzerOn = !buzzerOn; // toggle buzzer
    Serial.print("Buzzer ");
    Serial.println(buzzerOn ? "ON" : "OFF");
  }
  lastButton = buttonState;

  // ==== KELUARAN BUZZER ====
  if (buzzerOn) tone(BUZZER, freq);
  else noTone(BUZZER);

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}
