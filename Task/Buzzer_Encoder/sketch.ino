#define BUZZER 2
#define ENCA 19
#define ENCB 20
#define SW   21 

int lastA = LOW;
int lastB = LOW;
int freq = 1000;
bool buzzerOn = false;

void setup() {
  pinMode(BUZZER, OUTPUT);
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);
  pinMode(SW, INPUT_PULLUP);

  Serial.begin(115200);
  Serial.println("Encoder Buzzer Control");
}

void loop() {
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

  delay(2);
}