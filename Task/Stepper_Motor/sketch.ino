#define STEP_PIN 9
#define DIR_PIN 8
#define ENABLE_PIN 10
int nilaistep = 2000;

void setup() {
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  digitalWrite(ENABLE_PIN, LOW);
  Serial.begin(9600);
}



void loop() {
  Serial.println("Stepper maju... (Searah Jarum Jam)");
  digitalWrite(DIR_PIN, HIGH);
  moveStepper(nilaistep);
  delay(1000);

  Serial.println("Stepper mundur... (Berlawanan Jarum Jam)");
  digitalWrite(DIR_PIN, LOW);
  moveStepper(nilaistep);
  delay(1000);
}

void moveStepper(int steps) {
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(800);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(800);
  }
}
