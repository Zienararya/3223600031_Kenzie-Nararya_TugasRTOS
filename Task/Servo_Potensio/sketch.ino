#include <ESP32Servo.h>
#define POT 1
#define SERVO_PIN 18

Servo myServo;
void setup() {
    myServo.attach(SERVO_PIN);
}

void loop() {
  int potVal = analogRead(POT);
    int angle = map(potVal, 0, 4095, 0, 180);
    myServo.write(angle);
}