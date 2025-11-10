#include <Adafruit_SSD1306.h>

// --- OLED setup ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

#define BTN1 12
#define BTN2 13

void setup() {
  pinMode(BTN1, INPUT_PULLUP);
  pinMode(BTN2, INPUT_PULLUP);

  // OLED init
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)){
    Serial.println("OLED gagal!");
    for(;;);
  }
  display.clearDisplay();
  display.display();
}
void loop()   {
    int b1 = digitalRead(BTN1);
    int b2 = digitalRead(BTN2);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.print("Button1: "); display.println(b1 == LOW ? "Pressed" : "Released");
    display.print("Button2: "); display.println(b2 == LOW ? "Pressed" : "Released");
    display.display();
}