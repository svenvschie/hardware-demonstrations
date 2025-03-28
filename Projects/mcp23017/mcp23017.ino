#include <Wire.h>
#include <SPI.h>

#define LATCH_PIN 9
#define OE_PWM_PIN 6

#define MCP23017_ADDR 0x20
#define ENCODER_A_PIN 0      // Interrupt pin for encoder A (RX / INT2)
#define ENCODER_B_PIN 4      // Regular input pin for encoder B
#define DEBOUNCE_DELAY 50

// Rotary encoder state
volatile int encoderDelta = 0;

bool mode = false;
bool buttonHeld = false;
unsigned long lastPressTime = 0;
int brightness = 0;


volatile bool lastAState = HIGH;

void handleEncoder() {
  bool a = digitalRead(ENCODER_A_PIN);
  bool b = digitalRead(ENCODER_B_PIN);

  // Only react to rising edge of A
  if (!lastAState && a) {
    encoderDelta += (b ? -1 : 1);
  }

  lastAState = a;
}

void setupMCP23017() {
  // Set GP0–GP2 as inputs
  Wire.beginTransmission(MCP23017_ADDR);
  Wire.write(0x00);  // IODIRA
  Wire.write(0x07);
  Wire.endTransmission();

  // Enable pull-ups on GP0–GP2
  Wire.beginTransmission(MCP23017_ADDR);
  Wire.write(0x0C);  // GPPUA
  Wire.write(0x07);
  Wire.endTransmission();
}

void setup() {
  Wire.begin();
  SPI.begin();
  Serial.begin(115200);
  while (!Serial);  // Wait for USB

  pinMode(LATCH_PIN, OUTPUT);
  pinMode(OE_PWM_PIN, OUTPUT);

  // Rotary encoder pins
  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), handleEncoder, CHANGE);

  setupMCP23017();

  analogWrite(OE_PWM_PIN, 0);  // full brightness (active LOW)
}

void pollButton() {
  Wire.beginTransmission(MCP23017_ADDR);
  Wire.write(0x12);  // GPIOA
  Wire.endTransmission();
  Wire.requestFrom(MCP23017_ADDR, 1);

  if (Wire.available()) {
    uint8_t state = Wire.read();
    bool pressed = (state & 0b00000010) == 0;  // GP2 LOW = pressed
  
    if (pressed && !buttonHeld) {
      buttonHeld = true;
    } else if (!pressed && buttonHeld) {
      unsigned long now = millis();
      if (now - lastPressTime > DEBOUNCE_DELAY) {
        mode = !mode;
        Serial.println("Button toggled mode.");
        lastPressTime = now;
      }
      buttonHeld = false;
    }
  }
}

void updateLED() {
  static int lastBrightness = -1;

  noInterrupts();
  int delta = encoderDelta;
  encoderDelta = 0;
  interrupts();

  if (delta != 0) {
    brightness += delta * 5;
    brightness = constrain(brightness, 0, 255);
  }

  int pwmValue = 255 - brightness;
  analogWrite(OE_PWM_PIN, pwmValue);

  if (brightness != lastBrightness) {
    Serial.print("Brightness: ");
    Serial.println(brightness);
    lastBrightness = brightness;
  }

  digitalWrite(LATCH_PIN, LOW);
  SPI.transfer(mode ? B00000010 : 0);
  digitalWrite(LATCH_PIN, HIGH);
}

void loop() {
  pollButton();     // Now polling the button
  updateLED();      // Handle LED and encoder logic
}