//const int buttonPin = 16;
//const int ledPin = 15;

//0x0 = LOW, 0x1 = HIGH
volatile byte prevButtonState = 0x0;
byte ledMode = 0x0;

unsigned long debounceTime = 50;
unsigned long prevTimeButtonStateChanged = 0;

void setup() {
  //pinMode(buttonPin, INPUT);
  //pinMode(ledPin, OUTPUT);
  DDRB |= (1 << PB1);
  DDRB &= ~(1 << PB2);
  attachInterrupt(0, button_ISR, CHANGE);
}

void loop() {
  if (millis() - prevTimeButtonStateChanged > debounceTime) {
    byte buttonState = PINB & (1 << PB2);

    if (buttonState != prevButtonState) {
      prevButtonState = buttonState;
      prevTimeButtonStateChanged = millis();
      
      if (buttonState == 0x0) {
        if (ledMode == 0x1) {
          //digitalWrite(ledPin, LOW);
          PORTB &= ~(1 << PB1);
          ledMode = 0x0;
        } else if (ledMode == 0x0) {
          //digitalWrite(ledPin, HIGH);
          PORTB |= (1 << PB1);
          ledMode = 0x1;
        }
      }
    }
  }
}

void button_ISR() {
  
}
