// const int buttonPin = 2;  // PD1
// const int ledPin = 15;    // PB1

volatile unsigned long lastDebounceTime = 0;
const unsigned long debounceTime = 50;
bool ledState = false;

void setup() {
  DDRB |= (1 << PB1);
  DDRD &= ~(1 << PD1);
  PORTD |= (1 << PD1);

  attachInterrupt(1, button_ISR, FALLING);
}

void loop() {

}

void button_ISR() {
  unsigned long currentTime = millis();
  if (currentTime - lastDebounceTime > debounceTime) {
    lastDebounceTime = currentTime;
    
    ledState = !ledState;
    if (ledState) {
      PORTB |= (1 << PB1);
    } else {
      PORTB &= ~(1 << PB1);
    }
  }
}
