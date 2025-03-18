#include <SPI.h>

const int latchPin = 7; // RCLK_HC74595
const int pwmBrightnessPin = 9; // OE_HC74595
const int adcChipSelectPin = 10; // CS_MCP3008

const byte adcSingleCh0 = 0x08; //0b1000 1 = SGL/DIFF (single-ended), 000 = CH0
const byte adcSingleCh1 = 0x09; //0b1001 1 = SGL/DIFF (single-ended), 001 = CH1

byte leds = 0;
unsigned long previousMillis = 0;
int currentLED = 0;

void setup() 
{
  pinMode(latchPin, OUTPUT);
  pinMode(pwmBrightnessPin, OUTPUT);
  pinMode(adcChipSelectPin, OUTPUT);

  SPI.begin();
  Serial.begin(9600);
  digitalWrite(adcChipSelectPin, HIGH);
}

void loop() 
{
  int potSpeedVal = readMCP3008(adcSingleCh0);
  int potBrightnessVal = readMCP3008(adcSingleCh1);

  int brightness = map(potBrightnessVal, 0, 1023, 0, 255);
  analogWrite(pwmBrightnessPin, brightness);

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= potSpeedVal) 
  {
    previousMillis = currentMillis;
    leds = (1 << currentLED); 
    updateShiftRegister();

    currentLED++;
    if (currentLED > 7)
    {
      currentLED = 0;
      leds = 0;
    }
  }
}

int readMCP3008(byte readAddress)
{
  byte dataMSB = 0;
  byte dataLSB = 0;
  byte JUNK = 0x00;
  
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(adcChipSelectPin, LOW);
  SPI.transfer(0x01); // start ADC communication
  dataMSB = SPI.transfer(0b10000000) & 0x03; // Mask 2 unused bits after SGL/DIFF and CH using 0x03
  dataLSB = SPI.transfer(JUNK); // Read remaining 8 bits of data
  digitalWrite(adcChipSelectPin, HIGH);
  SPI.endTransaction();

  return dataMSB << 8 | dataLSB; // Add together first 2 bits and remaining 8 bits
}

void updateShiftRegister()
{
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(latchPin, LOW);
  SPI.transfer(leds);
  digitalWrite(latchPin, HIGH);
  SPI.endTransaction();
}

