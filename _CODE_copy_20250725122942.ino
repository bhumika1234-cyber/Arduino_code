#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Constants
const int piezoPin = A0;
const int ledPin = 8;
const int threshold = 100;        // Minimum analog value to detect a step
const long ledInterval = 100;     // LED off delay in ms
const int debounceDelay = 200;    // Debounce delay after step in ms

// Variables
bool stepDetected = false;
int stepCount = 0;
unsigned long ledOnTime = 0;
float vout, vin;

LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  lcd.init();
  lcd.backlight();

  // Startup display
  lcd.setCursor(0, 0);
  lcd.print("FOOT STEP POWER");
  lcd.setCursor(0, 1);
  lcd.print("GENERATOR");
  delay(2000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("STEP COUNT:");
  lcd.setCursor(0, 1);
  lcd.print("VOLTAGE: 0mV");
}

void loop() {
  unsigned long currentMillis = millis();

  // Read and average analog input
  int total = 0;
  const int samples = 10;
  for (int i = 0; i < samples; i++) {
    total += analogRead(piezoPin);
    delay(2);
  }
  int analogValue = total / samples;

  // Step detection
  if (analogValue > threshold && !stepDetected) {
    stepDetected = true;
    stepCount++;

    // Turn LED ON and record time
    digitalWrite(ledPin, HIGH);
    ledOnTime = currentMillis;

    // Update LCD step count
    lcd.setCursor(12, 0);
    lcd.print("    ");  // Clear old count
    lcd.setCursor(12, 0);
    lcd.print(stepCount);

    // Voltage calculation
    vout = (analogValue * 5.0) / 1024.0;
    vin = (vout / 0.040909) * 100.0;  // Convert to mV

    // Display voltage
    lcd.setCursor(9, 1);
    lcd.print("     "); // Clear previous value
    lcd.setCursor(9, 1);
    if (vin > 100.0) {
      lcd.print((int)vin);
      lcd.print("mV");
    } else {
      lcd.print("0mV");
    }

    // Serial output
    Serial.print("Step: ");
    Serial.print(stepCount);
    Serial.print(" | Voltage: ");
    Serial.print((int)vin);
    Serial.println(" mV");

    delay(debounceDelay); // Debounce delay
  } else if (analogValue <= threshold) {
    stepDetected = false;
  }

  // Turn off LED after interval
  if (digitalRead(ledPin) == HIGH && (currentMillis - ledOnTime >= ledInterval)) {
    digitalWrite(ledPin, LOW);
  }
}
