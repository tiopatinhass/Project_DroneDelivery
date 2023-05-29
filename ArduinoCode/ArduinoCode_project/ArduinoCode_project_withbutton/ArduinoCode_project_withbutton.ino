#include <DHT.h>
#include <ArduinoJson.h>
#include <Servo.h>

#define DHT_PIN 5    // Pin connected to the DHT sensor
#define DHT_TYPE DHT11   // DHT sensor type

Servo servo; // servo object representing the MG 996R servo
DHT dht(DHT_PIN, DHT_TYPE);

const int buttonPin = 0;  // Pin connected to the external button
bool buttonState = false;  // Current state of the button
bool previousButtonState = false;  // Previous state of the button
unsigned long debounceDelay = 10;  // Debounce delay in milliseconds
unsigned long lastDebounceTime = 0;  // Last time the button state changed

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }
  pinMode(LED_BUILTIN, OUTPUT);

  // Servo Motor code
  servo.attach(4); // servo is wired to Arduino on digital pin 4
  servo.write(180); // move MG996R's shaft to angle 0° (Fechado)

  // DHT Sensor code
  dht.begin();

  // External button setup
  pinMode(buttonPin, INPUT_PULLUP);
}

void loop() {
  // Read sensor data
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  // Handle serial input
  if (Serial.available() > 0) {
    char c = Serial.read();
    switch (c) {
      case '1':
        // OPEN servo motor
        if (servo.read() == 0) {
          servo.write(180); // move MG996R's shaft to angle 180° (It is an open position)
        }
        Serial.write("A"); // Send back 'A' as acknowledgement
        break;
      case '2':
        // Close servo motor
        if (servo.read() == 180) {
          servo.write(0); // move MG996R's shaft to angle 0° (It is a close position)
        }
        Serial.write("B"); // Send back 'B' as acknowledgement
        break;
      case '3':
        // Send DHT11 data
        if (isnan(humidity) || isnan(temperature)) {
          Serial.println(F("Failed to read from DHT sensor!"));
        } else {
          sendDHT11Data(temperature, humidity);
        }
        Serial.write("B"); // Send back 'B' as acknowledgement
        break;
      default:
        Serial.write("E"); // Send back 'E' for error
        break;
    }
  }

  // Check button state with debounce
  if (millis() - lastDebounceTime > debounceDelay) {
    buttonState = digitalRead(buttonPin);

    // Button pressed
    if (buttonState == LOW && previousButtonState == HIGH) {
      if (servo.read() == 0) {
        servo.write(180); // Open the servo if it's closed
      } else {
        servo.write(0); // Close the servo if it's open
      }
    }

    previousButtonState = buttonState;
    lastDebounceTime = millis();
  }
}

void sendDHT11Data(float temperature, float humidity) {
  StaticJsonDocument<128> jsonDoc;
  jsonDoc["temperature"] = temperature;
  jsonDoc["humidity"] = humidity;

  String jsonString;
  serializeJson(jsonDoc, jsonString);

  Serial.println(jsonString);
}
