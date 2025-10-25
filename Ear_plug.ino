#include <HX711.h>
#include <WiFi.h>

// ===== WiFi Settings =====
const char* ssid = "ESP32C3_AP";
const char* password = "12345678";

WiFiServer server(80);

// ===== HX711 Pins =====
#define DT1 D3
#define SCK1 D2
#define DT2 D5
#define SCK2 D4

#define PUMP_SUCK D7
#define PUMP_INJECT D8

HX711 pressureSensor1;
HX711 pressureSensor2;

#define NUM_SAMPLES 10
#define ALPHA 0.05
#define PRESSURE_TOLERANCE 0.3  // atm

long referencePressure1 = 0;
long referencePressure2 = 0;
float filteredPressure1 = 0;
float filteredPressure2 = 0;
float atm1 = 1.0, atm2 = 1.0, pressureDiff = 0;

void setup() {
  pinMode(PUMP_SUCK, OUTPUT);
  pinMode(PUMP_INJECT, OUTPUT);
  Serial.begin(115200);
  delay(500);

  // WiFi AP
  WiFi.mode(WIFI_AP);
  if (WiFi.softAP(ssid, password)) {
    Serial.println("✅ Access Point started!");
    Serial.print("ESP32-C3 AP IP: ");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("❌ Failed to start AP!");
  }

  server.begin();
  Serial.println("Server started.");

  // Pin setup
  pinMode(PUMP_SUCK, OUTPUT);
  pinMode(PUMP_INJECT, OUTPUT);
  digitalWrite(PUMP_SUCK, HIGH);
  digitalWrite(PUMP_INJECT, HIGH);

  // HX711 init
  pressureSensor1.begin(DT1, SCK1);
  pressureSensor2.begin(DT2, SCK2);

  calibrateSensors();
}

// ------------------- Functions -------------------

void calibrateSensors() {
  Serial.println("Calibrating sensors...");
  long sum1 = 0, sum2 = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    sum1 += pressureSensor1.read();
    sum2 += pressureSensor2.read();
    delay(100);
  }
  referencePressure1 = sum1 / NUM_SAMPLES;
  referencePressure2 = sum2 / NUM_SAMPLES;
  filteredPressure1 = referencePressure1;
  filteredPressure2 = referencePressure2;
  Serial.println("Calibration complete!");
}

void readSensors() {
  long raw1 = pressureSensor1.read();
  long raw2 = pressureSensor2.read();

  // EMA smoothing
  filteredPressure1 = ALPHA * raw1 + (1 - ALPHA) * filteredPressure1;
  filteredPressure2 = ALPHA * raw2 + (1 - ALPHA) * filteredPressure2;

  atm1 = 1.0 + ((referencePressure1 - filteredPressure1) / referencePressure1);
  atm2 = 1.0 + ((referencePressure2 - filteredPressure2) / referencePressure2);

  pressureDiff = atm1 - atm2;
}

void controlPumps() {
  if (abs(pressureDiff) <= PRESSURE_TOLERANCE) {
    Serial.println("stop");
    digitalWrite(PUMP_SUCK, LOW);
    digitalWrite(PUMP_INJECT, LOW);
  } else if (pressureDiff > PRESSURE_TOLERANCE) {
    Serial.println("IN");
    digitalWrite(PUMP_SUCK, LOW);
    digitalWrite(PUMP_INJECT, HIGH);
  } else if (pressureDiff < -PRESSURE_TOLERANCE) {
    Serial.println("OUT");
    digitalWrite(PUMP_SUCK, HIGH);
    digitalWrite(PUMP_INJECT, LOW);
  }
}

void transmitData() {
  WiFiClient client = server.available();
  if (client) {
    String request = client.readStringUntil('\r');
    client.flush();

    // Manual pump override via URL
    if (request.indexOf("/ON") != -1) {
      digitalWrite(PUMP_INJECT, HIGH);
      digitalWrite(PUMP_SUCK, LOW);
      Serial.println("Pump ON");
    } else if (request.indexOf("/OFF") != -1) {
      digitalWrite(PUMP_INJECT, LOW);
      digitalWrite(PUMP_SUCK, LOW);
      Serial.println("Pump OFF");
    }

    // Send sensor data
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/plain");
    client.println("Connection: close");
    client.println();
    client.print(atm1, 3);
    client.print(",");
    client.print(atm2, 3);
    client.print(",");
    client.println(pressureDiff, 3);
    delay(1);
  }
}

// ------------------- Main Loop -------------------

void loop() {
  readSensors();
  controlPumps();
  transmitData();

  Serial.print("Sensor1: "); Serial.print(atm1, 3);
  Serial.print(" | Sensor2: "); Serial.print(atm2, 3);
  Serial.print(" | ΔP: "); Serial.println(pressureDiff, 3);

  delay(200);
}
