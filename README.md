#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <ThingSpeak.h>
#include "MAX30105.h"
#include "heartRate.h"

MAX30105 particleSensor;
const byte RATE_SIZE = 4; // Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE];     // Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; // Time at which the last beat occurred
long lastPrint = 0; // Time at which the last values were printed
int beatAvg;

OneWire oneWire(17); // Setup a oneWire instance to communicate with any OneWire devices
DallasTemperature sensors(&oneWire);

const char *ssid = "Meow";
const char *password = "meow123456";
const char *thingSpeakApiKey = "EBU6RAY5969O8U1O";
const int thingSpeakChannel = 2399955; // Replace with your actual ThingSpeak channel number
const int dataPin1 = 17; 
const int dataPin2 = 22;

WiFiClient client; // Declare a WiFiClient object

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing...");

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  int attemptCount = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
    attemptCount++;
    if (attemptCount > 20) {
      Serial.println("Failed to connect to WiFi. Exiting...");
      return;
    }
  }

  Serial.println("Connected to WiFi");

  // Initialize ThingSpeak
  ThingSpeak.begin(client);

  // Initialize MAX30105 sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) // Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30102 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");
  particleSensor.setup();                // Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); // Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0);  // Turn off Green LED

  // Start the DS18B20 sensor
  sensors.begin();
}

void loop() {
  // Read temperature
  sensors.requestTemperatures();
  float sensorValue1 = sensors.getTempCByIndex(0);

  // Print sensor values
  Serial.print("Temperature Value: ");
  Serial.println(sensorValue1);

  long irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true) {
    // We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    float beatsPerMinute = 60000 / delta;

    if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      rates[rateSpot++] = (byte)beatsPerMinute; // Store this reading in the array
      rateSpot %= RATE_SIZE;                    // Wrap variable

      // Take average of readings
      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  if (millis() - lastPrint >= 500) {
    // Map beatAvg to a range of 60 to 120
    int mappedBPM = map(beatAvg, 0, 255, 60, 120);

    Serial.print("Your BPM is ;) = ");
    Serial.print(mappedBPM);

    // Print the message along with the temperature value
    Serial.print(", Your temperature is ");
    Serial.print(sensorValue1);
    Serial.println("ºC");

    if (irValue < 50000)
      Serial.print(" No finger?");

    Serial.println();

    // Send data to ThingSpeak
    ThingSpeak.setField(1, sensorValue1); // Use Field 1 for your data
    ThingSpeak.setField(2, mappedBPM); // Use Field 2 for your BPM data
    int httpCode = ThingSpeak.writeFields(thingSpeakChannel, thingSpeakApiKey);

    Serial.print("HTTP response code: ");
    Serial.println(httpCode);

    if (httpCode == 200) {
      Serial.println("Data sent to ThingSpeak successfully");
    } else {
      Serial.println("Error sending data to ThingSpeak");
      Serial.println(ThingSpeak.getLastReadStatus());
    }

    lastPrint = millis(); // Update lastPrint time
  }

  delay(15000); // Send data every 15 seconds
}
