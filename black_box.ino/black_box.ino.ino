#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <math.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

const char* WIFI_SSID = "visakh";
const char* WIFI_PASSWORD = "0987654321";

const char* MQTT_BROKER =
  "ed90876d116142e58d2b22006e6948c8.s1.eu.hivemq.cloud";
const int MQTT_PORT = 8883;
const char* MQTT_USERNAME = "hivemq.webclient.1791356460877";
const char* MQTT_PASSWORD = "ojs2HCG#MH6sTs$BdqlHtsaFFTbvv!8t";
const char* MQTT_CLIENT_ID = "ESP32_BLACK_BOX_STANISON";
const char* MQTT_TOPIC = "blackbox/data";

WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

#define SDA_PIN 21
#define SCL_PIN 22
#define MQ3_PIN 34
#define SW420_PIN 27
#define RELAY_PIN 26

#define RELAY_RUN HIGH
#define RELAY_STOP LOW

#define MQ3_THRESHOLD 800
#define IMPACT_THRESHOLD 5.8
#define TILT_THRESHOLD 35.0
#define FINGER_THRESHOLD 14000

#define MPU6050_ADDRESS 0x68
#define PWR_MGMT_1 0x6B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B
#define WHO_AM_I 0x75
#define ACCEL_SCALE 4096.0

float ax = 0;
float ay = 0;
float az = 0;
float totalAcceleration = 0;
float tiltAngle = 0;

MAX30105 particleSensor;

long irValue = 0;
float heartRate = 0;
float averageHeartRate = 0;
unsigned long lastBeat = 0;

#define RATE_SIZE 4
byte rates[RATE_SIZE] = {0, 0, 0, 0};
byte rateSpot = 0;

int mq3Value = 0;
int sw420Value = LOW;

bool accidentDetected = false;
bool accidentPublished = false;
String eventReason = "";

float savedAX = 0;
float savedAY = 0;
float savedAZ = 0;
float savedAcceleration = 0;
float savedTilt = 0;
int savedMQ3 = 0;
int savedSW420 = LOW;
long savedIR = 0;
float savedHeartRate = 0;

unsigned long previousDisplayTime = 0;
unsigned long previousSerialTime = 0;

const unsigned long DISPLAY_INTERVAL = 500;
const unsigned long SERIAL_INTERVAL = 1000;

void writeMPURegister(byte reg, byte value)
{
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void readMPURegisters(byte reg, byte *buffer, byte length)
{
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU6050_ADDRESS, length, true);

  for (byte i = 0; i < length; i++)
  {
    if (Wire.available())
      buffer[i] = Wire.read();
  }
}

void initializeMPU6050()
{
  Serial.println();
  Serial.println("Initializing MPU6050...");

  writeMPURegister(PWR_MGMT_1, 0x00);
  delay(100);

  writeMPURegister(ACCEL_CONFIG, 0x10);
  delay(100);

  byte whoAmI = 0;
  readMPURegisters(WHO_AM_I, &whoAmI, 1);

  Serial.print("MPU6050 WHO_AM_I: 0x");
  if (whoAmI < 16)
    Serial.print("0");
  Serial.println(whoAmI, HEX);

  Serial.println("MPU6050 accelerometer initialized.");
}

void readMPU6050()
{
  byte data[6];

  readMPURegisters(ACCEL_XOUT_H, data, 6);

  int16_t rawAX = ((int16_t)data[0] << 8) | data[1];
  int16_t rawAY = ((int16_t)data[2] << 8) | data[3];
  int16_t rawAZ = ((int16_t)data[4] << 8) | data[5];

  ax = rawAX / ACCEL_SCALE;
  ay = rawAY / ACCEL_SCALE;
  az = rawAZ / ACCEL_SCALE;

  totalAcceleration = sqrt(
    (ax * ax) +
    (ay * ay) +
    (az * az)
  );

  if (totalAcceleration > 0.01)
  {
    float value = az / totalAcceleration;

    if (value > 1.0)
      value = 1.0;

    if (value < -1.0)
      value = -1.0;

    tiltAngle = acos(value) * 180.0 / PI;
  }
}

void initializeMAX30102()
{
  Serial.println();
  Serial.println("Initializing MAX30102...");

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST))
  {
    Serial.println("ERROR: MAX30102 NOT FOUND!");

    while (1)
      delay(100);
  }

  Serial.println("MAX30102 FOUND!");

  particleSensor.setup(30, 4, 2, 100, 411, 4096);
  particleSensor.setPulseAmplitudeRed(0x20);
  particleSensor.setPulseAmplitudeIR(0x20);
  particleSensor.setPulseAmplitudeGreen(0);

  Serial.println("MAX30102 READY.");
}

void readHeartRate()
{
  particleSensor.check();

  while (particleSensor.available())
  {
    irValue = particleSensor.getFIFOIR();
    particleSensor.nextSample();

    if (irValue < FINGER_THRESHOLD)
    {
      heartRate = 0;
      averageHeartRate = 0;
      lastBeat = 0;
      continue;
    }

    if (checkForBeat(irValue))
    {
      unsigned long currentTime = millis();

      if (lastBeat != 0)
      {
        unsigned long delta = currentTime - lastBeat;

        if (delta > 300 && delta < 2000)
        {
          float bpm = 60000.0 / delta;

          if (bpm >= 40 && bpm <= 180)
          {
            heartRate = bpm;
            rates[rateSpot] = (byte)bpm;
            rateSpot++;
            rateSpot %= RATE_SIZE;

            int sum = 0;
            int count = 0;

            for (int i = 0; i < RATE_SIZE; i++)
            {
              if (rates[i] > 0)
              {
                sum += rates[i];
                count++;
              }
            }

            if (count > 0)
              averageHeartRate = (float)sum / count;
          }
        }
      }

      lastBeat = currentTime;
    }
  }
}

void readAllSensors()
{
  readHeartRate();
  readMPU6050();
  mq3Value = analogRead(MQ3_PIN);
  sw420Value = digitalRead(SW420_PIN);
}

void checkAccident()
{
  if (accidentDetected)
    return;

  if (mq3Value > MQ3_THRESHOLD)
  {
    triggerAccident("ALCOHOL DETECTED");
    return;
  }

  if (sw420Value == HIGH)
  {
    triggerAccident("IMPACT / VIBRATION");
    return;
  }

  if (totalAcceleration > IMPACT_THRESHOLD)
  {
    triggerAccident("HIGH ACCELERATION");
    return;
  }

  if (tiltAngle > TILT_THRESHOLD)
  {
    triggerAccident("VEHICLE TILT");
    return;
  }
}

void printBlackBoxRecord()
{
  Serial.println();
  Serial.println("========================================");
  Serial.println("          BLACK BOX EVENT RECORD");
  Serial.println("========================================");

  Serial.print("EVENT       : ");
  Serial.println(eventReason);

  Serial.println("----------------------------------------");

  Serial.print("AX          : ");
  Serial.print(savedAX, 2);
  Serial.println(" g");

  Serial.print("AY          : ");
  Serial.print(savedAY, 2);
  Serial.println(" g");

  Serial.print("AZ          : ");
  Serial.print(savedAZ, 2);
  Serial.println(" g");

  Serial.print("ACCELERATION: ");
  Serial.print(savedAcceleration, 2);
  Serial.println(" g");

  Serial.print("TILT        : ");
  Serial.print(savedTilt, 1);
  Serial.println(" deg");

  Serial.println("----------------------------------------");

  Serial.print("MQ-3        : ");
  Serial.println(savedMQ3);

  Serial.print("SW-420      : ");
  Serial.println(savedSW420 == HIGH ? "DETECTED" : "NORMAL");

  Serial.print("MAX30102 IR : ");
  Serial.println(savedIR);

  Serial.print("HEART RATE  : ");
  if (savedHeartRate > 0)
  {
    Serial.print(savedHeartRate, 1);
    Serial.println(" BPM");
  }
  else
  {
    Serial.println("--");
  }

  Serial.println("----------------------------------------");
  Serial.println("MOTOR       : STOPPED");
  Serial.println("BLACK BOX   : EVENT SAVED");
  Serial.println("========================================");
}

void updateOLED()
{
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  display.setCursor(38, 0);
  display.println("BLACK BOX");

  display.setCursor(0, 12);
  display.print("AX:");
  display.print(ax, 2);
  display.print("g");

  display.setCursor(65, 12);
  display.print("AY:");
  display.print(ay, 2);
  display.print("g");

  display.setCursor(0, 23);
  display.print("AZ:");
  display.print(az, 2);
  display.print("g");

  display.setCursor(65, 23);
  display.print("ACC:");
  display.print(totalAcceleration, 1);

  display.setCursor(0, 35);
  display.print("TILT:");
  display.print(tiltAngle, 1);
  display.print(" deg");

  display.setCursor(65, 35);
  display.print("MQ:");
  display.print(mq3Value);

  display.setCursor(0, 48);
  display.print("HR:");

  if (averageHeartRate > 0)
  {
    display.print(averageHeartRate, 0);
    display.print(" BPM");
  }
  else
  {
    display.print("--");
  }

  display.setCursor(65, 48);
  display.print("MOTOR:RUN");

  display.display();
}

void showAccidentScreen()
{
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  display.setCursor(25, 0);
  display.println("ACCIDENT!");

  display.setCursor(0, 11);
  display.print("REASON:");

  if (eventReason == "ALCOHOL DETECTED")
    display.println(" ALCOHOL");
  else if (eventReason == "IMPACT / VIBRATION")
    display.println(" IMPACT");
  else if (eventReason == "HIGH ACCELERATION")
    display.println(" HIGH ACCEL");
  else
    display.println(" VEHICLE TILT");

  display.setCursor(0, 22);
  display.print("ACC:");
  display.print(savedAcceleration, 2);
  display.print("g");

  display.setCursor(68, 22);
  display.print("TILT:");
  display.print(savedTilt, 1);

  display.setCursor(0, 33);
  display.print("MQ3:");
  display.print(savedMQ3);

  display.setCursor(68, 33);
  display.print("SW:");
  display.print(savedSW420 == HIGH ? "YES" : "NO");

  display.setCursor(0, 44);
  display.print("HR:");

  if (savedHeartRate > 0)
  {
    display.print(savedHeartRate, 0);
    display.print(" BPM");
  }
  else
  {
    display.print("--");
  }

  display.setCursor(68, 44);
  display.print("IR:");
  display.print(savedIR);

  display.setCursor(0, 55);
  display.print("MOTOR: STOP");

  display.display();
}

void printLiveData()
{
  Serial.println();
  Serial.println("----------------------------------------");

  Serial.print("AX: ");
  Serial.print(ax, 2);
  Serial.print(" g | AY: ");
  Serial.print(ay, 2);
  Serial.print(" g | AZ: ");
  Serial.print(az, 2);
  Serial.println(" g");

  Serial.print("ACCELERATION: ");
  Serial.print(totalAcceleration, 2);
  Serial.print(" g | TILT: ");
  Serial.print(tiltAngle, 1);
  Serial.println(" deg");

  Serial.print("MQ-3: ");
  Serial.print(mq3Value);

  Serial.print(" | SW-420: ");
  Serial.print(sw420Value == HIGH ? "DETECTED" : "NORMAL");

  Serial.print(" | IR: ");
  Serial.print(irValue);

  Serial.print(" | HEART RATE: ");

  if (averageHeartRate > 0)
  {
    Serial.print(averageHeartRate, 1);
    Serial.print(" BPM");
  }
  else
  {
    Serial.print("--");
  }

  Serial.println(" | MOTOR: RUNNING");
  Serial.println("----------------------------------------");
}

void connectWiFi()
{
  Serial.println();
  Serial.print("Connecting to WiFi");

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected!");

  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void connectMQTT()
{
  while (!mqttClient.connected())
  {
    Serial.println();
    Serial.println("Connecting to HiveMQ Cloud...");

    Serial.print("Broker: ");
    Serial.println(MQTT_BROKER);

    Serial.print("Port: ");
    Serial.println(MQTT_PORT);

    if (mqttClient.connect(
          MQTT_CLIENT_ID,
          MQTT_USERNAME,
          MQTT_PASSWORD))
    {
      Serial.println("MQTT CONNECTED!");
      Serial.print("Topic: ");
      Serial.println(MQTT_TOPIC);
      Serial.println("MQTT: ACCIDENT DATA ONLY");
    }
    else
    {
      Serial.print("MQTT FAILED, rc=");
      Serial.println(mqttClient.state());
      delay(3000);
    }
  }
}

void publishAccidentMQTTData()
{
  if (!mqttClient.connected())
    return;

  char message[700];

  const char* swStatus =
    (savedSW420 == HIGH) ? "DETECTED" : "NORMAL";

  snprintf(
    message,
    sizeof(message),
    "{"
    "\"ax\":%.2f,"
    "\"ay\":%.2f,"
    "\"az\":%.2f,"
    "\"acceleration\":%.2f,"
    "\"tilt\":%.1f,"
    "\"mq3\":%d,"
    "\"sw420\":\"%s\","
    "\"ir\":%ld,"
    "\"heart_rate\":%.1f,"
    "\"motor\":\"STOPPED\","
    "\"accident\":\"YES\","
    "\"reason\":\"%s\""
    "}",
    savedAX,
    savedAY,
    savedAZ,
    savedAcceleration,
    savedTilt,
    savedMQ3,
    swStatus,
    savedIR,
    savedHeartRate,
    eventReason.c_str()
  );

  bool success = mqttClient.publish(
    MQTT_TOPIC,
    message,
    true
  );

  if (success)
  {
    Serial.println();
    Serial.println("ACCIDENT DATA SENT TO MQTT");
    Serial.println(message);
  }
  else
  {
    Serial.println("ACCIDENT MQTT PUBLISH FAILED");
  }
}

void triggerAccident(String reason)
{
  if (accidentDetected)
    return;

  // Freeze the exact readings from the accident detection cycle.
  accidentDetected = true;
  eventReason = reason;

  savedAX = ax;
  savedAY = ay;
  savedAZ = az;
  savedAcceleration = totalAcceleration;
  savedTilt = tiltAngle;
  savedMQ3 = mq3Value;
  savedSW420 = sw420Value;
  savedIR = irValue;
  savedHeartRate = averageHeartRate;

  // Stop motor immediately.
  digitalWrite(RELAY_PIN, RELAY_STOP);

  // Print one accident record.
  Serial.println();
  Serial.println("!!! ACCIDENT DETECTED !!!");
  Serial.print("REASON: ");
  Serial.println(eventReason);

  printBlackBoxRecord();

  // Show frozen accident values.
  showAccidentScreen();

  // Publish the frozen accident snapshot only once.
  if (mqttClient.connected() && !accidentPublished)
  {
    publishAccidentMQTTData();
    accidentPublished = true;
  }
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("========================================");
  Serial.println("        ESP32 VEHICLE BLACK BOX");
  Serial.println("========================================");

  Wire.begin(SDA_PIN, SCL_PIN);

  pinMode(MQ3_PIN, INPUT);
  pinMode(SW420_PIN, INPUT);
  pinMode(RELAY_PIN, OUTPUT);

  // Motor starts running.
  digitalWrite(RELAY_PIN, RELAY_RUN);

  if (!display.begin(
        SSD1306_SWITCHCAPVCC,
        OLED_ADDRESS))
  {
    Serial.println("OLED NOT FOUND!");

    while (1)
      delay(100);
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  display.setCursor(25, 20);
  display.println("BLACK BOX");

  display.setCursor(15, 35);
  display.println("INITIALIZING...");

  display.display();
  delay(1000);

  initializeMPU6050();
  initializeMAX30102();

  connectWiFi();

  espClient.setInsecure();

  mqttClient.setServer(
    MQTT_BROKER,
    MQTT_PORT
  );

  connectMQTT();

  Serial.println();
  Serial.println("========================================");
  Serial.println("       BLACK BOX SYSTEM READY");
  Serial.println("========================================");
  Serial.println("Motor: RUNNING");
  Serial.println("MQTT: ACCIDENT DATA ONLY");
  Serial.print("MQTT Topic: ");
  Serial.println(MQTT_TOPIC);
  Serial.println();

  delay(1000);
}

void loop()
{
  // Once an accident occurs, freeze everything.
  if (accidentDetected)
  {
    mqttClient.loop();
    delay(10);
    return;
  }

  // Normal driving.
  if (!mqttClient.connected())
    connectMQTT();

  mqttClient.loop();

  // Continuously read sensors while motor is running.
  readAllSensors();

  // Check accident immediately.
  checkAccident();

  // triggerAccident() already froze the snapshot.
  if (accidentDetected)
    return;

  // Live Serial values during normal driving.
  if (millis() - previousSerialTime >= SERIAL_INTERVAL)
  {
    previousSerialTime = millis();
    printLiveData();
  }

  // Live OLED values during normal driving.
  if (millis() - previousDisplayTime >= DISPLAY_INTERVAL)
  {
    previousDisplayTime = millis();
    updateOLED();
  }
}
