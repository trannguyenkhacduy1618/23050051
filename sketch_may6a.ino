#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <math.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// WiFi
const char* ssid = "Fablab 2.4G";
const char* password = "Fira@2024";

// MQTT
const char* mqtt_server = "192.168.69.197";  
const int mqtt_port = 8883;
const char* mqtt_user = "master:espclient";       
const char* mqtt_password = "AW16vkkxjXzAp6yviZVOebhOXBdNiOdQ";       
const char* client_id = "client1234";             

// Asset & Attributes
const char* assetId = "38r75BrlYpigQRYk5XUAUy";
const char* attributeControlMode = "controlMode";
const char* attributeServoState = "servoState";
const char* attributeName = "writeAttribute";         // dB
const char* servoStatusAttribute = "WDst";            // OPEN / CLOSED

// MQTT
WiFiClientSecure secureClient;
PubSubClient client(secureClient);

// Phần cứng
Servo myServo;
LiquidCrystal_I2C lcd(0x27, 16, 2);
bool Window = false;

// Trạng thái điều khiển
bool isAutoMode = true;
bool manualServoState = false;

// Cảm biến âm thanh
#define SOUND_ANALOG_PIN 34
#define SOUND_DIGITAL_PIN 27
#define SERVO_PIN 14
#define SOUND_DB_THRESHOLD 41.0

float readSoundDb(int analogPin, int sampleWindow = 50) {
  unsigned long startMillis = millis();
  int signalMax = 0;
  int signalMin = 1023;
  while (millis() - startMillis < sampleWindow) {
    int sample = analogRead(analogPin);
    signalMax = max(signalMax, sample);
    signalMin = min(signalMin, sample);
  }
  int peakToPeak = signalMax - signalMin;
  peakToPeak = max(peakToPeak, 1);
  return 20.0 * log10((float)peakToPeak);
}

void setup_wifi() {
  Serial.begin(115200);
  Serial.print("Đang kết nối WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi đã kết nối");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Kết nối MQTT...");
    if (client.connect(client_id, mqtt_user, mqtt_password)) {
      Serial.println(" -> OK");

      String topic1 = "master/" + String(client_id) + "/attribute/" + attributeControlMode + "/" + assetId;
      String topic2 = "master/" + String(client_id) + "/attribute/" + attributeServoState + "/" + assetId;
      client.subscribe(topic1.c_str());
      client.subscribe(topic2.c_str());

      Serial.println("Đã subscribe: " + topic1);
      Serial.println("Đã subscribe: " + topic2);
    } else {
      Serial.print("Lỗi: ");
      Serial.println(client.state());
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (int i = 0; i < length; i++) msg += (char)payload[i];
  bool value = (msg == "true");

  String topicStr = String(topic);
  if (topicStr.indexOf(attributeControlMode) != -1) {
    isAutoMode = value;
Serial.println("→ Nhận controlMode: " + String(isAutoMode ? "AUTO" : "MANUAL"));
  }
  if (topicStr.indexOf(attributeServoState) != -1) {
    manualServoState = value;
    Serial.println("→ Nhận servoState (manual): " + String(manualServoState ? "OPEN" : "CLOSED"));
  }
}

void setup() {
  setup_wifi();
  secureClient.setInsecure(); 
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  myServo.attach(SERVO_PIN);
  myServo.write(0);
  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ESP Ready");
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  float soundDb = readSoundDb(SOUND_ANALOG_PIN);

  if (isAutoMode) {
    if (soundDb > SOUND_DB_THRESHOLD && !Window) {
      myServo.write(90);
      Window = true;
    } else if (soundDb <= SOUND_DB_THRESHOLD && Window) {
      myServo.write(0);
      Window = false;
    }
  } else {
    if (manualServoState) {
      myServo.write(90);
      Window = true;
    } else {
      myServo.write(0);
      Window = false;
    }
  }

  // Hiển thị
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("dB: ");
  lcd.print(soundDb, 1);
  lcd.setCursor(0, 1);
  lcd.print(isAutoMode ? "AUTO " : "MANUAL ");
  lcd.print(Window ? "OPEN " : "CLOSED");

  // Publish dữ liệu lên MQTT
  String pubNoise = "master/" + String(client_id) + "/writeattributevalue/" + attributeName + "/" + String(assetId);
  String pubServoStatus = "master/" + String(client_id) + "/writeattributevalue/" + servoStatusAttribute + "/" + String(assetId);
  String pubControlMode = "master/" + String(client_id) + "/writeattributevalue/" + attributeControlMode + "/" + String(assetId);
  String pubServoState = "master/" + String(client_id) + "/writeattributevalue/" + attributeServoState + "/" + String(assetId);

  client.publish(pubNoise.c_str(), String(soundDb, 2).c_str());
  client.publish(pubServoStatus.c_str(), Window ? "\"OPEN\"" : "\"CLOSED\"");
  client.publish(pubControlMode.c_str(), isAutoMode ? "true" : "false");
  client.publish(pubServoState.c_str(), manualServoState ? "true" : "false");

  delay(5000);
}
