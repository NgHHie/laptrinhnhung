#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>

// ====== WiFi & MQTT config ======
const char* ssid = "KawaII";
const char* password = "dmcayvcl";
const char* mqtt_server = "172.20.10.2"; // IP máy chạy Node.js server

WiFiClient espClient;
PubSubClient client(espClient);

// ====== L298N setup ======
int rightIN1 = 27;
int rightIN2 = 26;
int leftIN3 = 25;
int leftIN4 = 33;
int rightEnable = 14;
int leftEnable = 12;

const int freq = 1000;
const int pwmChannelRight = 0;
const int pwmChannelLeft = 1;
const int resolution = 8;
int dutyCycle = 200;
int timeDelay = 400;

String mqttCommand = "";

void stop() {
  digitalWrite(rightIN1, LOW);
  digitalWrite(rightIN2, LOW);
  digitalWrite(leftIN3, LOW);
  digitalWrite(leftIN4, LOW);
  Serial.println("⛔ Đã dừng");
}

void moveForward() {
  digitalWrite(rightIN1, LOW);
  digitalWrite(rightIN2, HIGH);
  digitalWrite(leftIN3, LOW);
  digitalWrite(leftIN4, HIGH);
  ledcWrite(pwmChannelRight, dutyCycle);
  ledcWrite(pwmChannelLeft, dutyCycle);
  Serial.println("➡️ Tiến");
  delay(timeDelay);
  stop();
}

void moveBackward() {
  digitalWrite(rightIN1, HIGH);
  digitalWrite(rightIN2, LOW);
  digitalWrite(leftIN3, HIGH);
  digitalWrite(leftIN4, LOW);
  ledcWrite(pwmChannelRight, dutyCycle);
  ledcWrite(pwmChannelLeft, dutyCycle);
  Serial.println("⬅️ Lùi");
  delay(timeDelay);
  stop();
}

void turnLeft() {
  digitalWrite(rightIN1, LOW);
  digitalWrite(rightIN2, HIGH);
  digitalWrite(leftIN3, HIGH);
  digitalWrite(leftIN4, LOW);
  ledcWrite(pwmChannelRight, dutyCycle);
  ledcWrite(pwmChannelLeft, dutyCycle);
  Serial.println("↪️ Trái");
  delay(timeDelay);
  stop();
}

void turnRight() {
  digitalWrite(rightIN1, HIGH);
  digitalWrite(rightIN2, LOW);
  digitalWrite(leftIN3, LOW);
  digitalWrite(leftIN4, HIGH);
  ledcWrite(pwmChannelRight, dutyCycle);
  ledcWrite(pwmChannelLeft, dutyCycle);
  Serial.println("↩️ Phải");
  delay(timeDelay);
  stop();
}

void moveLeft() {
  digitalWrite(rightIN1, LOW);
  digitalWrite(rightIN2, HIGH);
  digitalWrite(leftIN3, LOW);
  digitalWrite(leftIN4, HIGH);
  ledcWrite(pwmChannelRight, dutyCycle);
  ledcWrite(pwmChannelLeft, dutyCycle / 2);
  Serial.println("⬅️ Dịch trái");
  delay(timeDelay);
  stop();
}

void moveRight() {
  digitalWrite(rightIN1, LOW);
  digitalWrite(rightIN2, HIGH);
  digitalWrite(leftIN3, LOW);
  digitalWrite(leftIN4, HIGH);
  ledcWrite(pwmChannelRight, dutyCycle / 2);
  ledcWrite(pwmChannelLeft, dutyCycle);
  Serial.println("➡️ Dịch phải");
  delay(timeDelay);
  stop();
}

void setSpeed(int speed) {
  if (speed >= 0 && speed <= 255) {
    dutyCycle = speed;
    ledcWrite(pwmChannelRight, dutyCycle);
    ledcWrite(pwmChannelLeft, dutyCycle);
    Serial.print("⚙️ Tốc độ: ");
    Serial.println(dutyCycle);
  }
}

void printMenu() {
  Serial.println("===== MENU XE MQTT =====");
  Serial.println("Lệnh: forward | backward | left | right | stop | move_left | move_right");
  Serial.println("========================");
}

// ====== WiFi / MQTT Setup ======
void setup_wifi() {
  WiFi.begin(ssid, password);
  Serial.print("Đang kết nối WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ Đã kết nối WiFi");
}

void callback(char* topic, byte* payload, unsigned int length) {
  String msg = "";
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }

  Serial.print("📩 Nhận từ topic [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(msg);

  if (String(topic) == "mpu6050/alert") {
    mqttCommand = msg;
  }
}

void reconnect() { 
  while (!client.connected()) {
    Serial.print("Đang kết nối MQTT...");
    if (client.connect("ESP32Clie++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++.+;;;;;;;;;;;;;;;nt", "hiep", "1234")) {
      Serial.println("✅ MQTT kết nối");
      client.subscribe("mpu6050/alert");
    } else {
      Serial.print("❌ Thất bại, mã lỗi: ");
      Serial.println(client.state());
      delay(2000);
    }
  }
}

// ====== Setup & Loop ======
void setup() {
  Serial.begin(9600);
  setup_wifi();

  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  pinMode(rightIN1, OUTPUT);
  pinMode(rightIN2, OUTPUT);
  pinMode(leftIN3, OUTPUT);
  pinMode(leftIN4, OUTPUT);
  pinMode(rightEnable, OUTPUT);
  pinMode(leftEnable, OUTPUT);

  ledcSetup(pwmChannelRight, freq, resolution);
  ledcAttachPin(rightEnable, pwmChannelRight);
  ledcSetup(pwmChannelLeft, freq, resolution);
  ledcAttachPin(leftEnable, pwmChannelLeft);

  stop();
  printMenu();
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  if (mqttCommand.length() > 0) {
    String cmd = mqttCommand;
    mqttCommand = "";  // clear để không bị lặp lại

    cmd.toLowerCase();

    if (cmd == "forward") moveForward();
    else if (cmd == "backward") moveBackward();
    else if (cmd == "left") turnLeft();
    else if (cmd == "right") turnRight();
    else if (cmd == "move_left") moveLeft();
    else if (cmd == "move_right") moveRight();
    else if (cmd == "stop") stop();
    else if (cmd == "help") printMenu();
    else {
      Serial.print("❓ Lệnh không hợp lệ: ");
      Serial.println(cmd);
    }

  }

  delay(10);
}
