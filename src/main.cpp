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
int leftIN3 = 32;  // Đã thay đổi từ 25 sang 32
int leftIN4 = 33;
int rightEnable = 14;
int leftEnable = 12;

const int freq = 1000;
const int pwmChannelRight = 0;
const int pwmChannelLeft = 1;
const int resolution = 8;
int dutyCycle = 180;

String mqttCommand = "";
String currentState = "stop"; // Biến lưu trạng thái hiện tại

// ====== Encoder setup ======
const int encoderPin = 18;  // Chân kết nối encoder
volatile int encoderCount = 0;
unsigned int lastPublishTime = 0;
// const unsigned long publishInterval = 1000; // 1 giây
const int pulsesPerRotation = 20; // Điều chỉnh thông số này theo encoder của bạn

// ====== Auto Movement Control ======
bool autoModeEnabled = false;
int autoModeStep = 0;
int targetEncoderCount = 0;
int startEncoderCount = 0;

// Hàm ngắt đếm xung encoder
void IRAM_ATTR encoderISR() {
  encoderCount++;
}

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
}

void moveBackward() {
  digitalWrite(rightIN1, HIGH);
  digitalWrite(rightIN2, LOW);
  digitalWrite(leftIN3, HIGH);
  digitalWrite(leftIN4, LOW);
  ledcWrite(pwmChannelRight, dutyCycle);
  ledcWrite(pwmChannelLeft, dutyCycle);
  Serial.println("⬅️ Lùi");
}

void turnLeft() {
  digitalWrite(rightIN1, LOW);
  digitalWrite(rightIN2, HIGH);
  digitalWrite(leftIN3, HIGH);
  digitalWrite(leftIN4, LOW);
  ledcWrite(pwmChannelRight, dutyCycle);
  ledcWrite(pwmChannelLeft, dutyCycle);
  Serial.println("↪️ Trái");
}

void turnRight() {
  digitalWrite(rightIN1, HIGH);
  digitalWrite(rightIN2, LOW);
  digitalWrite(leftIN3, LOW);
  digitalWrite(leftIN4, HIGH);
  ledcWrite(pwmChannelRight, dutyCycle);
  ledcWrite(pwmChannelLeft, dutyCycle);
  Serial.println("↩️ Phải");
}

void moveLeft() {
  digitalWrite(rightIN1, LOW);
  digitalWrite(rightIN2, HIGH);
  digitalWrite(leftIN3, LOW);
  digitalWrite(leftIN4, HIGH);
  ledcWrite(pwmChannelRight, dutyCycle);
  ledcWrite(pwmChannelLeft, dutyCycle / 2);
  Serial.println("⬅️ Dịch trái");
}

void moveRight() {
  digitalWrite(rightIN1, LOW);
  digitalWrite(rightIN2, HIGH);
  digitalWrite(leftIN3, LOW);
  digitalWrite(leftIN4, HIGH);
  ledcWrite(pwmChannelRight, dutyCycle / 2);
  ledcWrite(pwmChannelLeft, dutyCycle);
  Serial.println("➡️ Dịch phải");
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

// Bắt đầu chế độ tự hành
void startAutoMode() {
  autoModeEnabled = true;
  autoModeStep = 0;
  
  Serial.println("🤖 Chế độ tự hành được kích hoạt");
  
  // Bắt đầu bước đầu tiên: đi thẳng 20 bước
  autoModeStep = 1;
  startEncoderCount = encoderCount;
  targetEncoderCount = startEncoderCount + 20;
  moveForward();
  currentState = "auto_mode";
  
  // Gửi thông báo qua MQTT
  client.publish("mpu6050/status", "Auto mode activated - Step 1: Moving forward 20 steps");
}

// Kiểm tra và điều chỉnh quá trình tự động di chuyển
void checkAutoMode() {
  if (!autoModeEnabled) return;
  
  int currentCount = encoderCount;
  
  switch(autoModeStep) {
    case 1: // Đi thẳng 20 bước
      if (currentCount >= targetEncoderCount) {
        stop();
        
        // Chuyển sang bước tiếp: quay trái 10 bước
        autoModeStep = 2;
        startEncoderCount = currentCount;
        targetEncoderCount = startEncoderCount + 17;
        turnLeft();
        
        Serial.println("🤖 Hoàn thành bước 1 - Đi thẳng 20 bước");
        Serial.println("🤖 Bắt đầu bước 2 - Quay trái 10 bước");
        
        // Thêm thông tin encoder vào thông báo
        char statusMsg[50];
        sprintf(statusMsg, "Step 2: Turning left 10 steps (Encoder: %ld)", encoderCount);
        client.publish("mpu6050/status", statusMsg);
      }
      break;
      
    case 2: // Quay trái 10 bước
      if (currentCount >= targetEncoderCount) {
        stop();        
        // Chuyển sang bước tiếp: đi thẳng 20 bước
        autoModeStep = 3;
        startEncoderCount = currentCount;
        targetEncoderCount = startEncoderCount + 20;
        moveForward();
        
        Serial.println("🤖 Hoàn thành bước 2 - Quay trái 10 bước");
        Serial.println("🤖 Bắt đầu bước 3 - Đi thẳng 20 bước");
        
        // Thêm thông tin encoder vào thông báo
        char statusMsg[50];
        sprintf(statusMsg, "Step 3: Moving forward 20 steps (Encoder: %ld)", encoderCount);
        client.publish("mpu6050/status", statusMsg);
      }
      break;
      
    case 3: // Đi thẳng 20 bước
      if (currentCount >= targetEncoderCount) {
        stop();
        
        // Kết thúc chế độ tự hành
        autoModeEnabled = false;
        currentState = "stop";
        
        Serial.println("🤖 Hoàn thành bước 3 - Đi thẳng 20 bước");
        Serial.println("🤖 Chế độ tự hành hoàn tất");
        
        // Thêm thông tin encoder vào thông báo
        char statusMsg[50];
        sprintf(statusMsg, "Auto mode completed (Final Encoder: %ld)", encoderCount);
        client.publish("mpu6050/status", statusMsg);
      }
      break;
  }
}

void printMenu() {
  Serial.println("===== MENU XE MQTT =====");
  Serial.println("Lệnh: forward | backward | left | right | stop | move_left | move_right | auto");
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
    if (client.connect("ESP32Client", "hiep", "1234")) {
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

  // Thiết lập encoder pin và ngắt
  pinMode(encoderPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPin), encoderISR, RISING);

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

  // Xử lý lệnh điều khiển
  if (mqttCommand.length() > 0) {
    String cmd = mqttCommand;
    mqttCommand = "";  // clear để không bị lặp lại

    cmd.toLowerCase();
    
    // Xử lý lệnh auto mode đặc biệt
    if (cmd == "auto") {
      startAutoMode();
    }
    // Chỉ xử lý các lệnh khác nếu không ở trong chế độ tự động
    else if (!autoModeEnabled && cmd != currentState) {
      currentState = cmd; // Cập nhật trạng thái hiện tại
      
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
        currentState = ""; // Reset state for invalid commands
      }
    } else if (!autoModeEnabled) {
      Serial.println("🔄 Giữ nguyên trạng thái: " + currentState);
    } else {
      Serial.println("🤖 Đang ở chế độ tự hành, bỏ qua lệnh: " + cmd);
    }
  }

  // Kiểm tra và cập nhật chế độ tự hành nếu được kích hoạt
  checkAutoMode();


}