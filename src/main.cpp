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
int dutyCycle = 200;

String mqttCommand = "";
String currentState = "stop"; // Biến lưu trạng thái hiện tại

// ====== Encoder setup ======
const int encoderPin = 18;  // Chân kết nối encoder
volatile long encoderCount = 0;
unsigned long lastPublishTime = 0;
const unsigned long publishInterval = 500; // Giảm xuống 500ms để cập nhật thường xuyên hơn
const int pulsesPerRotation = 20; // Điều chỉnh thông số này theo encoder của bạn

// ====== Auto Movement Control ======
bool autoModeEnabled = false;
int autoModeStep = 0;
long targetEncoderCount = 0;
long startEncoderCount = 0;
unsigned long stateChangeTime = 0;
const unsigned long pauseDuration = 1000; // Thời gian dừng giữa các bước (1 giây)
bool inPauseState = false;

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
  inPauseState = false;
  
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

// Kiểm tra và điều chỉnh quá trình tự động di chuyển - cơ chế không chặn
void checkAutoMode() {
  if (!autoModeEnabled) return;
  
  unsigned long currentTime = millis();
  long currentCount = encoderCount;
  
  // Kiểm tra xem có đang trong trạng thái dừng không
  if (inPauseState) {
    if (currentTime - stateChangeTime >= pauseDuration) {
      inPauseState = false;
      
      // Chuyển sang bước tiếp theo sau khi dừng xong
      if (autoModeStep == 1) {
        // Bắt đầu bước 2: quay trái 10 bước
        autoModeStep = 2;
        startEncoderCount = currentCount;
        targetEncoderCount = startEncoderCount + 10;
        turnLeft();
        
        Serial.println("🤖 Bắt đầu bước 2 - Quay trái 10 bước");
        client.publish("mpu6050/status", "Step 2: Turning left 10 steps");
      }
      else if (autoModeStep == 2) {
        // Bắt đầu bước 3: đi thẳng 20 bước
        autoModeStep = 3;
        startEncoderCount = currentCount;
        targetEncoderCount = startEncoderCount + 20;
        moveForward();
        
        Serial.println("🤖 Bắt đầu bước 3 - Đi thẳng 20 bước");
        client.publish("mpu6050/status", "Step 3: Moving forward 20 steps");
      }
    }
    // Nếu đang trong trạng thái dừng, không làm gì thêm
    return;
  }
  
  // Kiểm tra từng bước nếu không trong trạng thái dừng
  switch(autoModeStep) {
    case 1: // Đi thẳng 20 bước
      if (currentCount >= targetEncoderCount) {
        stop();
        stateChangeTime = currentTime;
        inPauseState = true;
        
        Serial.println("🤖 Hoàn thành bước 1 - Đi thẳng 20 bước");
        Serial.println("🤖 Đang dừng 1 giây");
      }
      break;
      
    case 2: // Quay trái 10 bước
      if (currentCount >= targetEncoderCount) {
        stop();
        stateChangeTime = currentTime;
        inPauseState = true;
        
        Serial.println("🤖 Hoàn thành bước 2 - Quay trái 10 bước");
        Serial.println("🤖 Đang dừng 1 giây");
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
        client.publish("mpu6050/status", "Auto mode completed");
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

// Hàm tính toán và gửi tổng số vòng quay - sửa đổi để đảm bảo gửi liên tục
void publishTotalRotations() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastPublishTime >= publishInterval) {
    // Lưu thời điểm gửi ngay tại đây
    lastPublishTime = currentTime;
    
    // Sao chép giá trị encoderCount để tránh thay đổi trong quá trình gửi
    long currentEncoderCount = encoderCount;
    
    // Chuyển thành chuỗi để gửi MQTT
    char rotationStr[10];
    dtostrf(currentEncoderCount, 2, 0, rotationStr);
    
    // In ra Serial trước khi gửi MQTT
    Serial.print("📊 Tổng số encoder: ");
    Serial.println(currentEncoderCount);
    
    // Gửi dữ liệu encoder ngay lập tức nếu đã kết nối
    if (client.connected()) {
      client.publish("mpu6050/vongquay", rotationStr);
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
  // Đảm bảo kết nối MQTT
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

  // Gửi tổng số vòng quay - được gọi thường xuyên trong loop
  publishTotalRotations();

}