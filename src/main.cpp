#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <queue>
#include <vector>

// ====== WiFi & MQTT config ======
const char* ssid = "KawaII";
const char* password = "dmcayvcl";
const char* mqtt_server = "172.20.10.2";

WiFiClient espClient;
PubSubClient client(espClient);

// ====== L298N setup ======
int rightIN1 = 27;
int rightIN2 = 26;
int leftIN3 = 32;
int leftIN4 = 33;
int rightEnable = 14;
int leftEnable = 12;

const int freq = 1000;
const int pwmChannelRight = 0;
const int pwmChannelLeft = 1;
const int resolution = 8;
int dutyCycle = 180;

String mqttCommand = "";
String currentState = "stop";

// ====== Encoder setup ======
const int encoderPin = 18;
volatile int encoderCount = 0;
unsigned int lastPublishTime = 0;
const int pulsesPerRotation = 20;

// ====== Map & Navigation Constants ======
const int MAP_SIZE = 10;
const int ENCODER_STEPS_PER_CELL = 30;
const float ENCODER_STEPS_PER_TURN = 17;  // Estimated from your code

// Add these global variables to your code
bool isDrawingShape = false;
String shapeCommand = "";
int shapeSteps = 0;
int shapeTotalSteps = 0;
int startDrawingCount = 0;

// Enum for shape drawing states
enum ShapeState {
  SHAPE_IDLE,
  SHAPE_MOVING,
  SHAPE_TURNING,
  SHAPE_FINISHED
};

ShapeState shapeState = SHAPE_IDLE;

// Directions
enum Direction { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3 };

// Map representation
int mapGrid[MAP_SIZE][MAP_SIZE] = {0};  // 0 = empty, 1 = robot position, 2 = destination

// Current robot position and orientation
int robotX = 0;
int robotY = 0;
Direction robotDir = NORTH;  // Assuming robot starts facing north

// Current turn tracking
String lastTurnDirection = "";

// Path planning
struct Node {
  int x;
  int y;
  Direction dir;    // Hướng của robot khi đến nút này
  int cost;         // Chi phí đã đi được từ điểm bắt đầu
  int turnCost;     // Chi phí xoay
  int heuristic;    // Ước lượng chi phí đến đích
  int totalCost;    // cost + turnCost + heuristic
  int parentX;
  int parentY;
  Direction parentDir; // Hướng của robot ở nút cha
  
  bool operator>(const Node& other) const {
      return totalCost > other.totalCost;
  }
};

// Thêm hằng số mới để biểu diễn vật cản trên bản đồ
const int EMPTY = 0;       // Ô trống
const int ROBOT = 1;       // Vị trí robot
const int GOAL = 2;        // Điểm đích
const int OBSTACLE = 3;    // Vật cản - không thể đi qua

// Hàm để thêm chướng ngại vật vào bản đồ
void addObstacle(int x, int y) {
    if (x >= 0 && x < MAP_SIZE && y >= 0 && y < MAP_SIZE) {
        if (mapGrid[y][x] != ROBOT && mapGrid[y][x] != GOAL) {
            mapGrid[y][x] = OBSTACLE;
            Serial.print("🚧 Đã thêm chướng ngại vật tại (");
            Serial.print(x);
            Serial.print(",");
            Serial.print(y);
            Serial.println(")");
            
            String obstacleMsg = "Obstacle added at (" + String(x) + "," + String(y) + ")";
            client.publish("robot/map", obstacleMsg.c_str());
        }
    }
}


// Hằng số chi phí cho mỗi lần xoay
const int TURN_COST = 3;    // Chi phí mỗi lần xoay 90 độ
const int MOVE_COST = 1;    // Chi phí di chuyển

// Hàm tính chi phí xoay từ hướng hiện tại sang hướng mới
int calculateTurnCost(Direction currentDir, Direction newDir) {
  if (currentDir == newDir) return 0;  // Không phải xoay
  
  int diff = abs(currentDir - newDir);
  if (diff == 2) return TURN_COST * 2;  // Xoay 180 độ
  return TURN_COST;  // Xoay 90 độ (diff = 1 hoặc 3)
}

std::vector<std::pair<int, int>> path;
int currentPathIndex = 0;
bool pathFound = false;
bool navigationActive = false;

// Function declarations
void IRAM_ATTR encoderISR();
void stop();
void moveForward();
void moveBackward();
void turnLeft();
void turnRight();
void moveLeft();
void moveRight();
void setup_wifi();
void callback(char* topic, byte* payload, unsigned int length);
void reconnect();
void printMenu();
void setSpeed(int speed);
void initializeMap();
void findPath();
void executeNextPathStep();
void navigateToNextCell();
void debugDirection(Direction dir);

// ====== Robot Movement Functions ======
void IRAM_ATTR encoderISR() {
    encoderCount++;
}

// ====== Navigation State Machine ======
enum NavigationState {
  IDLE,
  TURNING,
  TURN_PAUSE,
  MOVING,
  ARRIVED
};
unsigned long turnPauseStartTime = 0;
const unsigned long TURN_PAUSE_DURATION = 500;

NavigationState navState = IDLE;
int targetEncoderCount = 0;
int startEncoderCount = 0;
bool needSecondTurn = false;

void stop() {
    digitalWrite(rightIN1, LOW);
    digitalWrite(rightIN2, LOW);
    digitalWrite(leftIN3, LOW);
    digitalWrite(leftIN4, LOW);
    Serial.println("⛔ Đã dừng");
    client.publish("robot/movement", "Stopped");
    currentState = "stop";
}

void moveForward() {
    digitalWrite(rightIN1, LOW);
    digitalWrite(rightIN2, HIGH);
    digitalWrite(leftIN3, LOW);
    digitalWrite(leftIN4, HIGH);
    ledcWrite(pwmChannelRight, dutyCycle);
    ledcWrite(pwmChannelLeft, dutyCycle);
    Serial.println("➡️ Tiến");
    client.publish("robot/movement", "Moving forward");
    currentState = "forward";
}

void moveBackward() {
    digitalWrite(rightIN1, HIGH);
    digitalWrite(rightIN2, LOW);
    digitalWrite(leftIN3, HIGH);
    digitalWrite(leftIN4, LOW);
    ledcWrite(pwmChannelRight, dutyCycle);
    ledcWrite(pwmChannelLeft, dutyCycle);
    Serial.println("⬅️ Lùi");
    client.publish("robot/movement", "Moving backward");
    currentState = "backward";
}

void turnLeft() {
    digitalWrite(rightIN1, LOW);
    digitalWrite(rightIN2, HIGH);
    digitalWrite(leftIN3, HIGH);
    digitalWrite(leftIN4, LOW); 
    ledcWrite(pwmChannelRight, dutyCycle);
    ledcWrite(pwmChannelLeft, dutyCycle);
    Serial.println("↪️ Trái");
    client.publish("robot/movement", "Turning left");
    currentState = "left";
    lastTurnDirection = "left";
}

void turnRight() {
    digitalWrite(rightIN1, HIGH);
    digitalWrite(rightIN2, LOW);
    digitalWrite(leftIN3, LOW);
    digitalWrite(leftIN4, HIGH);
    ledcWrite(pwmChannelRight, dutyCycle);
    ledcWrite(pwmChannelLeft, dutyCycle);
    Serial.println("↩️ Phải");
    client.publish("robot/movement", "Turning right");
    currentState = "right";
    lastTurnDirection = "right";
}

void moveLeft() {
    digitalWrite(rightIN1, LOW);
    digitalWrite(rightIN2, HIGH);
    digitalWrite(leftIN3, LOW);
    digitalWrite(leftIN4, HIGH);
    ledcWrite(pwmChannelRight, dutyCycle);
    ledcWrite(pwmChannelLeft, dutyCycle / 2);
    Serial.println("⬅️ Dịch trái");
    client.publish("robot/movement", "Moving left");
    currentState = "move_left";
}

void moveRight() {
    digitalWrite(rightIN1, LOW);
    digitalWrite(rightIN2, HIGH);
    digitalWrite(leftIN3, LOW);
    digitalWrite(leftIN4, HIGH);
    ledcWrite(pwmChannelRight, 100);
    ledcWrite(pwmChannelLeft, 120);
    Serial.println("➡️ Dịch phải");
    client.publish("robot/movement", "Moving right");
    currentState = "move_right";
}

void setSpeed(int speed) {
    if (speed >= 0 && speed <= 255) {
        dutyCycle = speed;
        ledcWrite(pwmChannelRight, dutyCycle);
        ledcWrite(pwmChannelLeft, dutyCycle);
        Serial.print("⚙️ Tốc độ: ");
        Serial.println(dutyCycle);
        
        // Gửi thông tin tốc độ qua MQTT
        String speedMsg = "Speed set to: " + String(dutyCycle);
        client.publish("robot/speed", speedMsg.c_str());
    }
}

// Helper function to debug direction
void debugDirection(Direction dir) {
    switch(dir) {
        case NORTH: Serial.print("BẮC"); break;
        case EAST: Serial.print("ĐÔNG"); break;
        case SOUTH: Serial.print("NAM"); break;
        case WEST: Serial.print("TÂY"); break;
    }
}

// ====== WiFi & MQTT Functions ======
void setup_wifi() {
    WiFi.begin(ssid, password);
    Serial.print("Đang kết nối WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\n✅ Đã kết nối WiFi");
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

void printMenu() {
  Serial.println("===== MENU XE MQTT =====");
  Serial.println("Lệnh: forward | backward | left | right | stop | move_left | move_right | navigate");
  Serial.println("Lệnh navigate có thể chỉ định vị trí xuất phát và đích: navigate:startX,startY:goalX,goalY");
  Serial.println("Hoặc với chướng ngại vật: navigate:startX,startY:goalX,goalY:obstX1,obstY1;obstX2,obstY2;...");
  Serial.println("Ví dụ: navigate:0,0:9,9 hoặc navigate:0,0:9,9:3,4;5,2");
  Serial.println("========================");
  
  // Gửi menu lệnh qua MQTT
  client.publish("robot/help", "Commands: forward | backward | left | right | stop | move_left | move_right | navigate");
  client.publish("robot/help", "For custom navigation: navigate:startX,startY:goalX,goalY (e.g., navigate:0,0:9,9)");
  client.publish("robot/help", "With obstacles: navigate:startX,startY:goalX,goalY:obstX1,obstY1;obstX2,obstY2;...");
}

// ====== Map and Pathfinding Functions ======
void initializeMap() {
    // Clear the map
    for (int y = 0; y < MAP_SIZE; y++) {
        for (int x = 0; x < MAP_SIZE; x++) {
            mapGrid[y][x] = 0;  // Empty space
        }
    }
    
    // Set robot position and destination
    mapGrid[0][0] = 1;          // Robot starting position
    mapGrid[MAP_SIZE-1][MAP_SIZE-1] = 2;  // Destination
    
    robotX = 0;
    robotY = 0;
    robotDir = NORTH;
    
    Serial.println("🗺️ Bản đồ đã được khởi tạo");
    Serial.println("🤖 Robot ở vị trí (0,0)");
    Serial.println("🎯 Đích ở vị trí (9,9)");
    
    // Gửi trạng thái qua MQTT
    client.publish("robot/status", "Map initialized. Robot at (0,0), destination at (9,9)");
}

// Manhattan distance heuristic
int calculateHeuristic(int x, int y, int goalX, int goalY) {
    return abs(x - goalX) + abs(y - goalY);
}

void findPath() {
  Serial.println("🔍 Đang tìm đường đi với tối ưu xoay và tránh chướng ngại vật...");
  client.publish("robot/status", "Finding path with turn optimization and obstacle avoidance...");
  
  // Tìm tọa độ điểm đích từ giá trị 2 trong bản đồ
  int goalX = -1;
  int goalY = -1;
  
  for (int y = 0; y < MAP_SIZE; y++) {
      for (int x = 0; x < MAP_SIZE; x++) {
          if (mapGrid[y][x] == GOAL) {
              goalX = x;
              goalY = y;
              break;
          }
      }
      if (goalX != -1) break;
  }
  
  // Nếu không tìm thấy điểm đích, sử dụng mặc định (MAP_SIZE-1, MAP_SIZE-1)
  if (goalX == -1 || goalY == -1) {
      goalX = MAP_SIZE - 1;
      goalY = MAP_SIZE - 1;
      Serial.println("⚠️ Không tìm thấy đích trên bản đồ, sử dụng vị trí mặc định (9,9)");
  }
  
  // In ra thông tin hiện tại của robot
  Serial.print("🤖 Vị trí xuất phát: (");
  Serial.print(robotX);
  Serial.print(",");
  Serial.print(robotY);
  Serial.print("), Hướng hiện tại: ");
  debugDirection(robotDir);
  Serial.println();
  
  // In thông tin về vật cản trên bản đồ
  Serial.println("🚧 Các vật cản trên bản đồ:");
  for (int y = 0; y < MAP_SIZE; y++) {
      for (int x = 0; x < MAP_SIZE; x++) {
          if (mapGrid[y][x] == OBSTACLE) {
              Serial.print("  - (");
              Serial.print(x);
              Serial.print(",");
              Serial.print(y);
              Serial.println(")");
          }
      }
  }
  
  // Xóa đường đi trước đó
  path.clear();
  
  // Tạo danh sách đóng (các nút đã thăm)
  bool closed[MAP_SIZE][MAP_SIZE][4] = {false};  // x, y, direction
  
  // Tạo danh sách mở (hàng đợi ưu tiên)
  std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openList;
  
  // Tạo bản đồ nút cha
  int parentX[MAP_SIZE][MAP_SIZE][4];
  int parentY[MAP_SIZE][MAP_SIZE][4];
  Direction parentDir[MAP_SIZE][MAP_SIZE][4];
  
  // Khởi tạo các giá trị mặc định
  for (int z = 0; z < 4; z++) {
      for (int y = 0; y < MAP_SIZE; y++) {
          for (int x = 0; x < MAP_SIZE; x++) {
              parentX[y][x][z] = -1;
              parentY[y][x][z] = -1;
              parentDir[y][x][z] = robotDir;
          }
      }
  }
  
  // Thêm vị trí xuất phát vào danh sách mở
  Node startNode;
  startNode.x = robotX;
  startNode.y = robotY;
  startNode.dir = robotDir;
  startNode.cost = 0;
  startNode.turnCost = 0;
  startNode.heuristic = calculateHeuristic(robotX, robotY, goalX, goalY);
  startNode.totalCost = startNode.cost + startNode.turnCost + startNode.heuristic;
  startNode.parentX = -1;
  startNode.parentY = -1;
  startNode.parentDir = robotDir;
  
  openList.push(startNode);
  
  // Thuật toán A*
  while (!openList.empty()) {
      // Lấy nút có tổng chi phí thấp nhất
      Node current = openList.top();
      openList.pop();
      
      int x = current.x;
      int y = current.y;
      Direction dir = current.dir;
      
      // Bỏ qua nếu đã thăm
      if (closed[y][x][dir]) {
          continue;
      }
      
      // Đánh dấu là đã thăm
      closed[y][x][dir] = true;
      parentX[y][x][dir] = current.parentX;
      parentY[y][x][dir] = current.parentY;
      parentDir[y][x][dir] = current.parentDir;
      
      // Nếu đến đích, xây dựng đường đi
      if (x == goalX && y == goalY) {
          // Tạo đường đi từ đích về điểm xuất phát
          std::vector<std::pair<int, int>> reversePath;
          int currX = goalX;
          int currY = goalY;
          Direction currDir = dir;
          
          while (currX != robotX || currY != robotY) {
              reversePath.push_back(std::make_pair(currX, currY));
              int tempX = parentX[currY][currX][currDir];
              int tempY = parentY[currY][currX][currDir];
              Direction tempDir = parentDir[currY][currX][currDir];
              currX = tempX;
              currY = tempY;
              currDir = tempDir;
          }
          
          // Đảo ngược để có đường đi từ xuất phát đến đích
          for (int i = reversePath.size() - 1; i >= 0; i--) {
              path.push_back(reversePath[i]);
          }
          
          Serial.print("✅ Đường đi đã được tìm thấy! Số bước: ");
          Serial.println(path.size());
          
          // Chuẩn bị thông báo MQTT
          String pathMsg = "Path found with " + String(path.size()) + " steps: (";
          pathMsg += robotX;
          pathMsg += ",";
          pathMsg += robotY;
          pathMsg += ") -> ";
          
          for (size_t i = 0; i < path.size(); i++) {
              pathMsg += "(";
              pathMsg += path[i].first;
              pathMsg += ",";
              pathMsg += path[i].second;
              pathMsg += ")";
              
              if (i < path.size() - 1) {
                  pathMsg += " -> ";
              }
          }
          
          // Gửi đường đi qua MQTT
          client.publish("robot/path", pathMsg.c_str());
          
          // In đường đi
          Serial.println("🛣️ Đường đi: ");
          Serial.println(pathMsg);
          
          pathFound = true;
          return;
      }
      
      // Khám phá 4 hướng lân cận (lên, phải, xuống, trái)
      int dx[4] = {0, 1, 0, -1};
      int dy[4] = {-1, 0, 1, 0};
      
      for (int i = 0; i < 4; i++) {
          Direction newDir = (Direction)i;  // Hướng mới khi đi theo i
          int newX = x + dx[i];
          int newY = y + dy[i];
          
          // Kiểm tra vị trí hợp lệ và không phải vật cản
          if (newX >= 0 && newX < MAP_SIZE && 
              newY >= 0 && newY < MAP_SIZE && 
              mapGrid[newY][newX] != OBSTACLE && 
              !closed[newY][newX][newDir]) {
              
              // Tính chi phí xoay
              int turn = calculateTurnCost(dir, newDir);
              
              Node neighbor;
              neighbor.x = newX;
              neighbor.y = newY;
              neighbor.dir = newDir;
              neighbor.cost = current.cost + MOVE_COST;  // Mỗi bước di chuyển tốn 1
              neighbor.turnCost = current.turnCost + turn;
              neighbor.heuristic = calculateHeuristic(newX, newY, goalX, goalY);
              neighbor.totalCost = neighbor.cost + neighbor.turnCost + neighbor.heuristic;
              neighbor.parentX = x;
              neighbor.parentY = y;
              neighbor.parentDir = dir;
              
              openList.push(neighbor);
          }
      }
  }
  
  Serial.println("❌ Không tìm thấy đường đi!");
  Serial.println("⚠️ Có thể đích bị bao quanh bởi vật cản hoặc không có đường đi tới đích.");
  client.publish("robot/status", "Path not found! Goal may be surrounded by obstacles.");
  pathFound = false;
}

// Mở rộng hàm xử lý lệnh navigate để hỗ trợ thêm vật cản
void processNavigateCommand(String cmd) {
  // Kiểm tra xem có phải định dạng "navigate:startX,startY:goalX,goalY:obstX1,obstY1;obstX2,obstY2;..."
  int firstColon = cmd.indexOf(":");
  if (firstColon <= 0) {
      // Lệnh navigate đơn giản, sử dụng vị trí hiện tại đến 9,9
      findPath();
      if (pathFound) {
          navigationActive = true;
          currentPathIndex = 0;
          navState = IDLE;
          needSecondTurn = false;
          Serial.println("🚀 Bắt đầu điều hướng tự động");
          client.publish("robot/status", "Navigation started");
      }
      return;
  }

  // Phân tích lệnh phức tạp hơn với tọa độ
  int secondColon = cmd.indexOf(":", firstColon + 1);
  if (secondColon <= 0) {
      Serial.println("❌ Định dạng lệnh không hợp lệ!");
      client.publish("robot/status", "Invalid command format!");
      return;
  }

  // Lấy tọa độ xuất phát
  String startCoords = cmd.substring(firstColon + 1, secondColon);
  int startComma = startCoords.indexOf(",");
  if (startComma <= 0) {
      Serial.println("❌ Định dạng tọa độ xuất phát không hợp lệ!");
      client.publish("robot/status", "Invalid start coordinates format!");
      return;
  }
  
  int startX = startCoords.substring(0, startComma).toInt();
  int startY = startCoords.substring(startComma + 1).toInt();
  
  // Kiểm tra xem có dấu hai chấm thứ ba để chỉ định vật cản
  int thirdColon = cmd.indexOf(":", secondColon + 1);
  String goalCoords;
  
  if (thirdColon <= 0) {
      // Không có chỉ định vật cản
      goalCoords = cmd.substring(secondColon + 1);
  } else {
      // Có chỉ định vật cản
      goalCoords = cmd.substring(secondColon + 1, thirdColon);
  }
  
  int goalComma = goalCoords.indexOf(",");
  if (goalComma <= 0) {
      Serial.println("❌ Định dạng tọa độ đích không hợp lệ!");
      client.publish("robot/status", "Invalid goal coordinates format!");
      return;
  }
  
  int goalX = goalCoords.substring(0, goalComma).toInt();
  int goalY = goalCoords.substring(goalComma + 1).toInt();
  
  // Kiểm tra tọa độ hợp lệ
  if (startX < 0 || startX >= MAP_SIZE || startY < 0 || startY >= MAP_SIZE ||
      goalX < 0 || goalX >= MAP_SIZE || goalY < 0 || goalY >= MAP_SIZE) {
      Serial.println("❌ Tọa độ không hợp lệ!");
      client.publish("robot/status", "Invalid coordinates!");
      return;
  }
  
  // Cập nhật vị trí robot và xóa bản đồ
  robotX = startX;
  robotY = startY;
  
  // Xóa bản đồ
  for (int y = 0; y < MAP_SIZE; y++) {
      for (int x = 0; x < MAP_SIZE; x++) {
          mapGrid[y][x] = EMPTY;  // Ô trống
      }
  }
  
  // Thiết lập vị trí robot và đích
  mapGrid[startY][startX] = ROBOT;
  mapGrid[goalY][goalX] = GOAL;
  
  Serial.print("🚩 Đặt điểm xuất phát: (");
  Serial.print(startX);
  Serial.print(",");
  Serial.print(startY);
  Serial.println(")");
  
  Serial.print("🎯 Đặt điểm đích: (");
  Serial.print(goalX);
  Serial.print(",");
  Serial.print(goalY);
  Serial.println(")");
  
  // Xử lý vật cản nếu có
  if (thirdColon > 0) {
      String obstaclesStr = cmd.substring(thirdColon + 1);
      int currentPos = 0;
      int nextPos = 0;
      
      Serial.println("🚧 Thêm vật cản vào bản đồ:");
      
      // Phân tích các vật cản (định dạng: obsX1,obsY1;obsX2,obsY2;...)
      while (currentPos < obstaclesStr.length()) {
          nextPos = obstaclesStr.indexOf(";", currentPos);
          if (nextPos == -1) {
              nextPos = obstaclesStr.length();
          }
          
          String obstacle = obstaclesStr.substring(currentPos, nextPos);
          int obstComma = obstacle.indexOf(",");
          
          if (obstComma > 0) {
              int obstX = obstacle.substring(0, obstComma).toInt();
              int obstY = obstacle.substring(obstComma + 1).toInt();
              
              if (obstX >= 0 && obstX < MAP_SIZE && obstY >= 0 && obstY < MAP_SIZE) {
                  // Kiểm tra xem vật cản không phải ở vị trí robot hoặc đích
                  if ((obstX != startX || obstY != startY) && (obstX != goalX || obstY != goalY)) {
                      mapGrid[obstY][obstX] = OBSTACLE;
                      Serial.print("  - (");
                      Serial.print(obstX);
                      Serial.print(",");
                      Serial.print(obstY);
                      Serial.println(")");
                  }
              }
          }
          
          currentPos = nextPos + 1;
      }
  }
  
  client.publish("robot/status", "Custom start, goal and obstacles set");
  
  // Tìm đường đi với các vị trí mới
  findPath();
  if (pathFound) {
      navigationActive = true;
      currentPathIndex = 0;
      navState = IDLE;
      needSecondTurn = false;
      Serial.println("🚀 Bắt đầu điều hướng tự động");
      client.publish("robot/status", "Navigation started");
  }
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
        // Check for shape commands
        if (msg == "CIRCLE") {
            // Stop any ongoing movement
            stop();
            navigationActive = false;
            
            // Start circle drawing
            isDrawingShape = true;
            shapeCommand = "CIRCLE";
            startDrawingCount = encoderCount;
            targetEncoderCount = startDrawingCount + 74; // 74 encoder counts for circle
            shapeState = SHAPE_MOVING;
            
            // Move right to draw circle
            turnRight();
            Serial.println("🔵 Drawing Circle");
            client.publish("robot/status", "Drawing Circle");
        }
        else if (msg == "RECTANGLE") {
            // Stop any ongoing movement
            stop();
            navigationActive = false;
            
            // Start rectangle drawing
            isDrawingShape = true;
            shapeCommand = "RECTANGLE";
            shapeSteps = 0;
            shapeTotalSteps = 8; // 4 movements + 4 turns
            startDrawingCount = encoderCount;
            targetEncoderCount = startDrawingCount + 30; // 30 encoder counts for each side
            shapeState = SHAPE_MOVING;
            
            // Start with forward movement
            moveForward();
            Serial.println("🟦 Drawing Rectangle - Side 1");
            client.publish("robot/status", "Drawing Rectangle - Side 1");
        }
        else {
            // Handle other MQTT commands as before
            mqttCommand = msg;
        }
    }
  }
  
  

Direction getTargetDirection(int currentX, int currentY, int targetX, int targetY) {
    if (targetX > currentX) return EAST;
    if (targetX < currentX) return WEST;
    if (targetY > currentY) return SOUTH;
    if (targetY < currentY) return NORTH;
    return robotDir; // No change if same position
}

void turnToDirection(Direction targetDir) {
    // Calculate turns needed (shortest path)
    int turns = (targetDir - robotDir + 4) % 4;
    
    // Optimize to at most 2 turns
    if (turns > 2) turns -= 4;
    
    Serial.print("🧭 Cần quay: ");
    Serial.print(turns);
    Serial.print(" bước (Hiện tại: ");
    debugDirection(robotDir);
    Serial.print(", Mục tiêu: ");
    debugDirection(targetDir);
    Serial.println(")");
    
    // Gửi thông tin quay qua MQTT
    String turnMsg = "Turn info: Current=";
    switch(robotDir) {
        case NORTH: turnMsg += "NORTH"; break;
        case EAST: turnMsg += "EAST"; break;
        case SOUTH: turnMsg += "SOUTH"; break;
        case WEST: turnMsg += "WEST"; break;
    }
    turnMsg += ", Target=";
    switch(targetDir) {
        case NORTH: turnMsg += "NORTH"; break;
        case EAST: turnMsg += "EAST"; break;
        case SOUTH: turnMsg += "SOUTH"; break;
        case WEST: turnMsg += "WEST"; break;
    }
    turnMsg += ", Steps=" + String(turns);
    client.publish("robot/direction_info", turnMsg.c_str());
    
    if (turns == 0) {
        // Already facing the right direction
        navState = MOVING;
        startEncoderCount = encoderCount;
        targetEncoderCount = startEncoderCount + ENCODER_STEPS_PER_CELL;
        moveForward();
        Serial.println("➡️ Đã đúng hướng, tiến về phía trước");
        client.publish("robot/movement", "Moving forward");
        needSecondTurn = false;
    } else if (turns == 1 || turns == -3) {
        // Turn right
        navState = TURNING;
        startEncoderCount = encoderCount;
        targetEncoderCount = startEncoderCount + ENCODER_STEPS_PER_TURN;
        turnRight();
        Serial.println("↩️ Quay phải để đổi hướng");
        client.publish("robot/movement", "Turning right");
        needSecondTurn = false;
    } else if (turns == -1 || turns == 3) {
        // Turn left
        navState = TURNING;
        startEncoderCount = encoderCount;
        targetEncoderCount = startEncoderCount + ENCODER_STEPS_PER_TURN;
        turnLeft();
        Serial.println("↪️ Quay trái để đổi hướng");
        client.publish("robot/movement", "Turning left");
        needSecondTurn = false;
    } else if (turns == 2 || turns == -2) {
        // Turn around (two right turns)
        navState = TURNING;
        startEncoderCount = encoderCount;
        targetEncoderCount = startEncoderCount + ENCODER_STEPS_PER_TURN;
        turnRight();
        Serial.println("↩️↩️ Quay ngược lại (bước 1/2)");
        client.publish("robot/movement", "Turning around (step 1/2)");
        needSecondTurn = true;
    }
}

void navigateToNextCell() {
  if (currentPathIndex >= path.size()) {
      // Đã đến cuối đường đi
      stop();
      navigationActive = false;
      navState = ARRIVED;
      Serial.println("🎯 Đã đến đích!");
      client.publish("robot/status", "Destination reached! Mission completed!");
      return;
  }
  
  // Lấy ô tiếp theo
  int targetX = path[currentPathIndex].first;
  int targetY = path[currentPathIndex].second;
  
  switch (navState) {
      case IDLE:
          // Xác định hướng cần quay
          {
              Direction targetDir = getTargetDirection(robotX, robotY, targetX, targetY);
              
              // Tính số lần quay cần thiết (đường ngắn nhất)
              int turns = (targetDir - robotDir + 4) % 4;
              
              // Tối ưu hóa để tối đa 2 lần quay
              if (turns > 2) turns -= 4;
              
              Serial.print("🧭 Cần quay: ");
              Serial.print(turns);
              Serial.print(" bước (Hiện tại: ");
              debugDirection(robotDir);
              Serial.print(", Mục tiêu: ");
              debugDirection(targetDir);
              Serial.println(")");
              
              // Gửi thông tin quay qua MQTT
              String turnMsg = "Turn info: Current=";
              switch(robotDir) {
                  case NORTH: turnMsg += "NORTH"; break;
                  case EAST: turnMsg += "EAST"; break;
                  case SOUTH: turnMsg += "SOUTH"; break;
                  case WEST: turnMsg += "WEST"; break;
              }
              turnMsg += ", Target=";
              switch(targetDir) {
                  case NORTH: turnMsg += "NORTH"; break;
                  case EAST: turnMsg += "EAST"; break;
                  case SOUTH: turnMsg += "SOUTH"; break;
                  case WEST: turnMsg += "WEST"; break;
              }
              turnMsg += ", Steps=" + String(turns);
              client.publish("robot/direction_info", turnMsg.c_str());
              
              if (turns == 0) {
                  // Đã đúng hướng, di chuyển tiếp
                  navState = MOVING;
                  startEncoderCount = encoderCount;
                  targetEncoderCount = startEncoderCount + ENCODER_STEPS_PER_CELL;
                  moveForward();
                  Serial.println("➡️ Đã đúng hướng, tiến về phía trước");
                  client.publish("robot/movement", "Moving forward");
                  needSecondTurn = false;
              } else if (turns == 1 || turns == -3) {
                  // Quay phải
                  navState = TURNING;
                  startEncoderCount = encoderCount;
                  targetEncoderCount = startEncoderCount + ENCODER_STEPS_PER_TURN;
                  turnRight();
                  Serial.println("↩️ Quay phải để đổi hướng");
                  client.publish("robot/movement", "Turning right");
                  needSecondTurn = false;
              } else if (turns == -1 || turns == 3) {
                  // Quay trái
                  navState = TURNING;
                  startEncoderCount = encoderCount;
                  targetEncoderCount = startEncoderCount + ENCODER_STEPS_PER_TURN;
                  turnLeft();
                  Serial.println("↪️ Quay trái để đổi hướng");
                  client.publish("robot/movement", "Turning left");
                  needSecondTurn = false;
              } else if (turns == 2 || turns == -2) {
                  // Quay 180 độ (hai lần quay phải 90 độ với tạm dừng ở giữa)
                  navState = TURNING;
                  startEncoderCount = encoderCount;
                  targetEncoderCount = startEncoderCount + ENCODER_STEPS_PER_TURN;
                  turnRight();
                  Serial.println("↩️↩️ Quay ngược lại (bước 1/2)");
                  client.publish("robot/movement", "Turning around (step 1/2)");
                  needSecondTurn = true;
              }
          }
          break;
          
      case TURNING:
          // Kiểm tra xem đã quay xong chưa
          if (encoderCount >= targetEncoderCount) {
              stop();
              
              // Cập nhật hướng robot dựa trên lần quay cuối cùng
              Serial.print("🧭 Cập nhật hướng robot. Hướng cũ: ");
              debugDirection(robotDir);
              
              if (lastTurnDirection == "right") {
                  robotDir = (Direction)((robotDir + 1) % 4);
              } else if (lastTurnDirection == "left") {
                  robotDir = (Direction)((robotDir + 3) % 4);  // +3 mod 4 tương đương với -1 mod 4
              }
              
              Serial.print(", Hướng mới: ");
              debugDirection(robotDir);
              Serial.println();
              
              // Gửi hướng hiện tại qua MQTT
              String dirMsg = "Current direction: ";
              switch(robotDir) {
                  case NORTH: dirMsg += "NORTH"; break;
                  case EAST: dirMsg += "EAST"; break;
                  case SOUTH: dirMsg += "SOUTH"; break;
                  case WEST: dirMsg += "WEST"; break;
              }
              client.publish("robot/direction", dirMsg.c_str());
              
              // Kiểm tra xem có cần quay thêm 90 độ nữa không (cho trường hợp quay 180 độ)
              if (needSecondTurn) {
                  // Chuyển sang trạng thái tạm dừng thay vì dùng delay
                  navState = TURN_PAUSE;
                  turnPauseStartTime = millis();
                  
                  Serial.println("⏸️ Tạm dừng giữa hai lượt quay");
                  client.publish("robot/movement", "Pausing between turns");
                  return;
              }
              
              // Kiểm tra xem đã quay đúng hướng chưa
              Direction targetDir = getTargetDirection(robotX, robotY, targetX, targetY);
              
              if (robotDir != targetDir) {
                  // Nếu vẫn chưa đúng hướng, quay tiếp
                  navState = IDLE; // Quay lại trạng thái IDLE để tính toán lại
              } else {
                  // Đã đúng hướng, sẵn sàng di chuyển tiếp
                  navState = MOVING;
                  startEncoderCount = encoderCount;
                  targetEncoderCount = startEncoderCount + ENCODER_STEPS_PER_CELL;
                  moveForward();
                  Serial.println("➡️ Đã đúng hướng, tiến về phía trước");
                  client.publish("robot/movement", "Moving forward");
              }
          }
          break;
          
      case TURN_PAUSE:
          // Kiểm tra xem đã tạm dừng đủ lâu chưa
          if (millis() - turnPauseStartTime >= TURN_PAUSE_DURATION) {
              // Thời gian tạm dừng đã đủ, bắt đầu quay tiếp
              needSecondTurn = false;
              startEncoderCount = encoderCount;
              targetEncoderCount = startEncoderCount + ENCODER_STEPS_PER_TURN;
              turnRight();
              navState = TURNING;
              Serial.println("↩️↩️ Quay ngược lại (bước 2/2)");
              client.publish("robot/movement", "Turning around (step 2/2)");
          }
          break;
          
      case MOVING:
          // Kiểm tra xem đã di chuyển xong chưa
          if (encoderCount >= targetEncoderCount) {
              stop();
              
              // Cập nhật vị trí robot dựa trên hướng hiện tại
              Serial.print("📍 Di chuyển từ (");
              Serial.print(robotX);
              Serial.print(",");
              Serial.print(robotY);
              Serial.print(") theo hướng ");
              debugDirection(robotDir);
              
              switch (robotDir) {
                  case NORTH: robotY--; break;
                  case EAST:  robotX++; break;
                  case SOUTH: robotY++; break;
                  case WEST:  robotX--; break;
              }
              
              Serial.print(" đến (");
              Serial.print(robotX);
              Serial.print(",");
              Serial.print(robotY);
              Serial.println(")");
              
              // Gửi vị trí hiện tại qua MQTT
              String posMsg = "Position: (" + String(robotX) + "," + String(robotY) + ")";
              client.publish("robot/position", posMsg.c_str());
              
              // Chuyển sang bước tiếp theo trong đường đi
              currentPathIndex++;
              
              // Ngay lập tức chuyển sang trạng thái IDLE để tiếp tục đến ô tiếp theo
              navState = IDLE;
          }
          break;
          
      case ARRIVED:
          // Không làm gì, đã hoàn thành
          break;
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

    pinMode(encoderPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encoderPin), encoderISR, RISING);

    ledcSetup(pwmChannelRight, freq, resolution);
    ledcAttachPin(rightEnable, pwmChannelRight);
    ledcSetup(pwmChannelLeft, freq, resolution);
    ledcAttachPin(leftEnable, pwmChannelLeft);

    initializeMap();
    stop();
    printMenu();
}

void processShapeDrawing() {
    if (!isDrawingShape) return;
    
    switch (shapeState) {
      case SHAPE_MOVING:
        // Check if the movement is complete
        if (encoderCount >= targetEncoderCount) {
          stop();
          
          if (shapeCommand == "CIRCLE") {
            // Circle is complete
            isDrawingShape = false;
            shapeState = SHAPE_FINISHED;
            Serial.println("🔵 Circle Drawing Complete");
            client.publish("robot/status", "Circle Drawing Complete");
          }
          else if (shapeCommand == "RECTANGLE") {
            shapeSteps++;
            
            if (shapeSteps >= shapeTotalSteps) {
              // Rectangle is complete
              isDrawingShape = false;
              shapeState = SHAPE_FINISHED;
              Serial.println("🟦 Rectangle Drawing Complete");
              client.publish("robot/status", "Rectangle Drawing Complete");
            }
            else if (shapeSteps % 2 == 1) {
              // After moving, turn 90 degrees
              shapeState = SHAPE_TURNING;
              startDrawingCount = encoderCount;
              targetEncoderCount = startDrawingCount + ENCODER_STEPS_PER_TURN;
              turnRight();
              Serial.print("🟦 Rectangle Turn - Step ");
              Serial.println(shapeSteps + 1);
              String statusMsg = "Rectangle Turn - Step " + String(shapeSteps + 1);
              client.publish("robot/status", statusMsg.c_str());
            }
            else {
              // After turning, move forward
              shapeState = SHAPE_MOVING;
              startDrawingCount = encoderCount;
              targetEncoderCount = startDrawingCount + 30;
              moveForward();
              Serial.print("🟦 Rectangle Side - Step ");
              Serial.println(shapeSteps + 1);
              String sideMsg = "Rectangle Side - Step " + String(shapeSteps + 1);
              client.publish("robot/status", sideMsg.c_str());
            }
          }
        }
        break;
        
      case SHAPE_TURNING:
        // Check if turning is complete
        if (encoderCount >= targetEncoderCount) {
          stop();
          shapeSteps++;
          
          if (shapeSteps >= shapeTotalSteps) {
            // Rectangle is complete
            isDrawingShape = false;
            shapeState = SHAPE_FINISHED;
            Serial.println("🟦 Rectangle Drawing Complete");
            client.publish("robot/status", "Rectangle Drawing Complete");
          }
          else {
            // After turning, move forward
            shapeState = SHAPE_MOVING;
            startDrawingCount = encoderCount;
            targetEncoderCount = startDrawingCount + 30;
            moveForward();
            Serial.print("🟦 Rectangle Side - Step ");
            Serial.println(shapeSteps + 1);
            // client.publish("robot/status", "Rectangle Side - Step " + String(shapeSteps + 1));
          }
        }
        break;
        
      case SHAPE_FINISHED:
        // Reset shape drawing state
        isDrawingShape = false;
        shapeState = SHAPE_IDLE;
        break;
        
      default:
        break;
    }
  }

// Cập nhật trong hàm loop hoặc callback để xử lý lệnh navigate
void processMqttCommand(String cmd) {
  cmd.toLowerCase();
  
  if (cmd.startsWith("navigate")) {
      processNavigateCommand(cmd);
  } 
  else if (cmd == "stop") {
      stop();
      navigationActive = false;
      navState = IDLE;
      Serial.println("⛔ Dừng điều hướng");
      client.publish("robot/status", "Navigation stopped");
  }
  else if (!navigationActive) {
      // Các lệnh điều khiển thủ công
      if (cmd == "forward") moveForward();
      else if (cmd == "backward") moveBackward();
      else if (cmd == "left") turnLeft();
      else if (cmd == "right") turnRight();
      else if (cmd == "move_left") moveLeft();
      else if (cmd == "move_right") moveRight();
      else if (cmd == "help") printMenu();
      else {
          Serial.print("❓ Lệnh không hợp lệ: ");
          Serial.println(cmd);
      }
  } else {
      Serial.println("🤖 Đang trong chế độ điều hướng tự động, bỏ qua lệnh: " + cmd);
  }
}

void loop() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();
  
    // Process commands
    if (mqttCommand.length() > 0) {
        String cmd = mqttCommand;
        mqttCommand = "";
        
        // Skip processing MQTT commands when drawing shapes
        if (!isDrawingShape) {
            processMqttCommand(cmd);
        }
    }
    
    // Process shape drawing (has priority over navigation)
    if (isDrawingShape) {
        processShapeDrawing();
    }
    // Auto navigation process (only when not drawing shapes)
    else if (navigationActive) {
        navigateToNextCell();
    }
  }