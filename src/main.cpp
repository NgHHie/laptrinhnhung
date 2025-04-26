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
const int ENCODER_STEPS_PER_TURN = 17;  // Estimated from your code

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
    int cost;        // Cost from start
    int heuristic;   // Estimated cost to goal
    int totalCost;   // cost + heuristic
    int parentX;
    int parentY;
    
    bool operator>(const Node& other) const {
        return totalCost > other.totalCost;
    }
};

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
    digitalWrite(rightIN1, HIGH);
    digitalWrite(rightIN2, LOW);
    digitalWrite(leftIN3, LOW);
    digitalWrite(leftIN4, HIGH);
    ledcWrite(pwmChannelRight, dutyCycle);
    ledcWrite(pwmChannelLeft, dutyCycle);
    Serial.println("↪️ Trái");
    client.publish("robot/movement", "Turning left");
    currentState = "left";
    lastTurnDirection = "left";
}

void turnRight() {
    digitalWrite(rightIN1, LOW);
    digitalWrite(rightIN2, HIGH);
    digitalWrite(leftIN3, HIGH);
    digitalWrite(leftIN4, LOW); 
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
    ledcWrite(pwmChannelRight, dutyCycle / 2);
    ledcWrite(pwmChannelLeft, dutyCycle);
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

void printMenu() {
    Serial.println("===== MENU XE MQTT =====");
    Serial.println("Lệnh: forward | backward | left | right | stop | move_left | move_right | navigate");
    Serial.println("Lệnh navigate có thể chỉ định vị trí xuất phát và đích: navigate:startX,startY:goalX,goalY");
    Serial.println("Ví dụ: navigate:0,0:9,9");
    Serial.println("========================");
    
    // Gửi menu lệnh qua MQTT
    client.publish("robot/help", "Commands: forward | backward | left | right | stop | move_left | move_right | navigate");
    client.publish("robot/help", "For custom navigation: navigate:startX,startY:goalX,goalY (e.g., navigate:0,0:9,9)");
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
    Serial.println("🔍 Đang tìm đường đi...");
    client.publish("robot/status", "Finding path...");
    
    // Find destination coordinates by searching for value 2 in the map
    int goalX = -1;
    int goalY = -1;
    
    for (int y = 0; y < MAP_SIZE; y++) {
        for (int x = 0; x < MAP_SIZE; x++) {
            if (mapGrid[y][x] == 2) {
                goalX = x;
                goalY = y;
                break;
            }
        }
        if (goalX != -1) break;
    }
    
    // If destination not found, use default (MAP_SIZE-1, MAP_SIZE-1)
    if (goalX == -1 || goalY == -1) {
        goalX = MAP_SIZE - 1;
        goalY = MAP_SIZE - 1;
        Serial.println("⚠️ Không tìm thấy đích trên bản đồ, sử dụng vị trí mặc định (9,9)");
    }
    
    // Clear previous path
    path.clear();
    
    // Create closed list (visited nodes)
    bool closed[MAP_SIZE][MAP_SIZE] = {false};
    
    // Create open list (priority queue)
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openList;
    
    // Create parent map
    int parentX[MAP_SIZE][MAP_SIZE] = {-1};
    int parentY[MAP_SIZE][MAP_SIZE] = {-1};
    
    // Add starting position to open list
    Node startNode;
    startNode.x = robotX;
    startNode.y = robotY;
    startNode.cost = 0;
    startNode.heuristic = calculateHeuristic(robotX, robotY, goalX, goalY);
    startNode.totalCost = startNode.cost + startNode.heuristic;
    startNode.parentX = -1;
    startNode.parentY = -1;
    
    openList.push(startNode);
    
    // A* algorithm
    while (!openList.empty()) {
        // Get node with lowest total cost
        Node current = openList.top();
        openList.pop();
        
        int x = current.x;
        int y = current.y;
        
        // Skip if already visited
        if (closed[y][x]) {
            continue;
        }
        
        // Mark as visited
        closed[y][x] = true;
        parentX[y][x] = current.parentX;
        parentY[y][x] = current.parentY;
        
        // If goal reached, construct path
        if (x == goalX && y == goalY) {
            // Reconstruct path from goal to start
            std::vector<std::pair<int, int>> reversePath;
            int currX = goalX;
            int currY = goalY;
            
            while (currX != robotX || currY != robotY) {
                reversePath.push_back(std::make_pair(currX, currY));
                int tempX = parentX[currY][currX];
                int tempY = parentY[currY][currX];
                currX = tempX;
                currY = tempY;
            }
            
            // Reverse to get path from start to goal
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
            
            // Print the path
            Serial.println("🛣️ Đường đi: ");
            Serial.println(pathMsg);
            
            pathFound = true;
            return;
        }
        
        // Explore 4-connected neighbors (up, right, down, left)
        int dx[4] = {0, 1, 0, -1};
        int dy[4] = {-1, 0, 1, 0};
        
        for (int i = 0; i < 4; i++) {
            int newX = x + dx[i];
            int newY = y + dy[i];
            
            // Check if valid position
            if (newX >= 0 && newX < MAP_SIZE && newY >= 0 && newY < MAP_SIZE && !closed[newY][newX]) {
                Node neighbor;
                neighbor.x = newX;
                neighbor.y = newY;
                neighbor.cost = current.cost + 1;  // Each step costs 1
                neighbor.heuristic = calculateHeuristic(newX, newY, goalX, goalY);
                neighbor.totalCost = neighbor.cost + neighbor.heuristic;
                neighbor.parentX = x;
                neighbor.parentY = y;
                
                openList.push(neighbor);
            }
        }
    }
    
    Serial.println("❌ Không tìm thấy đường đi!");
    client.publish("robot/status", "Path not found!");
    pathFound = false;
}

// ====== Navigation State Machine ======
enum NavigationState {
    IDLE,
    TURNING,
    MOVING,
    ARRIVED
};

NavigationState navState = IDLE;
int targetEncoderCount = 0;
int startEncoderCount = 0;
bool needSecondTurn = false;

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
        // Reached the end of the path
        stop();
        navigationActive = false;
        navState = ARRIVED;
        Serial.println("🎯 Đã đến đích!");
        client.publish("robot/status", "Destination reached! Mission completed!");
        return;
    }
    
    // Get next target cell
    int targetX = path[currentPathIndex].first;
    int targetY = path[currentPathIndex].second;
    
    switch (navState) {
        case IDLE:
            // Determine direction to face
            {
                Direction targetDir = getTargetDirection(robotX, robotY, targetX, targetY);
                turnToDirection(targetDir);
            }
            break;
            
        case TURNING:
            // Check if turning is complete
            if (encoderCount >= targetEncoderCount) {
                stop();
                
                // Update robot direction based on the last turn action
                Serial.print("🧭 Cập nhật hướng robot. Hướng cũ: ");
                debugDirection(robotDir);
                
                if (lastTurnDirection == "right") {
                    robotDir = (Direction)((robotDir + 1) % 4);
                } else if (lastTurnDirection == "left") {
                    robotDir = (Direction)((robotDir + 3) % 4);  // +3 mod 4 is equivalent to -1 mod 4
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
                
                // Check if we need to do a second turn for a 180-degree turn
                if (needSecondTurn) {
                    needSecondTurn = false;
                    startEncoderCount = encoderCount;
                    targetEncoderCount = startEncoderCount + ENCODER_STEPS_PER_TURN;
                    turnRight();
                    Serial.println("↩️↩️ Quay ngược lại (bước 2/2)");
                    client.publish("robot/movement", "Turning around (step 2/2)");
                    return;
                }
                
                // Check if we're now facing the right direction
                Direction targetDir = getTargetDirection(robotX, robotY, targetX, targetY);
                
                if (robotDir != targetDir) {
                    // If we're still not facing the right direction, turn again
                    turnToDirection(targetDir);
                } else {
                    // Ready to move forward
                    navState = MOVING;
                    startEncoderCount = encoderCount;
                    targetEncoderCount = startEncoderCount + ENCODER_STEPS_PER_CELL;
                    moveForward();
                    Serial.println("➡️ Đã đúng hướng, tiến về phía trước");
                    client.publish("robot/movement", "Moving forward");
                }
            }
            break;
            
        case MOVING:
            // Check if move is complete
            if (encoderCount >= targetEncoderCount) {
                stop();
                
                // Update robot position based on current direction
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
                
                // Move to next step in path
                currentPathIndex++;
                
                // Immediately transition to IDLE to continue to the next cell
                navState = IDLE;
                
                // Give a short pause for stability
                delay(200);
            }
            break;
            
        case ARRIVED:
            // Do nothing, we're done
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

void loop() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();

    // Process commands
    if (mqttCommand.length() > 0) {
        String cmd = mqttCommand;
        mqttCommand = "";
        
        cmd.toLowerCase();
        
        if (cmd.startsWith("navigate")) {
            // Check if coordinates are provided in format: navigate:startX,startY:goalX,goalY
            if (cmd.indexOf(":") > 0) {
                // Parse coordinates
                int firstColon = cmd.indexOf(":");
                int secondColon = cmd.indexOf(":", firstColon + 1);
                
                if (secondColon > 0) {
                    String startCoords = cmd.substring(firstColon + 1, secondColon);
                    String goalCoords = cmd.substring(secondColon + 1);
                    
                    int startComma = startCoords.indexOf(",");
                    int goalComma = goalCoords.indexOf(",");
                    
                    if (startComma > 0 && goalComma > 0) {
                        int startX = startCoords.substring(0, startComma).toInt();
                        int startY = startCoords.substring(startComma + 1).toInt();
                        int goalX = goalCoords.substring(0, goalComma).toInt();
                        int goalY = goalCoords.substring(goalComma + 1).toInt();
                        
                        // Validate coordinates
                        if (startX >= 0 && startX < MAP_SIZE && 
                            startY >= 0 && startY < MAP_SIZE && 
                            goalX >= 0 && goalX < MAP_SIZE && 
                            goalY >= 0 && goalY < MAP_SIZE) {
                            
                            // Update robot position and map
                            robotX = startX;
                            robotY = startY;
                            
                            // Clear the map
                            for (int y = 0; y < MAP_SIZE; y++) {
                                for (int x = 0; x < MAP_SIZE; x++) {
                                    mapGrid[y][x] = 0;  // Empty space
                                }
                            }
                            
                            // Set robot position and destination
                            mapGrid[startY][startX] = 1;          // Robot starting position
                            mapGrid[goalY][goalX] = 2;            // Destination
                            
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
                            
                            client.publish("robot/status", "Custom start and goal set");
                            
                            // Find path with the new positions
                            findPath();
                            if (pathFound) {
                                navigationActive = true;
                                currentPathIndex = 0;
                                navState = IDLE;
                                needSecondTurn = false;
                                Serial.println("🚀 Bắt đầu điều hướng tự động");
                                client.publish("robot/status", "Navigation started");
                            }
                        } else {
                            Serial.println("❌ Tọa độ không hợp lệ!");
                            client.publish("robot/status", "Invalid coordinates!");
                        }
                    } else {
                        Serial.println("❌ Định dạng tọa độ không hợp lệ!");
                        client.publish("robot/status", "Invalid coordinate format!");
                    }
                } else {
                    Serial.println("❌ Thiếu dấu hai chấm thứ hai!");
                    client.publish("robot/status", "Missing second colon!");
                }
            } else {
                // Default navigation from current position to 9,9
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
        } 
        else if (cmd == "stop") {
            stop();
            navigationActive = false;
            navState = IDLE;
            Serial.println("⛔ Dừng điều hướng");
            client.publish("robot/status", "Navigation stopped");
        }
        else if (!navigationActive) {
            // Manual control commands
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
    
    // Auto navigation process
    if (navigationActive) {
        navigateToNextCell();
    }
}