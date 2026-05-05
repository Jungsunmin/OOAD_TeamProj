#include "rvc_controller.h"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <sstream>

// --- Helper Functions ---
static std::string get_json_value(const std::string& json, const std::string& key) {
    size_t key_pos = json.find("\"" + key + "\"");
    if (key_pos == std::string::npos) return "";
    size_t colon_pos = json.find(":", key_pos);
    if (colon_pos == std::string::npos) return "";
    size_t start = json.find_first_not_of(" \t\n\r\"", colon_pos + 1);
    size_t end = json.find_first_of(",}", start);
    if (start == std::string::npos || end == std::string::npos) return "";
    std::string val = json.substr(start, end - start);
    if (!val.empty() && val.back() == '"') val.pop_back();
    return val;
}

// --- Simulator Bridge (Single Connection) ---
class SimulatorBridge {
private:
    int sock = -1;
    std::mutex mtx;
public:
    static SimulatorBridge& getInstance() { static SimulatorBridge instance; return instance; }
    int get_sock() {
        std::lock_guard<std::mutex> lock(mtx);
        if (sock != -1) return sock;
        sock = socket(AF_INET, SOCK_STREAM, 0);
        struct sockaddr_in addr;
        addr.sin_family = AF_INET;
        addr.sin_port = htons(12345);
        inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
        if (connect(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) { close(sock); sock = -1; }
        return sock;
    }
    void send_cmd(const std::string& cmd) {
        int s = get_sock();
        if (s != -1) send(s, (cmd + "\n").c_str(), cmd.length() + 1, 0);
    }
};

// --- Timer ---
Timer::Timer(unsigned long ms) : duration(ms) {}
void Timer::setTimer() {
    std::this_thread::sleep_for(std::chrono::milliseconds(duration));
}

// --- ObstacleSensorInterface ---
ObstacleSensorInterface::ObstacleSensorInterface() : running(true) {
    listenerThread = std::thread(&ObstacleSensorInterface::listen, this);
}
ObstacleSensorInterface::~ObstacleSensorInterface() {
    running = false;
    if (listenerThread.joinable()) listenerThread.join();
}
void ObstacleSensorInterface::attachInterrupt(std::function<void()> cb) { onEmergency = cb; }

void ObstacleSensorInterface::listen() {
    std::cout << "[Interface] Listening for Hardware Interrupts..." << std::endl;
    std::string buffer = "";
    while (running) {
        int sock = SimulatorBridge::getInstance().get_sock();
        if (sock == -1) { std::this_thread::sleep_for(std::chrono::seconds(1)); continue; }
        
        char chunk[4096] = {0};
        int valread = recv(sock, chunk, 4096, 0);
        if (valread <= 0) { std::this_thread::sleep_for(std::chrono::milliseconds(50)); continue; }
        
        buffer += std::string(chunk, valread);
        
        size_t pos;
        while ((pos = buffer.find('\n')) != std::string::npos) {
            std::string msg = buffer.substr(0, pos);
            buffer.erase(0, pos + 1);
            if (msg.empty()) continue;

            std::string type = get_json_value(msg, "type");
            if (type == "INTERRUPT") {
                if (onEmergency) {
                    std::thread([this](){ this->onEmergency(); }).detach();
                }
            } 
            else if (type == "EVENT") {
                std::string name = get_json_value(msg, "name");
                std::cout << "[Interface] Received Event: " << name << std::endl;
                if (ctrl_ref) {
                    if (name == "BUTTON_ON") ctrl_ref->turnOn();
                    else if (name == "BUTTON_OFF") ctrl_ref->turnOff();
                    else if (name == "RESET") ctrl_ref->resetController();
                }
            } else {
                // Check for sensor values
                std::string front_val = get_json_value(msg, "front");
                std::string left_val = get_json_value(msg, "left");
                std::string right_val = get_json_value(msg, "right");
                std::string dust_val = get_json_value(msg, "dust");
                
                std::lock_guard<std::mutex> lock(sensorMutex);
                if (!front_val.empty()) last_front = std::stoi(front_val);
                if (!left_val.empty()) last_left = std::stoi(left_val);
                if (!right_val.empty()) last_right = std::stoi(right_val);
                if (!dust_val.empty()) last_dust = std::stoi(dust_val);
            }
        }
    }
}

bool ObstacleSensorInterface::isFrontBlocked() {
    SimulatorBridge::getInstance().send_cmd("{\"type\": \"GET_SENSORS\"}");
    std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Small wait for response
    std::lock_guard<std::mutex> lock(sensorMutex);
    return last_front <= threshold;
}

bool ObstacleSensorInterface::isLeftBlocked() {
    SimulatorBridge::getInstance().send_cmd("{\"type\": \"GET_SENSORS\"}");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    std::lock_guard<std::mutex> lock(sensorMutex);
    return last_left <= thresholdside;
}

bool ObstacleSensorInterface::isRightBlocked() {
    SimulatorBridge::getInstance().send_cmd("{\"type\": \"GET_SENSORS\"}");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    std::lock_guard<std::mutex> lock(sensorMutex);
    return last_right <= thresholdside;
}

ObstacleStatus ObstacleSensorInterface::isObstacleExist() {
    SimulatorBridge::getInstance().send_cmd("{\"type\": \"GET_SENSORS\"}");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    std::lock_guard<std::mutex> lock(sensorMutex);
    return {last_front <= threshold, last_left <= thresholdside, last_right <= thresholdside};
}

bool ObstacleSensorInterface::isDustExistence() {
    SimulatorBridge::getInstance().send_cmd("{\"type\": \"GET_SENSORS\"}");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    std::lock_guard<std::mutex> lock(sensorMutex);
    return last_dust > 0;
}

// --- DustSensorInterface ---
bool DustSensorInterface::isDustExistence() {
    return false; // Will be handled via ObstacleSensorInterface in Controller
}

// --- PathPlanner ---
PathPlanner::PathPlanner(ObstacleSensorInterface* o) : osi(o) {}
Location PathPlanner::decisionPath() { 
    if (osi->isLeftBlocked()){
        if (osi->isRightBlocked()) return Location::REAR;
        else return Location::RIGHT;
    } else return Location::LEFT;
}

// --- DriveManager ---
DriveManager::DriveManager(PathPlanner* pp) : pathPlanner(pp), currentDriveState(Driving::STOP), turnTimer(1020) {}
void DriveManager::sendDriveCommand(Driving state) {
    currentDriveState = state;
    int moveVal = 0;
    if (state == Driving::MOVEFORWARD) moveVal = 1;
    else if (state == Driving::MOVEBACKWARD) moveVal = -1;

    int turnVal = 0;
    if (state == Driving::TURNLEFT) turnVal = -1;
    else if (state == Driving::TURNRIGHT) turnVal = 1;

    std::string cmd = "{\"type\": \"SET_CONTROL\", \"move\": " + std::to_string(moveVal) + 
                      ", \"turn\": " + std::to_string(turnVal) + "}";
    SimulatorBridge::getInstance().send_cmd(cmd);
}
Location DriveManager::avoidObstacle() {
    stopMotor();
    Location turn = pathPlanner->decisionPath();
    if (turn == Location::LEFT) {
        rotateLeft();
        turnTimer.setTimer();
        stopMotor();
        rotateForward();
        return turn;
    }
    else if (turn == Location::RIGHT) {
        rotateRight();
        turnTimer.setTimer();
        stopMotor();
        rotateForward();
        return turn;
    }
    else {
        rotateBackward();
        return turn;
    }
    
}
void DriveManager::rotateForward() { sendDriveCommand(Driving::MOVEFORWARD); }
void DriveManager::rotateLeft() { sendDriveCommand(Driving::TURNLEFT); }
void DriveManager::rotateRight() { sendDriveCommand(Driving::TURNRIGHT); }
void DriveManager::rotateBackward() { sendDriveCommand(Driving::MOVEBACKWARD); }
void DriveManager::stopMotor() { sendDriveCommand(Driving::STOP); }
void DriveManager::rotateLeftb() {
    sendDriveCommand(Driving::TURNLEFT);
    turnTimer.setTimer();
    stopMotor();
    rotateForward();
 }
void DriveManager::rotateRightb() { 
    sendDriveCommand(Driving::TURNRIGHT);
    turnTimer.setTimer();
    stopMotor();
    rotateForward(); 
}
// --- CleanerManager ---
CleanerManager::CleanerManager() : currentMode(CleanerMode::OFF), boostTimer(3000) {}
void CleanerManager::cleanerMode(CleanerMode mode) {
    if (currentMode == mode) return;
    currentMode = mode;
    sendCleanerCommand(mode);
    if (mode == CleanerMode::UP) {
        // Non-blocking boost: Start a thread to switch back to ON after delay
        std::thread([this](){
            std::this_thread::sleep_for(std::chrono::milliseconds(3000));
            if (this->currentMode == CleanerMode::UP) {
                this->cleanerMode(CleanerMode::ON);
            }
        }).detach();
    }
}
void CleanerManager::sendCleanerCommand(CleanerMode mode) {
    std::string m = (mode == CleanerMode::OFF) ? "OFF" : (mode == CleanerMode::ON ? "ON" : "UP");
    std::string cmd = "{\"type\": \"SET_CONTROL\", \"clean\": " + std::string(mode != CleanerMode::OFF ? "true" : "false") + ", \"mode\": \"" + m + "\"}";
    SimulatorBridge::getInstance().send_cmd(cmd);
}
bool CleanerManager::iscleanerOn() {
    return currentMode != CleanerMode::OFF;
}

// --- Controller ---
Controller::Controller(DriveManager* d, CleanerManager* c, DustSensorInterface* ds, ObstacleSensorInterface* os)
    : driveManager(d), cleanerManager(c), dustSensorInterface(ds), obstacleSensorInterface(os) {
    obstacleSensorInterface->attachInterrupt([this](){ this->interruptHandler(); });
    obstacleSensorInterface->setController(this);
}

Controller::~Controller() { 
    onOff = false; 
    if (dustThread.joinable()) dustThread.join(); 
}

void Controller::interruptHandler() {
    if (!onOff) return;
    
    {
        std::lock_guard<std::mutex> lock(ctrlMutex);
        isAvoiding = true;
    }

    std::cout << "[System] INTERRUPT: Obstacle Avoidance Triggered!" << std::endl;
    cleanerManager->cleanerMode(CleanerMode::OFF);
    Location turnLocation = driveManager->avoidObstacle();
    
    if (turnLocation == Location::LEFT) {
        if (!obstacleSensorInterface->isRightBlocked()) {
            errorturnOff();
        }else{
            cleanerManager->cleanerMode(CleanerMode::ON);
        }
    } else if (turnLocation == Location::RIGHT) {
        if (!obstacleSensorInterface->isLeftBlocked()) {
            errorturnOff();
        }else cleanerManager->cleanerMode(CleanerMode::ON);
    }else{
        std::cout << "[System] All sides blocked. Moving backward until a side is clear..." << std::endl;
        while (onOff){
            if (!obstacleSensorInterface->isLeftBlocked()){
                driveManager->rotateLeftb();
                break;
            }
            else if (!obstacleSensorInterface->isRightBlocked()){
                driveManager->rotateRightb();
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        if (onOff) cleanerManager->cleanerMode(CleanerMode::ON);
    }
    
   

    {
        std::lock_guard<std::mutex> lock(ctrlMutex);
        isAvoiding = false;
    }
}

void Controller::dustDetect() {
    while(onOff) {
        bool canCheck = false;
        {
            std::lock_guard<std::mutex> lock(ctrlMutex);
            if (!isAvoiding && cleanerManager->iscleanerOn()) {
                canCheck = true;
            }
        }

        if (canCheck) {
            if (obstacleSensorInterface->isDustExistence()) {
                std::lock_guard<std::mutex> lock(ctrlMutex);
                if (!isAvoiding && onOff) {
                    cleanerManager->cleanerMode(CleanerMode::UP);
                }
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void Controller::turnOn() {
    if (onOff) return;
    std::cout << "[System] POWER ON" << std::endl;
    onOff = true;
    cleanerManager->cleanerMode(CleanerMode::ON);
    driveManager->rotateForward();
    
    if (dustThread.joinable()) dustThread.join();
    dustThread = std::thread(&Controller::dustDetect, this);
}

void Controller::turnOff() {
    if (!onOff) return;
    std::cout << "[System] POWER OFF" << std::endl;
    onOff = false;
    
    if (dustThread.joinable()) dustThread.join();
    
    cleanerManager->cleanerMode(CleanerMode::OFF);
    driveManager->stopMotor();
}

void Controller::resetController() {
    std::cout << "[System] RESETTING CONTROLLER STATE..." << std::endl;
    onOff = false;
    {
        std::lock_guard<std::mutex> lock(ctrlMutex);
        isAvoiding = false;
    }
    if (dustThread.joinable()) dustThread.join();
    
    cleanerManager->cleanerMode(CleanerMode::OFF);
    driveManager->stopMotor();
}

void Controller::errorturnOff() { turnOff(); }

// --- ButtonInterface ---
ButtonInterface::ButtonInterface(Controller* ctrl) : controller(ctrl) {}
void ButtonInterface::pushButtonOn() { if (controller) controller->turnOn(); }
void ButtonInterface::pushButtonOff() { if (controller) controller->turnOff(); }
