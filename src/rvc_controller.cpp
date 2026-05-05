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
        
        // Handle multiple JSON objects separated by \n or }{
        size_t pos;
        while ((pos = buffer.find('\n')) != std::string::npos) {
            std::string msg = buffer.substr(0, pos);
            buffer.erase(0, pos + 1);
            if (msg.empty()) continue;

            std::string type = get_json_value(msg, "type");
            if (type == "INTERRUPT") {
                if (onEmergency) onEmergency();
            } 
            else if (type == "EVENT") {
                std::string name = get_json_value(msg, "name");
                std::cout << "[Interface] Received Event: " << name << std::endl;
                if (ctrl_ref) {
                    if (name == "BUTTON_ON") ctrl_ref->turnOn();
                    else if (name == "BUTTON_OFF") ctrl_ref->turnOff();
                } else {
                    std::cout << "[Warning] No Controller Reference!" << std::endl;
                }
            }
        }
    }
}

// On-demand getters (can still poll if needed)
bool ObstacleSensorInterface::isFrontBlocked() {
    SimulatorBridge::getInstance().send_cmd("{\"type\": \"GET_SENSORS\"}");
    return false; // Result parsing would go here, keeping simple for Push focus
}

bool ObstacleSensorInterface::isLeftBlocked() { return false; }
bool ObstacleSensorInterface::isRightBlocked() { return false; }
ObstacleStatus ObstacleSensorInterface::isObstacleExist() { return {false, false, false}; }

// --- DustSensorInterface ---
bool DustSensorInterface::isDustExistence() {
    // Note: For now we return false or implement simple poll
    return false; 
}

// --- PathPlanner ---
PathPlanner::PathPlanner(ObstacleSensorInterface* o) : osi(o) {}
Location PathPlanner::decisionPath() { return Location::LEFT; } // Simple logic

// --- DriveManager ---
DriveManager::DriveManager(PathPlanner* pp) : pathPlanner(pp), currentDriveState(Driving::STOP), turnTimer(1000) {}
void DriveManager::sendDriveCommand(Driving state) {
    currentDriveState = state;
    std::string cmd = "{\"type\": \"SET_CONTROL\", \"move\": " + std::string(state == Driving::MOVEFORWARD ? "true" : "false") + 
                      ", \"turn\": " + std::string(state == Driving::TURNLEFT ? "-1" : (state == Driving::TURNRIGHT ? "1" : "0")) + "}";
    SimulatorBridge::getInstance().send_cmd(cmd);
}
Location DriveManager::avoidObstacle() {
    stopMotor();
    rotateLeft();
    turnTimer.setTimer(); // Blocking turn
    stopMotor();
    return Location::LEFT;
}
void DriveManager::rotateForward() { sendDriveCommand(Driving::MOVEFORWARD); }
void DriveManager::rotateLeft() { sendDriveCommand(Driving::TURNLEFT); }
void DriveManager::rotateRight() { sendDriveCommand(Driving::TURNRIGHT); }
void DriveManager::rotateBackward() { sendDriveCommand(Driving::MOVEBACKWARD); }
void DriveManager::stopMotor() { sendDriveCommand(Driving::STOP); }

// --- CleanerManager ---
CleanerManager::CleanerManager() : currentMode(CleanerMode::OFF), boostTimer(3000) {}
void CleanerManager::cleanerMode(CleanerMode mode) {
    currentMode = mode;
    sendCleanerCommand(mode);
    if (mode == CleanerMode::UP) {
        boostTimer.setTimer(); // Blocking boost
        cleanerMode(CleanerMode::ON);
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
        if (obstacleSensorInterface->isRightBlocked()) {
            errorturnOff();
        }
    } else if (turnLocation == Location::RIGHT) {
        if (obstacleSensorInterface->isLeftBlocked()) {
            errorturnOff();
        }
    }
    
    driveManager->rotateForward();
    cleanerManager->cleanerMode(CleanerMode::ON);

    {
        std::lock_guard<std::mutex> lock(ctrlMutex);
        isAvoiding = false;
    }
}

void Controller::dustDetect() {
    while(onOff) {
        bool shouldCheck = false;
        {
            std::lock_guard<std::mutex> lock(ctrlMutex);
            if (!isAvoiding && cleanerManager->iscleanerOn()) {
                shouldCheck = true;
            }
        }

        if (shouldCheck && dustSensorInterface->isDustExistence()) {
            std::lock_guard<std::mutex> lock(ctrlMutex);
            if (!isAvoiding && onOff) {
                cleanerManager->cleanerMode(CleanerMode::UP);
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

void Controller::errorturnOff() { turnOff(); }


// --- ButtonInterface ---
ButtonInterface::ButtonInterface(Controller* ctrl) : controller(ctrl) {}
void ButtonInterface::pushButtonOn() { if (controller) controller->turnOn(); }
void ButtonInterface::pushButtonOff() { if (controller) controller->turnOff(); }
