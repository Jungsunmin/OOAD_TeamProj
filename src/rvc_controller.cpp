#include "rvc_controller.h"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <sstream>

// --- Central Simulator Bridge ---
class SimulatorBridge {
private:
    int sock = -1;
    std::mutex mtx;
    std::string last_resp;

    int connect_to_sim() {
        if (sock != -1) return sock;
        sock = socket(AF_INET, SOCK_STREAM, 0);
        struct sockaddr_in addr;
        addr.sin_family = AF_INET;
        addr.sin_port = htons(12345);
        inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
        if (connect(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            close(sock); sock = -1;
        }
        return sock;
    }

public:
    static SimulatorBridge& getInstance() {
        static SimulatorBridge instance;
        return instance;
    }

    std::string request(const std::string& req) {
        std::lock_guard<std::mutex> lock(mtx);
        int s = connect_to_sim();
        if (s == -1) return "";
        if (send(s, req.c_str(), req.length(), 0) < 0) {
            close(sock); sock = -1; return "";
        }
        char buf[4096] = {0};
        int val = recv(s, buf, 4096, 0);
        if (val <= 0) {
            close(sock); sock = -1; return "";
        }
        last_resp = std::string(buf, val);
        return last_resp;
    }

    std::string getLastResponse() {
        std::lock_guard<std::mutex> lock(mtx);
        return last_resp;
    }
};

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

// --- Timer Implementation ---
Timer::Timer(unsigned long ms) : duration(ms) {}
void Timer::setTimer() {
    std::this_thread::sleep_for(std::chrono::milliseconds(duration));
}

// --- ObstacleSensorInterface ---
ObstacleSensorInterface::ObstacleSensorInterface() {}
bool ObstacleSensorInterface::isFrontBlocked() {
    std::string resp = SimulatorBridge::getInstance().getLastResponse();
    std::string v = get_json_value(resp, "front");
    return (!v.empty() && std::stoi(v) <= THRESHOLD);
}
bool ObstacleSensorInterface::isLeftBlocked() {
    std::string resp = SimulatorBridge::getInstance().getLastResponse();
    std::string v = get_json_value(resp, "left");
    return (!v.empty() && std::stoi(v) <= THRESHOLD);
}
bool ObstacleSensorInterface::isRightBlocked() {
    std::string resp = SimulatorBridge::getInstance().getLastResponse();
    std::string v = get_json_value(resp, "right");
    return (!v.empty() && std::stoi(v) <= THRESHOLD);
}
ObstacleStatus ObstacleSensorInterface::isObstacleExist() {
    return {isFrontBlocked(), isLeftBlocked(), isRightBlocked()};
}
void ObstacleSensorInterface::attachInterrupt(std::function<void()> cb) { onEmergency = cb; }
void ObstacleSensorInterface::triggerInterrupt() { if(onEmergency) onEmergency(); }

// --- DustSensorInterface ---
DustSensorInterface::DustSensorInterface() {}
bool DustSensorInterface::isDustExistence() {
    std::string resp = SimulatorBridge::getInstance().getLastResponse();
    std::string v = get_json_value(resp, "dust");
    return (!v.empty() && std::stoi(v) >= DUST_EXISTENCE_THRESHOLD);
}

// --- PathPlanner ---
PathPlanner::PathPlanner(ObstacleSensorInterface* osi) : osi(osi) {}
Location PathPlanner::decisionPath() {
    ObstacleStatus s = osi->isObstacleExist();
    if (!s.leftBlocked) return Location::LEFT;
    if (!s.rightBlocked) return Location::RIGHT;
    return Location::REAR;
}

// --- DriveManager ---
DriveManager::DriveManager(PathPlanner* pp) : pathPlanner(pp), currentDriveState(Driving::STOP), timer(1000) {}
void DriveManager::sendDriveCommand(Driving state) {
    currentDriveState = state;
    std::string cmd;
    if (state == Driving::MOVEFORWARD) cmd = "{\"type\": \"SET_CONTROL\", \"move\": true, \"turn\": 0}";
    else if (state == Driving::STOP) cmd = "{\"type\": \"SET_CONTROL\", \"move\": false, \"turn\": 0}";
    else if (state == Driving::TURNLEFT) cmd = "{\"type\": \"SET_CONTROL\", \"move\": false, \"turn\": -1}";
    else if (state == Driving::TURNRIGHT) cmd = "{\"type\": \"SET_CONTROL\", \"move\": false, \"turn\": 1}";
    else if (state == Driving::MOVEBACKWARD) cmd = "{\"type\": \"SET_CONTROL\", \"move\": true, \"turn\": 1}";
    SimulatorBridge::getInstance().request(cmd);
}
Location DriveManager::avoidObstacle() {
    sendDriveCommand(Driving::STOP);
    Location location = pathPlanner->decisionPath();
    if (location == Location::LEFT) {
        rotateLeft();
        timer.setTimer();
        stopMotor();
        rotateForward();
        return Location::LEFT;
    } else if (location == Location::RIGHT) {
        rotateRight();
        timer.setTimer();
        stopMotor();
        rotateForward();
        return Location::RIGHT;
    } else if (location == Location::REAR) {
        rotateBackward();
        return Location::REAR;
    } 
    return Location::FRONT;
}
void DriveManager::rotateForward() { sendDriveCommand(Driving::MOVEFORWARD); }
void DriveManager::rotateLeft() { sendDriveCommand(Driving::TURNLEFT); }
void DriveManager::rotateRight() { sendDriveCommand(Driving::TURNRIGHT); }
void DriveManager::rotateBackward() { sendDriveCommand(Driving::MOVEBACKWARD); }
void DriveManager::stopMotor() { sendDriveCommand(Driving::STOP); }

// --- CleanerManager ---
CleanerManager::CleanerManager() : currentMode(CleanerMode::OFF), timer(3000) {}
void CleanerManager::cleanerMode(CleanerMode mode) {
    currentMode = mode;
    sendCleanerCommand(mode);
    if (mode == CleanerMode::UP) {
        timer.setTimer(); // Blocking
        cleanerMode(CleanerMode::ON);
    }
}
void CleanerManager::sendCleanerCommand(CleanerMode mode) {
    std::string m = (mode == CleanerMode::OFF) ? "OFF" : (mode == CleanerMode::ON ? "ON" : "UP");
    bool clean = (mode != CleanerMode::OFF);
    std::string cmd = "{\"type\": \"SET_CONTROL\", \"clean\": " + std::string(clean?"true":"false") + ", \"mode\": \"" + m + "\"}";
    SimulatorBridge::getInstance().request(cmd);
}
void CleanerManager::updateTimer() {} // Blocking model, not used

// --- Controller ---
Controller::Controller(DriveManager* drive, CleanerManager* cleaner, DustSensorInterface* dsi, ObstacleSensorInterface* osi)
    : driveManager(drive), cleanerManager(cleaner), dustSensorInterface(dsi), obstacleSensorInterface(osi), onOff(false) {
    obstacleSensorInterface->attachInterrupt([this](){ this->interruptHandler(); });
}

Controller::~Controller() {
    onOff = false;
    if (dustThread.joinable()) dustThread.join();
}

void Controller::interruptHandler() {
    if (!onOff) return;
    cleanerManager->cleanerMode(CleanerMode::OFF);
    Location turnLocation = driveManager->avoidObstacle();
    if(turnLocation == Location::LEFT) {
        if(obstacleSensorInterface->isRightBlocked()) {
            errorturnOff();
        }
    } else if (turnLocation == Location::RIGHT) {
        if(obstacleSensorInterface->isLeftBlocked()) {
            errorturnOff();
        }
    } else if (turnLocation == Location::REAR) {
        resumeSideSensorCheck();
    }
    cleanerManager->cleanerMode(CleanerMode::ON);
    clearDustValue();
}

void Controller::turnOn() {
    if (onOff) return;
    onOff = true;
    cleanerManager->cleanerMode(CleanerMode::ON);
    driveManager->rotateForward();
    
    if (!dustThread.joinable()) {
        dustThread = std::thread(&Controller::dustDetect, this);
    }
}

void Controller::turnOff() {
    if (!onOff) return;
    onOff = false;
    cleanerManager->cleanerMode(CleanerMode::OFF);
    driveManager->stopMotor();
}

void Controller::errorturnOff() {
    std::cout << "Controller: Error Turn Off\n";
    turnOff();
}

void Controller::clearDustValue() {
    clearValue = true;
}

void Controller::resumeSideSensorCheck() {
    while(obstacleSensorInterface->isLeftBlocked() && obstacleSensorInterface->isRightBlocked()) {
        usleep(100000);
        continue;
    }
    if (!obstacleSensorInterface->isLeftBlocked()) {
        driveManager->rotateLeft();
    } else {
        driveManager->rotateRight();
    }
}

void Controller::dustDetect() {
    while(onOff) {
        usleep(100000);
        if(dustSensorInterface->isDustExistence()) {
            cleanerManager->cleanerMode(CleanerMode::UP);
        }
        if(clearValue) {
            cleanerManager->cleanerMode(CleanerMode::ON);
            clearValue = false;
        }
    }
}

// --- ButtonInterface ---
ButtonInterface::ButtonInterface(Controller* ctrl) : controller(ctrl), running(true) {
    listener = std::thread(&ButtonInterface::listen, this);
}
ButtonInterface::~ButtonInterface() {
    running = false;
    if (listener.joinable()) listener.join();
}

void ButtonInterface::listen() {
    std::cout << "[System] Monitoring Boundary..." << std::endl;
    while (running) {
        std::string resp = SimulatorBridge::getInstance().request("{\"type\": \"GET_SENSORS\"}");
        if (!resp.empty()) {
            // Button Events
            std::string ev = get_json_value(resp, "event");
            if (ev == "BUTTON_ON") pushButtonOn();
            else if (ev == "BUTTON_OFF") pushButtonOff();

            // Obstacle Interrupts
            std::string front = get_json_value(resp, "front");
            if (!front.empty() && std::stoi(front) <= 10) {
                controller->getObstacleSensorInterface()->triggerInterrupt();
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void ButtonInterface::pushButtonOn() { if (controller) controller->turnOn(); }
void ButtonInterface::pushButtonOff() { if (controller) controller->turnOff(); }
