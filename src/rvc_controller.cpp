#include "rvc_controller.h"

// --- Motor Implementation ---
void Motor::rotateForward() { std::cout << "Motor: Forward\n"; }
void Motor::rotateBackward() { std::cout << "Motor: Backward\n"; }
void Motor::rotateLeft() { std::cout << "Motor: Left\n"; }
void Motor::rotateRight() { std::cout << "Motor: Right\n"; }
void Motor::stopMotor() { std::cout << "Motor: Stop\n"; }

// --- Sensor Implementations ---
void FrontObstacleSensor::setBlocked(bool b) { blocked = b; }
bool FrontObstacleSensor::isBlocked() const { return blocked; }

void SideObstacleSensor::setStatus(bool l, bool r) { leftBlocked = l; rightBlocked = r; }
bool SideObstacleSensor::isLeftBlocked() const { return leftBlocked; }
bool SideObstacleSensor::isRightBlocked() const { return rightBlocked; }

void DustSensor::setDust(bool d) { dustDetected = d; }
bool DustSensor::isDustExist() const { return dustDetected; }

// --- Timer Implementation ---
Timer::Timer(bool isRunning) : duration(0), isRunning(isRunning) {}

void Timer::setTimer() {
    startTime = std::chrono::steady_clock::now();
    isRunning = true;
}

void Timer::clearTimer() {
    isRunning = false;
}

bool Timer::checkTimer() {
    if (!isRunning) return false;
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime).count();
    if (elapsed >= (long long)duration) {
        isRunning = false;
        return true;
    }
    return false;
}

void Timer::setDuration(unsigned long ms) { 
    duration = ms; 
}

// --- ObstacleSensorInterface Implementation ---
ObstacleSensorInterface::ObstacleSensorInterface(FrontObstacleSensor* front, SideObstacleSensor* left, SideObstacleSensor* right)
    : frontSensor(front), leftSensor(left), rightSensor(right) {}

void ObstacleSensorInterface::hardwareISR() {
    if (onEmergencyCallback) onEmergencyCallback();
}

bool ObstacleSensorInterface::isFrontBlocked() {
    return frontSensor->isBlocked();
}

bool ObstacleSensorInterface::isLeftBlocked() {
    return leftSensor->isLeftBlocked();
}

bool ObstacleSensorInterface::isRigntBlocked() {
    return rightSensor->isRightBlocked();
}

ObstacleStatus ObstacleSensorInterface::isObstacleExist() {
    return {isFrontBlocked(), isLeftBlocked(), isRigntBlocked()};
}

void ObstacleSensorInterface::attachInterrupt(std::function<void()> callback) {
    onEmergencyCallback = callback;
}

// --- DustSensorInterface Implementation ---
DustSensorInterface::DustSensorInterface(DustSensor* dustSensor) : dustSensor(dustSensor) {}

bool DustSensorInterface::isDustExistence() {
    return dustSensor->isDustExist();
}

// --- PathPlanner Implementation ---
PathPlanner::PathPlanner(ObstacleSensorInterface* osi) : obstacleSensorInterface(osi) {}

void PathPlanner::decisionPath() {
    currentObstacleStatus = obstacleSensorInterface->isObstacleExist();
}

Driving PathPlanner::getSelectedDriving() {
    if (currentObstacleStatus.frontBlocked) {
        if (!currentObstacleStatus.leftBlocked) return Driving::TURNLEFT;
        if (!currentObstacleStatus.rightBlocked) return Driving::TURNRIGHT;
        return Driving::MOVEBACKWARD;
    }
    return Driving::MOVEFORWARD;
}

// --- DriveManager Implementation ---
DriveManager::DriveManager(Motor* left, Motor* right, PathPlanner* pathPlanner, Driving initialState)
    : leftMotor(left), rightMotor(right), pathPlanner(pathPlanner), currentDriveState(initialState), controller(nullptr) {}

void DriveManager::avoidObstacle() {
    pathPlanner->decisionPath();
    currentDriveState = pathPlanner->getSelectedDriving();
    
    if (currentDriveState == Driving::TURNLEFT) {
        leftMotor->rotateLeft();
        rightMotor->rotateLeft();
        timer.setDuration(100);
        timer.setTimer();
    } else if (currentDriveState == Driving::TURNRIGHT) {
        leftMotor->rotateRight();
        rightMotor->rotateRight();
        timer.setDuration(100);
        timer.setTimer();
    } else if (currentDriveState == Driving::MOVEBACKWARD) {
        leftMotor->rotateBackward();
        rightMotor->rotateBackward();
        timer.setDuration(200);
        timer.setTimer();
    } else {
        leftMotor->rotateForward();
        rightMotor->rotateForward();
    }
}

void DriveManager::resumeLeft() {
    std::cout << "DriveManager: Resume Left\n";
    currentDriveState = Driving::MOVEFORWARD;
    leftMotor->rotateForward();
    rightMotor->rotateForward();
}

void DriveManager::resumeRight() {
    std::cout << "DriveManager: Resume Right\n";
    currentDriveState = Driving::MOVEFORWARD;
    leftMotor->rotateForward();
    rightMotor->rotateForward();
}

void DriveManager::setController(Controller* ctrl) { 
    controller = ctrl; 
}

Driving DriveManager::getCurrentState() const { 
    return currentDriveState; 
}

// --- CleanerManager Implementation ---
void CleanerManager::cleanerMode(CleanerMode initialMode) {
    currentCleanerMode = initialMode;
    if (initialMode == CleanerMode::UP) {
        timer.setDuration(300);
        timer.setTimer();
    }
}

CleanerMode CleanerManager::getCurrentMode() const { 
    return currentCleanerMode; 
}

bool CleanerManager::isBoostPeriodOver() { 
    return timer.checkTimer(); 
}

// --- Controller Implementation ---
Controller::Controller(DriveManager* drive, CleanerManager* cleaner, DustSensorInterface* dsi)
    : driveManager(drive), cleanerManager(cleaner), dustSensorInterface(dsi), obstacleSensorInterface(nullptr) {
    if (driveManager) driveManager->setController(this);
}

void Controller::interruptHandler() {
    if (obstacleSensorInterface && (obstacleSensorInterface->isFrontBlocked() || 
        obstacleSensorInterface->isLeftBlocked() || obstacleSensorInterface->isRigntBlocked())) {
        std::cout << "Controller: Interrupt Detected!\n";
        driveManager->avoidObstacle();
    }

    if (cleanerManager->getCurrentMode() == CleanerMode::ON && dustSensorInterface->isDustExistence()) {
        std::cout << "Controller: Dust Detected!\n";
        cleanerManager->cleanerMode(CleanerMode::UP);
    }

    if (cleanerManager->isBoostPeriodOver()) {
        std::cout << "Controller: Boost Finished\n";
        cleanerManager->cleanerMode(CleanerMode::ON);
    }
}

void Controller::errorturnOff() {
    std::cout << "Controller: Error Turn Off\n";
    turnOff();
}

void Controller::trunOn() {
    std::cout << "Controller: System ON\n";
    cleanerManager->cleanerMode(CleanerMode::ON);
    driveManager->resumeLeft(); // Initial move
}

void Controller::turnOff() {
    std::cout << "Controller: System OFF\n";
    cleanerManager->cleanerMode(CleanerMode::OFF);
}

void Controller::turnRightOver() {
    std::cout << "Controller: Turn Right Over\n";
    resumeSideSensorCheck();
}

void Controller::turnLeftOver() {
    std::cout << "Controller: Turn Left Over\n";
    resumeSideSensorCheck();
}

void Controller::resumeSideSensorCheck() {
    if (obstacleSensorInterface && !obstacleSensorInterface->isFrontBlocked()) {
        driveManager->resumeLeft();
    } else {
        driveManager->avoidObstacle();
    }
}

void Controller::setObstacleSensorInterface(ObstacleSensorInterface* osi) { 
    obstacleSensorInterface = osi; 
}

// --- Button Implementation ---
Button::Button(Controller* ctrl) : controller(ctrl) {}

void Button::pushButtonOn() {
    std::cout << "Button: ON\n";
    if (controller) controller->trunOn();
}

void Button::pushButtonOff() {
    std::cout << "Button: OFF\n";
    if (controller) controller->turnOff();
}
