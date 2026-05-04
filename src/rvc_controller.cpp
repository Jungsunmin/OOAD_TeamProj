#include "rvc_controller.h"

// // --- Motor Implementation ---
// void Motor::rotateForward() { std::cout << "Motor: Forward\n"; }
// void Motor::rotateBackward() { std::cout << "Motor: Backward\n"; }
// void Motor::rotateLeft() { std::cout << "Motor: Left\n"; }
// void Motor::rotateRight() { std::cout << "Motor: Right\n"; }
// void Motor::stopMotor() { std::cout << "Motor: Stop\n"; }

// // --- Sensor Implementations ---
// void FrontObstacleSensor::setBlocked(bool b) { blocked = b; }
// bool FrontObstacleSensor::isBlocked() const { return blocked; }

// void SideObstacleSensor::setStatus(bool l, bool r) { leftBlocked = l; rightBlocked = r; }
// bool SideObstacleSensor::isLeftBlocked() const { return leftBlocked; }
// bool SideObstacleSensor::isRightBlocked() const { return rightBlocked; }

// void DustSensor::setDust(bool d) { dustDetected = d; }
// bool DustSensor::isDustExist() const { return dustDetected; }

// --- Timer Implementation ---
Timer::Timer(unsigned long duration, bool isRunning = false, std::function<void()> timerCallback)
 : duration(duration), isRunning(isRunning), timerCallback(timerCallback) {}

void Timer::setTimer() {
    isRunning = true;
    usleep(duration);
    if(isRunning == false) return;   //timerOff()가 실행될 경우 실행됨
    isRunning = false;
    timerCallback();
    return;
}

void Timer::timerOff() {
    isRunning = false;
    return;
}

// // --- ObstacleSensorInterface Implementation ---
// ObstacleSensorInterface::ObstacleSensorInterface(FrontObstacleSensor* front, SideObstacleSensor* left, SideObstacleSensor* right)
//     : frontSensor(front), leftSensor(left), rightSensor(right) {}

// void ObstacleSensorInterface::hardwareISR() {
//     if (onEmergencyCallback) onEmergencyCallback();
// }

// bool ObstacleSensorInterface::isFrontBlocked() {
//     return frontSensor->isBlocked();
// }

// bool ObstacleSensorInterface::isLeftBlocked() {
//     return leftSensor->isLeftBlocked();
// }

// bool ObstacleSensorInterface::isRigntBlocked() {
//     return rightSensor->isRightBlocked();
// }

// ObstacleStatus ObstacleSensorInterface::isObstacleExist() {
//     return {isFrontBlocked(), isLeftBlocked(), isRigntBlocked()};
// }

// void ObstacleSensorInterface::attachInterrupt(std::function<void()> callback) {
//     onEmergencyCallback = callback;
// }

// // --- DustSensorInterface Implementation ---
// DustSensorInterface::DustSensorInterface(DustSensor* dustSensor) : dustSensor(dustSensor) {}

// bool DustSensorInterface::isDustExistence() {
//     return dustSensor->isDustExist();
// }

// // --- PathPlanner Implementation ---
// PathPlanner::PathPlanner(ObstacleSensorInterface* osi) : obstacleSensorInterface(osi) {}

// Location PathPlanner::decisionPath() {
//     currentObstacleStatus = obstacleSensorInterface->isObstacleExist();
// }

// Driving PathPlanner::getSelectedDriving() {
//     if (currentObstacleStatus.frontBlocked) {
//         if (!currentObstacleStatus.leftBlocked) return Driving::TURNLEFT;
//         if (!currentObstacleStatus.rightBlocked) return Driving::TURNRIGHT;
//         return Driving::MOVEBACKWARD;
//     }
//     return Driving::MOVEFORWARD;
// }

// --- DriveManager Implementation ---
DriveManager::DriveManager(PathPlanner* pathPlanner, Driving initialState, Controller* controller)
    : pathPlanner(pathPlanner), currentDriveState(initialState), controller(controller) {}

Location DriveManager::avoidObstacle() {
    stopMotor();    //
    location = pathPlanner->decisionPath();
    
    if (location == Location::LEFT) {
        rotateLeft();   //
        timer.setTimer();
        stopMotor();    //
        rotateForward();    //
        return Location::LEFT;
    } else if (location == Location::RIGHT) {
        rotateRight();   //
        timer.setTimer();
        stopMotor();    //
        rotateForward();    //
        return Location::RIGHT;
    } else if (location == Location::REAR) {
        rotateBackward();   //
        return Location::REAR;
    } 
    return Location::FRONT;
}

void DriveManager::resumeForward() {
    rotateForward(); //
    return;
}

void DriveManager::resumeLeft() {
    stopMotor(); //
    rotateLeft();   //
    timer.setTimer();
    stopMotor();    //
    rotateForward();
    return;
}

void DriveManager::resumeRight() {
    stopMotor(); //
    rotateRight();   //
    timer.setTimer();
    stopMotor();    //
    rotateForward();
    return;
}

void DriveManager::stop() {
    stopMotor(); //
    return;
}


// // --- CleanerManager Implementation ---
// void CleanerManager::cleanerMode(CleanerMode initialMode) {
//     currentCleanerMode = initialMode;
//     if (initialMode == CleanerMode::UP) {
//         timer.setDuration(300);
//         timer.setTimer();
//     }
// }

// CleanerMode CleanerManager::getCurrentMode() const { 
//     return currentCleanerMode; 
// }

// bool CleanerManager::isBoostPeriodOver() { 
//     return timer.checkTimer(); 
// }

// --- Controller Implementation ---
Controller::Controller(DriveManager* drive, CleanerManager* cleaner, DustSensorInterface* dsi, ObstacleSensorInterface* osi)
    : driveManager(drive), cleanerManager(cleaner), dustSensorInterface(dsi), obstacleSensorInterface(osi) {}

void Controller::interruptHandler() {
    cleanerManager->cleanerMode(CleanerMode::OFF);
    Location turnLocation = driveManager->avoidObstacle();
    if(turnLocation == Location::LEFT) {
        if(!obstacleSensorInterface->isRightBlocked()) {
            errorturnOff();
        }
    } else if (turnLocation == Location::RIGHT) {
        if(!obstacleSensorInterface->isLeftBlocked()) {
            errorturnOff();
        }
    } else if (turnLocation == Location::REAR) {
        resumeSideSensorCheck();
    }
    cleanerManager->cleanerMode(CleanerMode::ON);
    clearDustValue();
}
void Controller::turnOn() {
    onOff = true;
    dustSensorInterface->isDustExistence();
    obstacleSensorInterface->isObstacleExist();
    attachInterrupt(interruptHandler);  //

    cleanerManager->cleanerMode(CleanerMode::ON);
    driveManager->resumeForward();
}



void Controller::turnOff() {
    onOff = false;
    cleanerManager->cleanerMode(CleanerMode::OFF);
    driveManager->stop();
}

void Controller::errorturnOff() {
    std::cout << "Controller: Error Turn Off\n";
    turnOff();
}

void Controller::clearDustValue() {
    clearValue = true;
}

// void Controller::turnOn() {
//     std::cout << "Controller: System ON\n";
//     cleanerManager->cleanerMode(CleanerMode::ON);
//     driveManager->resumeLeft(); // Initial move
// }

void Controller::resumeSideSensorCheck() {
    while(obstacleSensorInterface->isLeftBlocked() && obstacleSensorInterface->isRightBlocked()) {
        usleep(100000);
        continue;
    }
    if (!obstacleSensorInterface->isLeftBlocked()) {
        driveManager->resumeLeft();
    } else {
        driveManager->resumeRight();
    }
}

void Controller::dustDetect() {
    while(true) {
        usleep(100000);
        if(dustSensorInterface->isDustExistence()) {
            cleanerManager->cleanerMode(CleanerMode::UP);
        }
        if(clearValue) {
            cleanerManager->cleanerMode(CleanerMode::ON);
            clearValue = false;
        }
        if(!onOff) break;
    }
}
// --- Button Implementation ---
Button::Button(Controller* ctrl) : controller(ctrl) {}

void Button::pushButtonOn() {
    std::cout << "Button: ON\n";
    if (controller) controller->turnOn();
}

void Button::pushButtonOff() {
    std::cout << "Button: OFF\n";
    if (controller) controller->turnOff();
}
