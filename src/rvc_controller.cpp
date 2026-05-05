#include "rvc_controller.h"


bool ObstacleSensorInterface::isFrontBlocked() {
    return frontSensor->getSensorValue() <= OBSTACLE_DISTANCE_THRESHOLD
}

bool ObstacleSensorInterface::isLeftBlocked() {
    return leftSensor->getSensorValue() <= OBSTACLE_DISTANCE_THRESHOLD
}
bool ObstacleSensorInterface::isRigntBlocked() {
    return rightSensor->getSensorValue() <= OBSTACLE_DISTANCE_THRESHOLD
}

ObstacleStatus ObstacleSensorInterface::isObstacleExist() {
    return {isFrontBlocked(), isLeftBlocked(), isRigntBlocked()};
}

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
// }/

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

