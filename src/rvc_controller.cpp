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

void ObstacleSensorInterface::attachInterrupt(std::function<void()> callback) {
    onEmergencyCallback = callback;
}

// --- DustSensorInterface Implementation ---
bool DustSensorInterface::isDustExistence() {
    return dustSensor->getSensorValue()>= DUST_EXISTENCE_THRESHOLD;
}

// --- PathPlanner Implementation ---
Location PathPlanner::decisionPath() {
    currentObstacleStatus = obstacleSensorInterface->isObstacleExist();
    if (!currentObstacleStatus.leftBlocked) return Location::LEFT;
    else if (!currentObstacleStatus.rightBlocked) return Location::RIGHT;
    else return Location::REAR;
}


// --- CleanerManager Implementation ---
void CleanerManager::cleanerMode(CleanerMode initialMode) {
    currentCleanerMode = initialMode;
    if (initialMode == CleanerMode::UP) {
        cleaner.powerOn();
        timer.setTimer();
    }
    else if (initialMode == CleanerMode::ON) cleaner.cleanerOn();
    else cleaner.cleanrOff();
    

}


