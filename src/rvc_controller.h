#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <functional>
#include <chrono>

// --- Enums & Structs ---

enum class Location {
    FRONT,
    LEFT,
    RIGHT,
    REAR
};

enum class Driving {
    MOVEFORWARD,
    TURNLEFT,
    TURNRIGHT,
    MOVEBACKWARD,
    STOP
};

enum class CleanerMode {
    OFF,
    ON,
    UP
};

struct ObstacleStatus {
    bool frontBlocked;
    bool leftBlocked;
    bool rightBlocked;
};

// --- Forward Declarations ---
//class ObstacleSensorInterface;
//class DustSensorInterface;
//class PathPlanner;
class Timer;
class DriveManager;
//class CleanerManager;
class Controller;

// --- Components ---


class Timer {
private:
    unsigned long duration;
    bool isRunning;
    std::function<void()> timerCallback;

public:
    Timer(unsigned long duration, bool isRunning = false, std::function<void()> timerCallback);
    void setTimer();
    void timerOff();
};

// class ObstacleSensorInterface {
// private:
//     FrontObstacleSensor* frontSensor;
//     SideObstacleSensor* leftSensor;
//     SideObstacleSensor* rightSensor;
//     const int OBSTACLE_DISTANCE_THRESHOLD = 10;
//     std::function<void()> onEmergencyCallback;

// public:
//     ObstacleSensorInterface(FrontObstacleSensor* front, SideObstacleSensor* left, SideObstacleSensor* right);
//     void hardwareISR();
//     bool isFrontBlocked();
//     bool isLeftBlocked();
//     bool isRigntBlocked(); 
//     ObstacleStatus isObstacleExist();
//     void attachInterrupt(std::function<void()> callback);
// };

// class DustSensorInterface {
// private:
//     DustSensor* dustSensor;
//     const int DUST_EXISTENCE_THRESHOLD = 5;

// public:
//     DustSensorInterface(DustSensor* dustSensor);
//     bool isDustExistence();
// };

// class PathPlanner {
// private:
//     ObstacleSensorInterface* obstacleSensorInterface;
//     ObstacleStatus currentObstacleStatus;

// public:
//     PathPlanner(ObstacleSensorInterface* obstacleSensorInterface);
//     void decisionPath();
    
//     // Internal helper to get result
//     Driving getSelectedDriving();
// };

class DriveManager {
private:
    PathPlanner* pathPlanner;
    Driving currentDriveState;
    Controller* controller;
    Timer timer{ 3, false, [](){} };  //콜백 함수로 아무일도 안함
    Location location = Location::FRONT;

public:
    DriveManager(PathPlanner* pathPlanner, Driving initialState, controller* controller);
    Location avoidObstacle();
    void resumeForward();
    void resumeLeft();
    void resumeRight();
    void stop();
    
};

// class CleanerManager {
// private:
//     CleanerMode currentCleanerMode;
//     Timer timer;

// public:
//     void cleanerMode(CleanerMode initialMode);
    
//     // Internal helper
//     CleanerMode getCurrentMode() const;
//     bool isBoostPeriodOver();
// };

class Controller {
private:
    DriveManager* driveManager;
    CleanerManager* cleanerManager;
    DustSensorInterface* dustSensorInterface;
    ObstacleSensorInterface* obstacleSensorInterface;

    bool onOff = false;
    bool clearValue = false;

    void errorturnOff();
    void clearDustValue();
    void resumeSideSensorCheck();
    void dustDetect();
public:
    Controller(DriveManager* drive, CleanerManager* cleaner, DustSensorInterface* dustSensorInterface);
    void interruptHandler();
    
    void turnOn(); 
    void turnOff();
    
};

class ButtonInterface {
public:
    virtual ~ButtonInterface() = default;
    virtual void pushButtonOn() = 0;
    virtual void pushButtonOff() = 0;
};

// Concrete implementation of Button
class Button : public ButtonInterface {
private:
    Controller* controller;
public:
    Button(Controller* ctrl);
    void pushButtonOn() override;
    void pushButtonOff() override;
};
