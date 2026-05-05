#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <functional>
#include <chrono>
#include <thread>
#include <atomic>

// --- Forward Declarations ---
class ObstacleSensorInterface;
class DustSensorInterface;
class PathPlanner;
class Timer;
class DriveManager;
class CleanerManager;
class Controller;

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

// --- Class Diagram Specified Classes (Boundary Interfaces) ---

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
    Timer timer;
    int client_fd; // To send movement commands to Simulator

public:
    DriveManager(PathPlanner* pathPlanner, Driving initialState);
    void avoidObstacle();
    void resumeLeft();
    void resumeRight();
    void setController(Controller* ctrl);
    Driving getCurrentState() const;
    
    // Command Bridge to Simulator
    void sendDriveCommand(Driving state);
};

class CleanerManager {
private:
    CleanerMode currentCleanerMode;
    Timer timer;
    int client_fd; // To send cleaner commands to Simulator

public:
    CleanerManager();
    void cleanerMode(CleanerMode initialMode);
    void sendCleanerCommand(CleanerMode mode);
};

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
    Controller(DriveManager* drive, CleanerManager* cleaner, DustSensorInterface* dustSensor, ObstacleSensorInterface* obstacleSensor);
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
