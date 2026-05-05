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
class Motor;
class FrontObstacleSensor;
class SideObstacleSensor;
class DustSensor;
class ObstacleSensorInterface;
class DustSensorInterface;
class PathPlanner;
class Timer;
class DriveManager;
class CleanerManager;
class Controller;

// --- Components ---

class Motor {
public:
    void rotateForward();
    void rotateBackward();
    void rotateLeft();
    void rotateRight();
    void stopMotor();
};

class FrontObstacleSensor {
    bool blocked = false;
public:
    void setBlocked(bool b);
    bool isBlocked() const;
};

class SideObstacleSensor {
    bool leftBlocked = false;
    bool rightBlocked = false;
public:
    void setStatus(bool l, bool r);
    bool isLeftBlocked() const;
    bool isRightBlocked() const;
};

class DustSensor {
    bool dustDetected = false;
public:
    void setDust(bool d);
    bool isDustExist() const;
};

// --- Class Diagram Specified Classes ---

class Timer {
private:
    unsigned long duration;
    bool isRunning;
    std::chrono::steady_clock::time_point startTime;

public:
    Timer(bool isRunning = false);
    void setTimer();
    void clearTimer();
    
    // Internal helpers
    bool checkTimer();
    void setDuration(unsigned long ms);
};

class ObstacleSensorInterface {
private:
    FrontObstacleSensor* frontSensor;
    SideObstacleSensor* leftSensor;
    SideObstacleSensor* rightSensor;
    const int OBSTACLE_DISTANCE_THRESHOLD = 10;
    std::function<void()> onEmergencyCallback = nullptr;

public:
    ObstacleSensorInterface(FrontObstacleSensor* front, SideObstacleSensor* left, SideObstacleSensor* right);
    bool isFrontBlocked();
    bool isLeftBlocked();
    bool isRigntBlocked(); 
    ObstacleStatus isObstacleExist();
    void attachInterrupt(std::function<void()> callback);
};

class DustSensorInterface {
private:
    DustSensor* dustSensor;
    const int DUST_EXISTENCE_THRESHOLD = 60;

public:
    DustSensorInterface(DustSensor* dustSensor);
    bool isDustExistence();
};

class PathPlanner {
private:
    ObstacleSensorInterface* obstacleSensorInterface;
    ObstacleStatus currentObstacleStatus;

public:
    PathPlanner(ObstacleSensorInterface* obstacleSensorInterface);
    Location decisionPath();
};

class DriveManager {
private:
    Motor* leftMotor;
    Motor* rightMotor;
    PathPlanner* pathPlanner;
    Driving currentDriveState;
    Controller* controller;
    Timer timer;

public:
    DriveManager(Motor* left, Motor* right, PathPlanner* pathPlanner, Driving initialState);
    void avoidObstacle();
    void resumeLeft();
    void resumeRight();
    
    // Internal helpers
    void setController(Controller* ctrl);
    Driving getCurrentState() const;
};

class CleanerManager {
private:
    CleanerMode currentCleanerMode;
    Timer timer;
    Cleaner cleaner;

public:
    void cleanerMode(CleanerMode initialMode);
};

class Controller {
private:
    DriveManager* driveManager;
    CleanerManager* cleanerManager;
    DustSensorInterface* dustSensorInterface;
    ObstacleSensorInterface* obstacleSensorInterface;

public:
    Controller(DriveManager* drive, CleanerManager* cleaner, DustSensorInterface* dustSensorInterface);
    void interruptHandler();
    void errorturnOff();
    void turnOn(); 
    void turnOff();
    void turnRightOver();
    void turnLeftOver();
    void resumeSideSensorCheck();
    
    // Internal helper
    void setObstacleSensorInterface(ObstacleSensorInterface* osi);
};


