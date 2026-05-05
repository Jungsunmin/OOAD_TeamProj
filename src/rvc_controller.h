#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <functional>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>

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

// --- Boundary Interfaces ---

class Timer {
private:
    unsigned long duration;

public:
    Timer(unsigned long ms = 0);
    void setTimer();
};

class ObstacleSensorInterface {
private:
    const int THRESHOLD = 10;
    std::function<void()> onEmergency = nullptr;

public:
    ObstacleSensorInterface();
    bool isFrontBlocked();
    bool isLeftBlocked();
    bool isRightBlocked(); 
    ObstacleStatus isObstacleExist();
    void attachInterrupt(std::function<void()> cb);
    void triggerInterrupt();
};

class DustSensorInterface {
private:
    const int DUST_EXISTENCE_THRESHOLD = 60;
public:
    DustSensorInterface();
    bool isDustExistence();
};

class PathPlanner {
private:
    ObstacleSensorInterface* osi;
public:
    PathPlanner(ObstacleSensorInterface* osi);
    Location decisionPath();
};

class DriveManager {
private:
    PathPlanner* pathPlanner;
    Driving currentDriveState;
    Timer timer;
public:
    DriveManager(PathPlanner* pp);
    Location avoidObstacle();
    void rotateForward();
    void rotateLeft();
    void rotateRight();
    void stopMotor();
    void rotateBackward();
    void sendDriveCommand(Driving state);
    Driving getCurrentState() const { return currentDriveState; }
};

class CleanerManager {
private:
    CleanerMode currentMode;
    Timer timer;
public:
    CleanerManager();
    void cleanerMode(CleanerMode mode);
    void sendCleanerCommand(CleanerMode mode);
    void updateTimer();
    CleanerMode getCurrentMode() const { return currentMode; }
};

class Controller {
private:
    DriveManager* driveManager;
    CleanerManager* cleanerManager;
    DustSensorInterface* dustSensorInterface;
    ObstacleSensorInterface* obstacleSensorInterface;

    bool onOff = false;
    bool clearValue = false;
    std::thread dustThread;

    void errorturnOff();
    void clearDustValue();
    void resumeSideSensorCheck();
    void dustDetect();
public:
    Controller(DriveManager* drive, CleanerManager* cleaner, DustSensorInterface* dsi, ObstacleSensorInterface* osi);
    ~Controller();
    void interruptHandler();
    
    void turnOn(); 
    void turnOff();

    ObstacleSensorInterface* getObstacleSensorInterface() { return obstacleSensorInterface; }
};

class ButtonInterface {
private:
    Controller* controller;
    std::thread listener;
    std::atomic<bool> running;
    void listen();

public:
    ButtonInterface(Controller* ctrl);
    ~ButtonInterface();
    void pushButtonOn();
    void pushButtonOff();
};
