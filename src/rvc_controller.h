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
    std::chrono::steady_clock::time_point startTime;

public:
    Timer(bool isRunning = false);
    void setTimer();
    void clearTimer();
    bool checkTimer();
    void setDuration(unsigned long ms);
};

// This is the "Window" to the external Simulator
class ObstacleSensorInterface {
private:
    int client_fd;
    const int OBSTACLE_DISTANCE_THRESHOLD = 10; // Value <= 10 is BLOCKED
    std::function<void()> onEmergencyCallback = nullptr;
    std::thread listenerThread;
    std::atomic<bool> running;

    void listenToSimulator();

public:
    ObstacleSensorInterface();
    ~ObstacleSensorInterface();
    
    // Boundary methods that talk to Simulator
    bool isFrontBlocked();
    bool isLeftBlocked();
    bool isRightBlocked(); 
    ObstacleStatus isObstacleExist();
    
    // The "Interrupt" registration
    void attachInterrupt(std::function<void()> callback);
};

class DustSensorInterface {
private:
    int client_fd;
    const int DUST_EXISTENCE_THRESHOLD = 60; // Value >= 60 is DUSTY

public:
    DustSensorInterface();
    bool isDustExistence();
};

class PathPlanner {
private:
    ObstacleSensorInterface* obstacleSensorInterface;

public:
    PathPlanner(ObstacleSensorInterface* osi);
    Location decisionPath();
};

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

public:
    Controller(DriveManager* drive, CleanerManager* cleaner, DustSensorInterface* dustSensor, ObstacleSensorInterface* obstacleSensor);
    void interruptHandler();
    void errorturnOff();
    void turnOn(); 
    void turnOff();
    void turnRightOver();
    void turnLeftOver();
    void resumeSideSensorCheck();
};
