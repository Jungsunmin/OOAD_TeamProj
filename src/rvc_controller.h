#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <functional>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include <sys/time.h>


// --- Forward Declarations ---
class ObstacleSensorInterface;
class DustSensorInterface;
class PathPlanner;
class DriveManager;
class CleanerManager;
class Controller;

// --- Enums & Structs ---

enum class Location { FRONT, LEFT, RIGHT, REAR };
enum class Driving { MOVEFORWARD, TURNLEFT, TURNRIGHT, MOVEBACKWARD, STOP };
enum class CleanerMode { OFF, ON, UP };

struct ObstacleStatus {
    bool frontBlocked;
    bool leftBlocked;
    bool rightBlocked;
};

// --- Inㅇterfaces & Components ---

class Timer {
private:
    unsigned long duration;
public:
    Timer(unsigned long ms = 0);
    void setTimer();
    void setAlarmTimer();
    void removeTimer();
};

class ObstacleSensorInterface {
private:
    std::function<void()> onEmergency = nullptr;
    std::thread listenerThread;
    std::atomic<bool> running;
    Controller* ctrl_ref = nullptr; // Reference to trigger other events
    
    std::mutex sensorMutex;
    int last_front = 127;
    int last_left = 127;
    int last_right = 127;
    int last_dust = 0;
    const int threshold = 20;
    const int thresholdside = 60;

    void listen();
    void updateSensors();

public:
    ObstacleSensorInterface();
    ~ObstacleSensorInterface();
    
    // Blocking/Event Registration
    void attachInterrupt(std::function<void()> cb);
    void setController(Controller* c) { ctrl_ref = c; }
    
    // On-demand Polling
    bool isFrontBlocked();
    bool isLeftBlocked();
    bool isRightBlocked();
    ObstacleStatus isObstacleExist();
    bool isDustExistence();
};

class DustSensorInterface {
public:
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
    Timer turnTimer;
public:
    DriveManager(PathPlanner* pp);
    Location avoidObstacle();
    void rotateForward();
    void rotateLeft();
    void rotateRight();
    void rotateBackward();
    void stopMotor();
    void sendDriveCommand(Driving state);
    void rotateLeftb();
    void rotateRightb();
    Driving getCurrentState() const { return currentDriveState; }
};

class CleanerManager {
private:
    CleanerMode currentMode;
    Timer boostTimer;
public:
    CleanerManager();
    void cleanerMode(CleanerMode mode);
    bool iscleanerOn();
    bool isBoosterOn(); //
    void sendCleanerCommand(CleanerMode mode);
    CleanerMode getCurrentMode() const { return currentMode; }
};

class Controller {
private:
    DriveManager* driveManager;
    CleanerManager* cleanerManager;
    DustSensorInterface* dustSensorInterface;
    ObstacleSensorInterface* obstacleSensorInterface;

    std::atomic<bool> onOff{false};
    bool isAvoiding = false;
    std::atomic<bool> isAlarmSigExist = false;
    std::thread dustThread;
    std::thread obstacleThread;
    std::mutex ctrlMutex;
    std::mutex timerMutex;
    void boosterOverHandler();

public:
    Controller(DriveManager* d, CleanerManager* c, DustSensorInterface* ds, ObstacleSensorInterface* os);
    ~Controller();

    void interruptHandler(); // The Callback Function
    void turnOn(); 
    void turnOff();
    void errorturnOff();
    void dustDetect();
    void avoidanceLoop();
};

class ButtonInterface {
private:
    Controller* controller;
public:
    ButtonInterface(Controller* ctrl);
    void pushButtonOn();
    void pushButtonOff();
};