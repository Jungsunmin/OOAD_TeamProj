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

// --- Interfaces & Components ---

class Timer {
private:
    unsigned long duration;
public:
    Timer(unsigned long ms = 0);
    virtual ~Timer() = default;
    virtual void setTimer();
    virtual void setAlarmTimer();
    virtual void removeTimer();
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
    virtual ~ObstacleSensorInterface();
    
    // Blocking/Event Registration
    virtual void attachInterrupt(std::function<void()> cb);
    virtual void setController(Controller* c) { ctrl_ref = c; }
    
    // On-demand Polling
    virtual bool isFrontBlocked();
    virtual bool isLeftBlocked();
    virtual bool isRightBlocked();
    virtual ObstacleStatus isObstacleExist();
    virtual bool isDustExistence();
};

class DustSensorInterface {
public:
    virtual ~DustSensorInterface() = default;
    virtual bool isDustExistence();
};

class PathPlanner {
private:
    ObstacleSensorInterface* osi;
public:
    PathPlanner(ObstacleSensorInterface* osi);
    virtual ~PathPlanner() = default;
    virtual Location decisionPath();
};

class DriveManager {
private:
    PathPlanner* pathPlanner;
    Driving currentDriveState;
    Timer turnTimer;
public:
    DriveManager(PathPlanner* pp);
    virtual ~DriveManager() = default;
    virtual Location avoidObstacle();
    virtual void rotateForward();
    virtual void rotateLeft();
    virtual void rotateRight();
    virtual void rotateBackward();
    virtual void stopMotor();
    virtual void sendDriveCommand(Driving state);
    virtual void rotateLeftb();
    virtual void rotateRightb();
    Driving getCurrentState() const { return currentDriveState; }
};

class CleanerManager {
private:
    CleanerMode currentMode;
    Timer boostTimer;
public:
    CleanerManager();
    virtual ~CleanerManager() = default;
    virtual void cleanerMode(CleanerMode mode);
    virtual bool iscleanerOn();
    virtual bool isBoosterOn(); //
    virtual void sendCleanerCommand(CleanerMode mode);
    CleanerMode getCurrentMode() const { return currentMode; }
};

class Controller {
    friend class ControllerTest;
protected:
    DriveManager* driveManager;
    CleanerManager* cleanerManager;
    DustSensorInterface* dustSensorInterface;
    ObstacleSensorInterface* obstacleSensorInterface;

    std::atomic<bool> onOff{false};
    bool isAvoiding = false;
    std::atomic<bool> isAlarmSigExist = false;
    std::atomic<bool> frontObstacleTriggered{false};
    std::thread dustThread;
    void boosterOverHandler();

public:
    Controller(DriveManager* d, CleanerManager* c, DustSensorInterface* ds, ObstacleSensorInterface* os);
    virtual ~Controller();

    virtual void interruptHandler(); // The Callback Function
    virtual void turnOn(); 
    virtual void turnOff();
    virtual void errorturnOff();
    virtual void dustDetect();
    virtual void avoidanceAction();
};

class ButtonInterface {
private:
    Controller* controller;
public:
    ButtonInterface(Controller* ctrl);
    virtual ~ButtonInterface() = default;
    virtual void pushButtonOn();
    virtual void pushButtonOff();
};