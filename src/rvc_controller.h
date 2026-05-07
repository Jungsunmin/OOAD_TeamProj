#pragma once
#include "common_types.h"
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
class SensorInterface;
class DustSensorInterface;
class PathPlanner;
class DriveManager;
class CleanerManager;
class Controller;


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

class SensorInterface {
private:
    std::function<void()> onEmergency = nullptr;
    Controller* ctrl_ref = nullptr; // Reference to trigger other events
    
    const int threshold = 20;
    const int thresholdside = 60;

    void updateSensors();

public:
    SensorInterface();
    virtual ~SensorInterface();
    
    // Blocking/Event Registration
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
    SensorInterface* osi;
public:
    PathPlanner(SensorInterface* osi);
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
    //virtual void sendDriveCommand(Driving state);
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
    //virtual void sendCleanerCommand(CleanerMode mode);
    CleanerMode getCurrentMode() const { return currentMode; }
};

class Controller {
    friend class ControllerTest;
protected:
    DriveManager* driveManager;
    CleanerManager* cleanerManager;
    DustSensorInterface* dustSensorInterface;
    SensorInterface* sensorInterface;

    std::atomic<bool> onOff{false};
    bool isAvoiding = false;
    std::atomic<bool> isAlarmSigExist = false;
    std::atomic<bool> frontObstacleTriggered{false};
    std::thread dustThread;
    void boosterOverHandler();

public:
    Controller(DriveManager* d, CleanerManager* c, SensorInterface* os);
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