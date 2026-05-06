#ifndef RVC_SW_CONTROLLER_UTEST_MOCK_H
#define RVC_SW_CONTROLLER_UTEST_MOCK_H
#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <condition_variable>
#include <mutex>
#include <future>

class MockDriveManager : public DriveManager {
public:
    MockDriveManager(PathPlanner* pp) : DriveManager(pp) {}
    MOCK_METHOD(Location, avoidObstacle, (), (override));
    MOCK_METHOD(void, rotateForward, (), (override));
    MOCK_METHOD(void, rotateLeft, (), (override));
    MOCK_METHOD(void, rotateRight, (), (override));
    MOCK_METHOD(void, rotateBackward, (), (override));
    MOCK_METHOD(void, stopMotor, (), (override));
};

class MockPathPlanner : public PathPlanner {
public:
    MockPathPlanner(ObstacleSensorInterface* osi) : PathPlanner(osi) {}
    MOCK_METHOD(Location, decisionPath, (), (override));
};

class MockCleanerManager : public CleanerManager {
public:
    MOCK_METHOD(void, cleanerMode, (CleanerMode), (override));
    MOCK_METHOD(bool, iscleanerOn, (), (override));
    MOCK_METHOD(void, sendCleanerCommand, (CleanerMode), (override));
};

class MockObstacleSensorInterface : public ObstacleSensorInterface {
public:
    MOCK_METHOD(void, attachInterrupt, (std::function<void()>), (override));
    MOCK_METHOD(void, setController, (Controller*), (override));
    MOCK_METHOD(bool, isFrontBlocked, (), (override));
    MOCK_METHOD(bool, isLeftBlocked, (), (override));
    MOCK_METHOD(bool, isRightBlocked, (), (override));
    MOCK_METHOD(ObstacleStatus, isObstacleExist, (), (override));
    MOCK_METHOD(bool, isDustExistence, (), (override));
};

class MockDustSensorInterface : public DustSensorInterface {
public:
    MOCK_METHOD(bool, isDustExistence, (), (override));
};

class MockController : public Controller {
public:
    MockController(DriveManager* d, CleanerManager* c, DustSensorInterface* ds, ObstacleSensorInterface* os) 
        : Controller(d, c, ds, os) { }
    MOCK_METHOD(void, turnOn, (), (override));
    MOCK_METHOD(void, turnOff, (), (override));
    MOCK_METHOD(void, errorturnOff, (), (override));
    MOCK_METHOD(void, dustDetect, (), (override));
    MOCK_METHOD(void, avoidanceAction, (), (override));
};

#endif //RVC_SW_CONTROLLER_UTEST_MOCK_H
