#ifndef RVC_SW_CONTROLLER_UTEST_MOCK_H
#define RVC_SW_CONTROLLER_UTEST_MOCK_H
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <rvc_controller.h>

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
    MOCK_METHOD(void, rotateLeftb, (), (override));
    MOCK_METHOD(void, rotateRightb, (), (override));
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
    MOCK_METHOD(bool, isBoosterOn, (), (override));
    //시뮬레이터 변경후 생긴 문제
    //MOCK_METHOD(void, sendCleanerCommand, (CleanerMode), (override));
    ///삭제 - 없어진 함수
};

class MockObstacleSensorInterface : public ObstacleSensorInterface {
public:
    //시뮬레이터 변경후 생긴 문제
    //MOCK_METHOD(void, attachInterrupt, (std::function<void()>), (override));
    ///삭제 - 없어진 함수
    MOCK_METHOD(void, setController, (Controller*), (override));
    MOCK_METHOD(bool, isFrontBlocked, (), (override));
    MOCK_METHOD(bool, isLeftBlocked, (), (override));
    MOCK_METHOD(bool, isRightBlocked, (), (override));
    ////삭제된 함수와 구조체 자료형
    // MOCK_METHOD(ObstacleStatus, isObstacleExist, (), (override));
    MOCK_METHOD(bool, isDustExistence, (), (override));
};

//삭제
// class MockDustSensorInterface : public DustSensorInterface {
// public:
//     MOCK_METHOD(bool, isDustExistence, (), (override));
// };

class MockController : public Controller {
public:
    //시뮬레이터 변경후 생긴 문제
    MockController(DriveManager* d, CleanerManager* c, ObstacleSensorInterface* os)
        : Controller(d, c, os) { }
    ///생성자 변경: 인자 4개 -> 3개 (먼지 센서 인터페이스 삭제)
    MOCK_METHOD(void, turnOn, (), (override));
    MOCK_METHOD(void, turnOff, (), (override));
    MOCK_METHOD(void, errorturnOff, (), (override));
    MOCK_METHOD(void, dustDetect, (), (override));
    MOCK_METHOD(void, avoidanceAction, (), (override));
};

#endif //RVC_SW_CONTROLLER_UTEST_MOCK_H
