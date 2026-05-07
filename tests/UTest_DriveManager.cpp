#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "rvc_controller.h"
#include "UTest_Mock.h"
#include <chrono>

using namespace testing;

class DriveManagerTest : public ::testing::Test {
protected:
    NiceMock<MockObstacleSensorInterface>* mockOSI;
    NiceMock<MockPathPlanner>* mockPP;
    DriveManager* driveManager;

    void SetUp() override {
        mockOSI = new NiceMock<MockObstacleSensorInterface>();
        mockPP = new NiceMock<MockPathPlanner>(mockOSI);
        driveManager = new DriveManager(mockPP);
    }
    void TearDown() override {
        delete driveManager;
        delete mockPP;
        delete mockOSI;
    }
};

// 1. 기본 주행 상태 변경 테스트
TEST_F(DriveManagerTest, DrivingTest_rotateXXX_getCurrentState) {
    driveManager->rotateForward();
    EXPECT_EQ(driveManager->getCurrentState(), Driving::MOVEFORWARD);

    driveManager->rotateLeft();
    EXPECT_EQ(driveManager->getCurrentState(), Driving::TURNLEFT);

    driveManager->rotateRight();
    EXPECT_EQ(driveManager->getCurrentState(), Driving::TURNRIGHT);

    driveManager->rotateBackward();
    EXPECT_EQ(driveManager->getCurrentState(), Driving::MOVEBACKWARD);

    driveManager->stopMotor();
    EXPECT_EQ(driveManager->getCurrentState(), Driving::STOP);
}

// 2. 회피 로직 호출 및 상태 복구 테스트
TEST_F(DriveManagerTest, AvoidObstacle_LogicTest) {
    // PathPlanner가 왼쪽으로 가라고 명령할 것임을 설정
    EXPECT_CALL(*mockPP, decisionPath()).WillOnce(Return(Location::LEFT));
    
    Location result = driveManager->avoidObstacle();

    EXPECT_EQ(result, Location::LEFT);
    // 회피 후 다시 전진 상태로 복구되었는지 확인
    EXPECT_EQ(driveManager->getCurrentState(), Driving::MOVEFORWARD);
}

//2-2. 회피 로직 중 우회전 상태 테스트
TEST_F(DriveManagerTest, DriveManagerTest_AvoidObstacle_LogicTest_Right) {    // PathPlanner가 오른쪽으로 가라고 명령할 것임을 설정
    EXPECT_CALL(*mockPP, decisionPath()).WillOnce(Return(Location::RIGHT));
    Location result = driveManager->avoidObstacle();    EXPECT_EQ(result, Location::RIGHT);    // 회피 후 다시 전진 상태로 복구되었는지 확인
    EXPECT_EQ(driveManager->getCurrentState(), Driving::MOVEFORWARD);
}
// 3. 특수 회전 로직 테스트
TEST_F(DriveManagerTest, RotateRightb_ExecutionTest) {
    driveManager->rotateRightb();
    EXPECT_EQ(driveManager->getCurrentState(), Driving::MOVEFORWARD);
}

//3-2. 후진 후 회전. 좌회전 방향
TEST_F(DriveManagerTest, RotateLeftb_ExecutionTest) {
    driveManager->rotateLeftb();
    EXPECT_EQ(driveManager->getCurrentState(), Driving::MOVEFORWARD);
}