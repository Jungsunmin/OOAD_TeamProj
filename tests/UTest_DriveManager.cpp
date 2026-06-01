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

// 3. 우회전 후 전방이 뚫려 있을 때 전진하는지 테스트
TEST_F(DriveManagerTest, AvoidObstacle_Forward_After_Clockwise) {
    EXPECT_CALL(*mockPP, decisionPath()).WillOnce(Return(Location::Clockwise));
    EXPECT_CALL(*mockOSI, isFrontBlocked()).WillOnce(Return(false));
 
    Location result = driveManager->avoidObstacle();
 
    EXPECT_EQ(result, Location::Forward);
    EXPECT_EQ(driveManager->getCurrentState(), Driving::MOVEFORWARD);
}

// 4. 우회전 후 전방이 막혀 있을 때 다시 좌회전하는지 테스트
TEST_F(DriveManagerTest, AvoidObstacle_Counterclockwise_After_Clockwise) {
    EXPECT_CALL(*mockPP, decisionPath())
        .WillOnce(Return(Location::Clockwise))
        .WillOnce(Return(Location::LEFT));
 
    EXPECT_CALL(*mockOSI, isFrontBlocked()).WillOnce(Return(true));
 
    Location result = driveManager->avoidObstacle();
 
    EXPECT_EQ(result, Location::LEFT);
    EXPECT_EQ(driveManager->getCurrentState(), Driving::MOVEFORWARD);
}

// 5. 회전 후 전진 상태로 복귀하는지 확인
TEST_F(DriveManagerTest, Forward_After_RotateLeft) {
    driveManager->rotateLeftb();
    EXPECT_EQ(driveManager->getCurrentState(), Driving::MOVEFORWARD);
}
TEST_F(DriveManagerTest, Forward_After_RotateRight) {
    driveManager->rotateRightb();
    EXPECT_EQ(driveManager->getCurrentState(), Driving::MOVEFORWARD);
}


 // 6. avoidObstacle: decisionPath 가 알 수 없는 값을 돌려주면 안전 정지
TEST_F(DriveManagerTest, AvoidObstacle_UnknownValue_StopsMotor) {
    // Forward 는 직접 반환되지 않는 값 → unknown 분기로 빠짐
    EXPECT_CALL(*mockPP, decisionPath()).WillOnce(Return(Location::Forward));
 
    Location result = driveManager->avoidObstacle();
 
    EXPECT_EQ(result, Location::Forward);
    EXPECT_EQ(driveManager->getCurrentState(), Driving::STOP);
}