#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "rvc_controller.h"
#include "UTest_Mock.h"

using namespace testing;

class PathPlannerTest : public ::testing::Test {
protected:
    NiceMock<MockObstacleSensorInterface>* mockOSI;
    PathPlanner* pathPlanner;

    void SetUp() override {
        mockOSI = new NiceMock<MockObstacleSensorInterface>();
        pathPlanner = new PathPlanner(mockOSI);
    }
    void TearDown() override {
        delete pathPlanner;
        delete mockOSI;
    }
};

// 1. 왼쪽이 비어있을 때 왼쪽으로 결정을 내리는지 확인
TEST_F(PathPlannerTest, decisionPath_TurnLeft) {
    // Expectation: isLeftBlocked() 호출 시 false 리턴
    EXPECT_CALL(*mockOSI, isLeftBlocked()).WillOnce(Return(false));
    
    EXPECT_EQ(pathPlanner->decisionPath(), Location::LEFT);
}

// 2. 왼쪽이 막히고 오른쪽이 비어있을 때 회전탐색을 시작하는지 확인
TEST_F(PathPlannerTest, decisionPath_Clockwise) {
    // Expectation: 왼쪽은 막히고, 오른쪽은 뚫림
    EXPECT_CALL(*mockOSI, isLeftBlocked()).WillOnce(Return(true));
    
    EXPECT_EQ(pathPlanner->decisionPath(), Location::Clockwise);
}

// 3. 회전 후 전방이 뚫려 있으면 직진하는지 확인
TEST_F(PathPlannerTest, Forward_After_Clockwise) {
    EXPECT_CALL(*mockOSI, isFrontBlocked()).WillOnce(Return(false));
 
    EXPECT_EQ(pathPlanner->decisionAfterClockwise(), Location::Forward);
}

// 4. 회전 후 전방이 막혀 있으면 다시 원래 방향으로 회전하는지 확인
TEST_F(PathPlannerTest, CounterClockwise_After_Clockwise) {
    EXPECT_CALL(*mockOSI, isFrontBlocked()).WillOnce(Return(true));
 
    EXPECT_EQ(pathPlanner->decisionAfterClockwise(), Location::CounterClockwise);
}
 