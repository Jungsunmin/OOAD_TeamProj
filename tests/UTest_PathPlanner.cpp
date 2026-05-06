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

// 2. 왼쪽이 막히고 오른쪽이 비어있을 때 오른쪽으로 결정을 내리는지 확인
TEST_F(PathPlannerTest, decisionPath_TurnRight) {
    // Expectation: 왼쪽은 막히고, 오른쪽은 뚫림
    EXPECT_CALL(*mockOSI, isLeftBlocked()).WillOnce(Return(true));
    EXPECT_CALL(*mockOSI, isRightBlocked()).WillOnce(Return(false));
    
    EXPECT_EQ(pathPlanner->decisionPath(), Location::RIGHT);
}

// 3. 양쪽이 모두 막혔을 때 뒤로 결정을 내리는지 확인
TEST_F(PathPlannerTest, decisionPath_MoveBackward) {
    // Expectation: 양쪽 모두 막힘
    EXPECT_CALL(*mockOSI, isLeftBlocked()).WillOnce(Return(true));
    EXPECT_CALL(*mockOSI, isRightBlocked()).WillOnce(Return(true));
    
    EXPECT_EQ(pathPlanner->decisionPath(), Location::REAR);
}
