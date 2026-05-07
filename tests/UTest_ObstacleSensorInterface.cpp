#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "rvc_controller.h"
#include "UTest_Mock.h"
#include <chrono>

using namespace testing;

class ObstacleSensorInterface : public ::testing::Test {
protected:

    ObstacleSensorInterface* ObstacleSensorInterface;

    void SetUp() override {
        obstacleSensorInterface = new ObstacleSensorInterface();
    }
    void TearDown() override {
        delete obstacleSensorInterface;
    }
};

//1-1-1 전방센서 장애물 여부 반환값 확인
TEST_F(ObstacleSensorInterface, Check_Front_Blocked) {
    ObstacleSensorInterface->last_front = 2;
    EXPECT_TRUE(ObstacleSensorInterface->isFrontBlocked());
    ObstacleSensorInterface->last_front = 10;
    EXPECT_TRUE(ObstacleSensorInterface->isFrontBlocked());
    ObstacleSensorInterface->last_front = 19;
    EXPECT_TRUE(ObstacleSensorInterface->isFrontBlocked());
    ObstacleSensorInterface->last_front = 20;
    EXPECT_TRUE(ObstacleSensorInterface->isFrontBlocked());
    ObstacleSensorInterface->last_front = 21;
    EXPECT_FALSE(ObstacleSensorInterface->isFrontBlocked());
    ObstacleSensorInterface->last_front = 52;
    EXPECT_FALSE(ObstacleSensorInterface->isFrontBlocked());
    ObstacleSensorInterface->last_front = 76;
    EXPECT_FALSE(ObstacleSensorInterface->isFrontBlocked());
}

//1-1-2 좌측센서 장애물 여부 반환값 확인
TEST_F(ObstacleSensorInterface, Check_Left_Blocked) {
    ObstacleSensorInterface->last_left = 18;
    EXPECT_TRUE(ObstacleSensorInterface->isLeftBlocked());
    ObstacleSensorInterface->last_left = 44;
    EXPECT_TRUE(ObstacleSensorInterface->isLeftBlocked());
    ObstacleSensorInterface->last_left = 59;
    EXPECT_TRUE(ObstacleSensorInterface->isLeftBlocked());
    ObstacleSensorInterface->last_left = 60;
    EXPECT_TRUE(ObstacleSensorInterface->isLeftBlocked());
    ObstacleSensorInterface->last_left = 61;
    EXPECT_FALSE(ObstacleSensorInterface->isLeftBlocked());
    ObstacleSensorInterface->last_left = 73;
    EXPECT_FALSE(ObstacleSensorInterface->isLeftBlocked());
    ObstacleSensorInterface->last_left = 115;
    EXPECT_FALSE(ObstacleSensorInterface->isLeftBlocked());
}

//1-1-3 우측센서 장애물 여부 반환값 확인
TEST_F(ObstacleSensorInterface, Check_Right_Blocked) {
    ObstacleSensorInterface->last_right = 5;
    EXPECT_TRUE(ObstacleSensorInterface->isRightBlocked());
    ObstacleSensorInterface->last_right = 33;
    EXPECT_TRUE(ObstacleSensorInterface->isRightBlocked());
    ObstacleSensorInterface->last_right = 59;
    EXPECT_TRUE(ObstacleSensorInterface->isRightBlocked());
    ObstacleSensorInterface->last_right = 60;
    EXPECT_TRUE(ObstacleSensorInterface->isRightBlocked());
    ObstacleSensorInterface->last_right = 61;
    EXPECT_FALSE(ObstacleSensorInterface->isRightBlocked());
    ObstacleSensorInterface->last_right = 91;
    EXPECT_FALSE(ObstacleSensorInterface->isRightBlocked());
    ObstacleSensorInterface->last_right = 125;
    EXPECT_FALSE(ObstacleSensorInterface->isRightBlocked());
}

//1-2 장애물 구조체 생성 확인
TEST_F(ObstacleSensorInterface, Check_Obstacle_Struct) {
    ObstacleSensorInterface->last_front = 6;
    ObstacleSensorInterface->last_left = 44;
    ObstacleSensorInterface->last_right = 48;
    EXPECT_EQ(isObstacleExist,{true,true,true});
    ObstacleSensorInterface->last_front = 11;
    ObstacleSensorInterface->last_left = 32;
    ObstacleSensorInterface->last_right = 117;
    EXPECT_EQ(isObstacleExist,{true,true,false});
    ObstacleSensorInterface->last_front = 1;
    ObstacleSensorInterface->last_left = 108;
    ObstacleSensorInterface->last_right = 38;
    EXPECT_EQ(isObstacleExist,{true,false,true});
    ObstacleSensorInterface->last_front = 4;
    ObstacleSensorInterface->last_left = 112;
    ObstacleSensorInterface->last_right = 104;
    EXPECT_EQ(isObstacleExist,{true,false,false});
    ObstacleSensorInterface->last_front = 114;
    ObstacleSensorInterface->last_left = 42;
    ObstacleSensorInterface->last_right = 40;
    EXPECT_EQ(isObstacleExist,{false,true,true});
    ObstacleSensorInterface->last_front = 61;
    ObstacleSensorInterface->last_left = 52;
    ObstacleSensorInterface->last_right = 85;
    EXPECT_EQ(isObstacleExist,{false,true,false});
    ObstacleSensorInterface->last_front = 22;
    ObstacleSensorInterface->last_left = 61;
    ObstacleSensorInterface->last_right = 47;
    EXPECT_EQ(isObstacleExist,{false,false,true});
    ObstacleSensorInterface->last_front = 86;
    ObstacleSensorInterface->last_left = 68;
    ObstacleSensorInterface->last_right = 93;
    EXPECT_EQ(isObstacleExist,{false,false,false});
}

//1-3 인터업트 즉시성 확인
//TEST_F() {
//
//}



//2-1 먼지 여부 반환값 확인
TEST_F(ObstacleSensorInterface, Check_Dust_Existence) {
    ObstacleSensorInterface->last_dust = 0;
    EXPECT_FALSE(ObstacleSensorInterface->isDustExistence());
    ObstacleSensorInterface->last_dust = 1;
    EXPECT_TRUE(ObstacleSensorInterface->isDustExistence());
    ObstacleSensorInterface->last_dust = 27;
    EXPECT_TRUE(ObstacleSensorInterface->isDustExistence());
    ObstacleSensorInterface->last_dust = 101;
    EXPECT_TRUE(ObstacleSensorInterface->isDustExistence());
}