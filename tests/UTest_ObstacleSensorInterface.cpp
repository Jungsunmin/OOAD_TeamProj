#include <gtest/gtest.h>
#include <gmock/gmock.h>

// private 멤버 접근을 위한 trick (반드시 헤더 포함 전에 선언)
#define private public
#include "rvc_controller.h"
#undef private

using namespace testing;

// ObstacleStatus 비교를 위한 전역 연산자 정의 (EXPECT_EQ 사용을 위함)
inline bool operator==(const ObstacleStatus& a, const ObstacleStatus& b) {
    return a.frontBlocked == b.frontBlocked &&
           a.leftBlocked == b.leftBlocked &&
           a.rightBlocked == b.rightBlocked;
}

class ObstacleSensorInterfaceTest : public ::testing::Test {
protected:
    ObstacleSensorInterface* osi;

    void SetUp() override {
        osi = new ObstacleSensorInterface();
    }

    void TearDown() override {
        delete osi;
    }
};

// 1-1-1 전방센서 장애물 여부 반환값 확인 (경계값 테스트)
TEST_F(ObstacleSensorInterfaceTest, Check_Front_Blocked) {
    const int values_true[] = {2, 10, 19, 20};
    for (int v : values_true) {
        osi->last_front = v;
        EXPECT_TRUE(osi->isFrontBlocked()) << "Failed at front value: " << v;
    }

    const int values_false[] = {21, 52, 76};
    for (int v : values_false) {
        osi->last_front = v;
        EXPECT_FALSE(osi->isFrontBlocked()) << "Failed at front value: " << v;
    }
}

// 1-1-2 좌측센서 장애물 여부 반환값 확인 (경계값 테스트)
TEST_F(ObstacleSensorInterfaceTest, Check_Left_Blocked) {
    const int values_true[] = {18, 44, 59, 60};
    for (int v : values_true) {
        osi->last_left = v;
        EXPECT_TRUE(osi->isLeftBlocked()) << "Failed at left value: " << v;
    }

    const int values_false[] = {61, 73, 115};
    for (int v : values_false) {
        osi->last_left = v;
        EXPECT_FALSE(osi->isLeftBlocked()) << "Failed at left value: " << v;
    }
}

// 1-1-3 우측센서 장애물 여부 반환값 확인 (경계값 테스트)
TEST_F(ObstacleSensorInterfaceTest, Check_Right_Blocked) {
    const int values_true[] = {5, 33, 59, 60};
    for (int v : values_true) {
        osi->last_right = v;
        EXPECT_TRUE(osi->isRightBlocked()) << "Failed at right value: " << v;
    }

    const int values_false[] = {61, 91, 125};
    for (int v : values_false) {
        osi->last_right = v;
        EXPECT_FALSE(osi->isRightBlocked()) << "Failed at right value: " << v;
    }
}

// 1-2 장애물 구조체 생성 확인 (8가지 조합 진리표 테스트)
TEST_F(ObstacleSensorInterfaceTest, Check_Obstacle_Struct) {
    // 1. T, T, T
    osi->last_front = 6; osi->last_left = 44; osi->last_right = 48;
    EXPECT_EQ(osi->isObstacleExist(), (ObstacleStatus{true, true, true}));

    // 2. T, T, F
    osi->last_front = 11; osi->last_left = 32; osi->last_right = 117;
    EXPECT_EQ(osi->isObstacleExist(), (ObstacleStatus{true, true, false}));

    // 3. T, F, T
    osi->last_front = 1; osi->last_left = 108; osi->last_right = 38;
    EXPECT_EQ(osi->isObstacleExist(), (ObstacleStatus{true, false, true}));

    // 4. T, F, F
    osi->last_front = 4; osi->last_left = 112; osi->last_right = 104;
    EXPECT_EQ(osi->isObstacleExist(), (ObstacleStatus{true, false, false}));

    // 5. F, T, T
    osi->last_front = 114; osi->last_left = 42; osi->last_right = 40;
    EXPECT_EQ(osi->isObstacleExist(), (ObstacleStatus{false, true, true}));

    // 6. F, T, F
    osi->last_front = 61; osi->last_left = 52; osi->last_right = 85;
    EXPECT_EQ(osi->isObstacleExist(), (ObstacleStatus{false, true, false}));

    // 7. F, F, T
    osi->last_front = 22; osi->last_left = 61; osi->last_right = 47;
    EXPECT_EQ(osi->isObstacleExist(), (ObstacleStatus{false, false, true}));

    // 8. F, F, F
    osi->last_front = 86; osi->last_left = 68; osi->last_right = 93;
    EXPECT_EQ(osi->isObstacleExist(), (ObstacleStatus{false, false, false}));
}

// 2-1 먼지 여부 반환값 확인
TEST_F(ObstacleSensorInterfaceTest, Check_Dust_Existence) {
    osi->last_dust = 0;
    EXPECT_FALSE(osi->isDustExistence());

    const int dust_values[] = {1, 27, 101};
    for (int v : dust_values) {
        osi->last_dust = v;
        EXPECT_TRUE(osi->isDustExistence()) << "Failed at dust value: " << v;
    }
}
