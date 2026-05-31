#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "rvc_controller.h"
#include "UTest_Mock.h"

using namespace testing;


class FakeOSI : public ObstacleSensorInterface {
public:
    SensorData fake{0, 0, 0, 0};
 
    void setSensorData(int front, int left, int right, int dust) {
        fake = SensorData{front, left, right, dust};
    }
 
    // 테스트가 임계값을 그대로 참조할 수 있도록 접근자 제공 (protected 멤버 노출)
    int frontThreshold() const { return threshold; }
    int sideThreshold()  const { return thresholdside; }
 
protected:
    SensorData readSensorData() override { return fake; }
};
 
class ObstacleSensorInterfaceTest : public ::testing::Test {
protected:
    FakeOSI osi;
};

// 전방센서 임계값 = threshold =20
// 1-1-1 전방센서 장애물 여부 반환값 확인 (경계값 테스트)
TEST_F(ObstacleSensorInterfaceTest, Check_Front_Blocked) {
    ASSERT_EQ(osi.frontThreshold(), 20);
 
    struct Case { int front; bool expected; };
    const Case cases[] = {
        {  2, true},
        { 10, true},
        { 19, true},
        { 20, true},
        { 21, false},
        { 52, false},
        { 76, false},
    };
 
    for (const auto& c : cases) {
        osi.setSensorData(c.front, 100, 100, 0);
        EXPECT_EQ(osi.isFrontBlocked(), c.expected)
            << "front=" << c.front;
    }
}

// 측면 센서 임계값 = thresholdside = 60
// 1-1-2 좌측센서 장애물 여부 반환값 확인 (경계값 테스트)
TEST_F(ObstacleSensorInterfaceTest, Check_Left_Blocked) {
    ASSERT_EQ(osi.sideThreshold(), 60);
 
    struct Case { int left; bool expected;};
    const Case cases[] = {
        { 18,  true},
        { 44,  true},
        { 59,  true},
        { 60,  true},
        { 61,  false},
        { 73,  false},
        { 115, false},
    };
 
    for (const auto& c : cases) {
        osi.setSensorData(100, c.left, 100, 0);
        EXPECT_EQ(osi.isLeftBlocked(), c.expected)
            << "left=" << c.left;
    }
}


// // 1-2 장애물 구조체 생성 확인 (8가지 조합 진리표 테스트)
// TEST_F(ObstacleSensorInterfaceTest, Check_Obstacle_Struct) {
//     // 1. T, T, T
//     osi->last_front = 6; osi->last_left = 44; osi->last_right = 48;
//     EXPECT_EQ(osi->isObstacleExist(), (ObstacleStatus{true, true, true}));
//
//     // 2. T, T, F
//     osi->last_front = 11; osi->last_left = 32; osi->last_right = 117;
//     EXPECT_EQ(osi->isObstacleExist(), (ObstacleStatus{true, true, false}));
//
//     // 3. T, F, T
//     osi->last_front = 1; osi->last_left = 108; osi->last_right = 38;
//     EXPECT_EQ(osi->isObstacleExist(), (ObstacleStatus{true, false, true}));
//
//     // 4. T, F, F
//     osi->last_front = 4; osi->last_left = 112; osi->last_right = 104;
//     EXPECT_EQ(osi->isObstacleExist(), (ObstacleStatus{true, false, false}));
//
//     // 5. F, T, T
//     osi->last_front = 114; osi->last_left = 42; osi->last_right = 40;
//     EXPECT_EQ(osi->isObstacleExist(), (ObstacleStatus{false, true, true}));
//
//     // 6. F, T, F
//     osi->last_front = 61; osi->last_left = 52; osi->last_right = 85;
//     EXPECT_EQ(osi->isObstacleExist(), (ObstacleStatus{false, true, false}));
//
//     // 7. F, F, T
//     osi->last_front = 22; osi->last_left = 61; osi->last_right = 47;
//     EXPECT_EQ(osi->isObstacleExist(), (ObstacleStatus{false, false, true}));
//
//     // 8. F, F, F
//     osi->last_front = 86; osi->last_left = 68; osi->last_right = 93;
//     EXPECT_EQ(osi->isObstacleExist(), (ObstacleStatus{false, false, false}));
// }

// 2-1 먼지 여부 반환값 확인
TEST_F(ObstacleSensorInterfaceTest, IsDustExistence_BoundaryValues) {
    struct Case { int dust; bool expected;};
    const Case cases[] = {
        {   0, false},
        {   1, true},
        {  27, true},
        { 101, true},
    };
 
    for (const auto& c : cases) {
        osi.setSensorData(100, 100, 100, c.dust);
        EXPECT_EQ(osi.isDustExistence(), c.expected)
            << "dust=" << c.dust;
    }
}
