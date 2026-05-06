#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "rvc_controller.h"
#include "UTest_Mock.h"

using namespace testing;

class ControllerTest : public ::testing::Test {
protected:
    NiceMock<MockDriveManager>* mockDM;
    NiceMock<MockCleanerManager>* mockCM;
    NiceMock<MockDustSensorInterface>* mockDS;
    NiceMock<MockObstacleSensorInterface>* mockOS;
    NiceMock<MockPathPlanner>* mockPP;
    Controller* controller;

    void SetUp() override {
        mockOS = new NiceMock<MockObstacleSensorInterface>();
        mockDS = new NiceMock<MockDustSensorInterface>();
        mockCM = new NiceMock<MockCleanerManager>();
        mockPP = new NiceMock<MockPathPlanner>(mockOS);
        mockDM = new NiceMock<MockDriveManager>(mockPP);
        
        controller = new Controller(mockDM, mockCM, mockDS, mockOS);
    }

    void TearDown() override {
        delete controller;
        delete mockDM;
        delete mockPP;
        delete mockCM;
        delete mockDS;
        delete mockOS;
    }
};

// 1. turnOn() 시 내부 컴포넌트 호출 검증
TEST_F(ControllerTest, TurnOn_CallsComponents) {
    EXPECT_CALL(*mockCM, cleanerMode(CleanerMode::ON)).Times(1);
    EXPECT_CALL(*mockDM, rotateForward()).Times(1);
    
    controller->turnOn();
}

// 2. turnOff() 시 내부 컴포넌트 호출 검증
TEST_F(ControllerTest, TurnOff_CallsComponents) {
    controller->turnOn();
    
    EXPECT_CALL(*mockCM, cleanerMode(CleanerMode::OFF)).Times(1);
    EXPECT_CALL(*mockDM, stopMotor()).Times(1);
    
    controller->turnOff();
}

// 3. 장애물 감지 시 회피 로직 호출 검증
TEST_F(ControllerTest, avoidanceAction_TriggersAvoidance) {
    // onOff를 true로 설정하여 루프 진입 가능하게 함 (protected 접근)
    struct TestController : public Controller {
        using Controller::onOff;
        using Controller::Controller;
    };
    TestController* testCtrl = new TestController(mockDM, mockCM, mockDS, mockOS);
    testCtrl->onOff = true;

    // 전방 장애물 감지 시나리오
    EXPECT_CALL(*mockOS, isFrontBlocked()).WillOnce(Return(true)).WillRepeatedly(Return(false));
    EXPECT_CALL(*mockDM, stopMotor()).Times(AtLeast(1));
    EXPECT_CALL(*mockCM, cleanerMode(CleanerMode::OFF)).Times(AtLeast(1));
    EXPECT_CALL(*mockDM, avoidObstacle()).WillOnce(Return(Location::LEFT));
    
    // 회피 성공 후 청소기를 다시 ON으로 돌리는 호출 추가
    EXPECT_CALL(*mockCM, cleanerMode(CleanerMode::ON)).Times(AtLeast(1));
    
    // 루프를 한 번 돌리고 빠져나오기 위해 별도 스레드에서 실행
    std::thread t(&Controller::avoidanceAction, testCtrl);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    testCtrl->onOff = false;
    t.join();

    delete testCtrl;
}
