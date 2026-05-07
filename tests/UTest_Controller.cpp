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
    struct TestController : public Controller {
        using Controller::Controller;
    };
    TestController* testCtrl = new TestController(mockDM, mockCM, mockDS, mockOS);

    // 전방 장애물 감지 시나리오
    EXPECT_CALL(*mockDM, stopMotor()).Times(AtLeast(1));
    EXPECT_CALL(*mockCM, cleanerMode(CleanerMode::OFF)).Times(AtLeast(1));
    EXPECT_CALL(*mockDM, avoidObstacle()).WillOnce(Return(Location::LEFT));

    // 회피 후 정면 체크 (성공했다고 가정하여 false 리턴)=

    EXPECT_CALL(*mockOS, isRightBlocked()).WillOnce(Return(true));
    
    // 회피 성공 후 청소기를 다시 ON으로 돌리는 호출 추가
    EXPECT_CALL(*mockCM, cleanerMode(CleanerMode::ON)).Times(AtLeast(1));
    
    testCtrl->avoidanceAction();

    delete testCtrl;
}


// 4. dustDetect()에서 먼지 감지시 booster 모드 확인, on -> up시 sigalarm을 기다리는 스레드를 생성하는 코드부분 실행(첫번쨰 루프), up ->up시 스레드 생성 없이 리셋만 하는지 확인
// TEST_F(ControllerTest, DustDetect_TwoLoopIteration_ThreadSpawningLogic) {
//     struct TestController : public Controller {
//         using Controller::Controller;
//         void setOnOff(bool val) { this->onOff = val; }
//         void setFrontObstacle(bool val) { this->frontObstacleTriggered.store(val); }
//     };

//     TestController* testCtrl = new TestController(mockDM, mockCM, mockDS, mockOS);
    
//     // 초기 조건 설정
//     testCtrl->setOnOff(true);
//     testCtrl->setFrontObstacle(false);

//     //Sequence를 사용하여 순서를 보장
//     Sequence s;

//     // [Loop 1]
//     EXPECT_CALL(*mockOS, isDustExistence())
//         .InSequence(s)
//         .WillOnce(Return(true)); // 먼지 있음

//     // 첫 번째 루프에서의 isBoosterOn() 호출들 (코드상 총 2번 호출됨)
//     // 1. if (isAlarmSigExist && isBoosterOn) 체크용
//     // 2. if (isBoosterOn == true) 체크용
//     EXPECT_CALL(*mockCM, isBoosterOn())
//         .InSequence(s)
//         .WillOnce(Return(false)); // 타이머 체크 시 false
//     EXPECT_CALL(*mockCM, isBoosterOn())
//         .InSequence(s)
//         .WillOnce(Return(false)); // 메인 로직 체크 시 false -> else문 진입(스레드 생성)

//     EXPECT_CALL(*mockCM, cleanerMode(CleanerMode::UP))
//         .InSequence(s)
//         .Times(1);

//     // [Loop 2]
//     EXPECT_CALL(*mockOS, isDustExistence())
//         .InSequence(s)
//         .WillOnce(Return(true)); // 여전히 먼지 있음

//     // 두 번째 루프에서의 isBoosterOn() 호출들
//     EXPECT_CALL(*mockCM, isBoosterOn())
//         .InSequence(s)
//         .WillOnce(Return(true));  // 타이머 체크 시 true
    
//     EXPECT_CALL(*mockCM, isBoosterOn())
//         .InSequence(s)
//         .WillOnce(Invoke([&]() {
//             // 두 번째 루프의 메인 로직에서 true를 리턴하여 if문 진입
//             // 루프를 종료시키기 위해 여기서 onOff를 false로 변경
//             testCtrl->setOnOff(false);
//             return true; 
//         }));

//     EXPECT_CALL(*mockCM, cleanerMode(CleanerMode::UP))
//         .InSequence(s)
//         .Times(1);

//     testCtrl->dustDetect();

//     delete testCtrl;
// }

// //5. interruptHandler() 실행시 frontObstacleTriggered 값이 false -> true 되는지 체크
// TEST_F(ControllerTest, InterruptHandler_StopsMotorAndSetsFlag) {
//     struct TestController : public Controller {
//         using Controller::Controller;
        
//         bool isFrontTriggered() {
//             return this->frontObstacleTriggered.load();
//         }
//         // 테스트 시작 전 초기화
//         void resetFlag() {
//             this->frontObstacleTriggered.store(false);
//         }
//     };

//     TestController* testCtrl = new TestController(mockDM, mockCM, mockDS, mockOS);
//     testCtrl->resetFlag(); // 시작은 false

//     EXPECT_CALL(*mockDM, stopMotor()).Times(1);

//     testCtrl->interruptHandler();

//     EXPECT_TRUE(testCtrl->isFrontTriggered());

//     delete testCtrl;
// }

// //6.frontObstacleTriggered가 true일 때 avoidanceAction() 내부적으로 stopMotor()와 cleanerMode(OFF)이 호출되는지 체크
// // TEST_F(ControllerTest, DustDetect_ExecutesAvoidanceActionOnTrigger) {
// //     struct TestController : public Controller {
// //         using Controller::Controller;
// //         void setOnOff(bool val) { this->onOff = val; }
// //         void setFrontTrigger(bool val) { this->frontObstacleTriggered.store(val); }
// //     };

// //     TestController* testCtrl = new TestController(mockDM, mockCM, mockDS, mockOS);
// //     testCtrl->setOnOff(true);
// //     testCtrl->setFrontTrigger(true); // 장애물 인터럽트 발생 가정

// //     // avoidanceAction()이 실행될 때 호출되어야 하는 함수들
// //     EXPECT_CALL(*mockDM, stopMotor()).Times(AtLeast(1));
// //     EXPECT_CALL(*mockCM, cleanerMode(CleanerMode::OFF)).Times(AtLeast(1));
    
// //     // 무한 루프 방지: avoidObstacle 호출 시 onOff를 false로 변경하여 루프 탈출
// //     EXPECT_CALL(*mockDM, avoidObstacle())
// //         .WillOnce(Invoke([&]() {
// //             testCtrl->setOnOff(false); 
// //             return Location::LEFT; 
// //         }));

// //     testCtrl->dustDetect();

// //     delete testCtrl;
// // }

// //7.isAlarmSigExist가 true가 되었을 때, 청소기가 ON으로 복귀하고 플래그가 다시 false
// TEST_F(ControllerTest, DustDetect_RevertsToNormalModeOnAlarm) {
//     struct TestController : public Controller {
//         using Controller::Controller;
//         void setOnOff(bool val) { this->onOff = val; }
//         void setAlarmSig(bool val) { this->isAlarmSigExist.store(val); }
//         bool getAlarmSig() { return this->isAlarmSigExist.load(); }
//     };

//     TestController* testCtrl = new TestController(mockDM, mockCM, mockDS, mockOS);
//     testCtrl->setOnOff(true);
//     testCtrl->setAlarmSig(true); // 타이머가 종료되어 알람 신호가 온 상태 가정

//     //알람이 있고 부스터 모드인 상황
//     EXPECT_CALL(*mockCM, isBoosterOn()).WillRepeatedly(Return(true));

//     // 실행: cleanerMode(ON)이 호출되어야 함
//     EXPECT_CALL(*mockCM, cleanerMode(CleanerMode::ON))
//         .WillOnce(Invoke([&](CleanerMode mode) {
//             // 루프 종료를 위해 onOff를 false로
//             testCtrl->setOnOff(false);
//         }));

//     testCtrl->dustDetect();

//     EXPECT_FALSE(testCtrl->getAlarmSig());

//     delete testCtrl;
// }