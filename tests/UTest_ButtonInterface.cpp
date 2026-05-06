#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "rvc_controller.h"
#include "UTest_Mock.h"

using namespace testing;

class ButtonInterfaceTest : public ::testing::Test {
protected:
    MockDriveManager* mockDM;
    MockCleanerManager* mockCM;
    MockDustSensorInterface* mockDS;
    MockObstacleSensorInterface* mockOS;
    MockPathPlanner* mockPP;
    MockController* mockController;
    ButtonInterface* button;

    void SetUp() override {
        mockOS = new NiceMock<MockObstacleSensorInterface>();
        mockDS = new NiceMock<MockDustSensorInterface>();
        mockCM = new NiceMock<MockCleanerManager>();
        mockPP = new NiceMock<MockPathPlanner>(mockOS);
        mockDM = new NiceMock<MockDriveManager>(mockPP);
        mockController = new NiceMock<MockController>(mockDM, mockCM, mockDS, mockOS);
        button = new ButtonInterface(mockController);
    }

    void TearDown() override {
        delete button;
        delete mockController;
        delete mockDM;
        delete mockPP;
        delete mockCM;
        delete mockDS;
        delete mockOS;
    }
};

// 1. 꺼진 상황에서 버튼을 눌렀을 때 turnOn()이 호출되는지 확인
TEST_F(ButtonInterfaceTest, PushButtonOn_WhenOff) {
    EXPECT_CALL(*mockController, turnOn()).Times(1);
    button->pushButtonOn();
}

// 2. 켜진 상황에서 버튼을 눌렀을 때 turnOff()가 호출되는지 확인
TEST_F(ButtonInterfaceTest, PushButtonOff_WhenOn) {
    EXPECT_CALL(*mockController, turnOff()).Times(1);
    button->pushButtonOff();
}
