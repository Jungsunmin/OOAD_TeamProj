#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "rvc_controller.h"
#include "UTest_Mock.h"
#include <chrono>
#include <thread>

using namespace testing;

class CleanerManagerTest : public ::testing::Test {
protected:
    // CleanerManager contains a real Timer boostTimer. 
    // Since we made Timer methods virtual, we can't easily replace the 
    // member without changing the class. 
    // However, we can use a signal handler or just ensure the test 
    // doesn't wait long enough for the real alarm to fire.
    
    CleanerManager* cleanerManager;

    void SetUp() override {
        cleanerManager = new CleanerManager();
    }
    void TearDown() override {
        // Ensure alarm is removed before finishing to avoid SIGALRM in other tests
        cleanerManager->cleanerMode(CleanerMode::OFF);
        delete cleanerManager;
    }
};

TEST_F(CleanerManagerTest, Test_iscleanerOn) {
    EXPECT_FALSE(cleanerManager->iscleanerOn());
    cleanerManager->cleanerMode(CleanerMode::ON);
    EXPECT_TRUE(cleanerManager->iscleanerOn());
    cleanerManager->cleanerMode(CleanerMode::OFF);
    EXPECT_FALSE(cleanerManager->iscleanerOn());
}

TEST_F(CleanerManagerTest, Test_getCurrentMode_iscleanerOn) {
    EXPECT_EQ(cleanerManager->getCurrentMode(), CleanerMode::OFF);
    cleanerManager->cleanerMode(CleanerMode::ON);
    EXPECT_EQ(cleanerManager->getCurrentMode(), CleanerMode::ON);
    cleanerManager->cleanerMode(CleanerMode::OFF);
    EXPECT_EQ(cleanerManager->getCurrentMode(), CleanerMode::OFF);
}

TEST_F(CleanerManagerTest, Test_cleanerMode_UP) {
    // UP 모드 설정 (내부적으로 3000ms 타이머 시작)
    cleanerManager->cleanerMode(CleanerMode::UP);
    EXPECT_EQ(cleanerManager->getCurrentMode(), CleanerMode::UP);

    // 짧게 대기하여 시그널이 발생하기 전에 상태 확인
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    EXPECT_EQ(cleanerManager->getCurrentMode(), CleanerMode::UP);
    
    // 테스트 종료 전 반드시 타이머 제거 (OFF 모드로 변경)
    cleanerManager->cleanerMode(CleanerMode::OFF);
}
