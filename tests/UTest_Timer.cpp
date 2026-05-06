#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "rvc_controller.h"
#include "UTest_Mock.h"
#include <chrono>

using namespace testing;

class TimerTest : public ::testing::Test {
protected:
    void TearDown() override {
        // 테스트 중에 설정되었을 수 있는 시스템 타이머를 안전하게 제거합니다.
        Timer temp(0);
        temp.removeTimer();
    }
};

// 1. setTimer()가 설정된 시간(ms)만큼 대기하는지 확인
TEST_F(TimerTest, SetTimer_WaitCorrectly) {
    unsigned long waitTime = 100;
    Timer timer(waitTime);

    auto start = std::chrono::high_resolution_clock::now();
    timer.setTimer(); // 실제 std::this_thread::sleep_for 실행
    auto end = std::chrono::high_resolution_clock::now();
    
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    
    // OS 스케줄링 오차를 고려하여 검증 (보통 5~10ms 내외)
    EXPECT_GE(elapsed, waitTime - 5);
}

// 2. setAlarmTimer() 호출 시 예외가 발생하지 않는지 확인
TEST_F(TimerTest, SetAlarmTimer_Execution) {
    Timer timer(1000);
    
    // 시스템 호출(setitimer)이 성공적으로 수행되는지 확인
    EXPECT_NO_THROW(timer.setAlarmTimer());
    
    // 테스트용 알람이 실제 SIGALRM을 발생시키기 전에 바로 제거
    timer.removeTimer();
}

// 3. removeTimer() 호출 시 예외가 발생하지 않는지 확인
TEST_F(TimerTest, RemoveTimer_Execution) {
    Timer timer(0);
    EXPECT_NO_THROW(timer.removeTimer());
}
