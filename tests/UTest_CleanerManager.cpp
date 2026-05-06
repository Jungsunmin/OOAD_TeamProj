#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "rvc_controller.h"
#include "UTest_Mock.h"
#include <chrono>
#include <thread>

using namespace testing;

class CleanerManagerTest : public ::testing::Test {
protected:
    CleanerManager* cleanerManager;

    void SetUp() override {
        cleanerManager = new CleanerManager();
    }
    void TearDown() override {
        // Ensure alarm is removed before finishing
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
    cleanerManager->cleanerMode(CleanerMode::UP);
    EXPECT_EQ(cleanerManager->getCurrentMode(), CleanerMode::UP);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    EXPECT_EQ(cleanerManager->getCurrentMode(), CleanerMode::UP);
    
    cleanerManager->cleanerMode(CleanerMode::OFF);
}
