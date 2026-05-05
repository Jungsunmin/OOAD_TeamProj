#include <gtest/gtest.h>
#include "rvc_controller.h"

// Mock class to override sensor behavior for testing
class MockObstacleSensorInterface : public ObstacleSensorInterface {
public:
    bool front = false;
    bool left = false;
    bool right = false;
    bool dust = false;

    bool isFrontBlocked() { return front; }
    bool isLeftBlocked() { return left; }
    bool isRightBlocked() { return right; }
    bool isDustExistence() { return dust; }
    ObstacleStatus isObstacleExist() { return {front, left, right}; }
};

class RvcControllerTest : public ::testing::Test {
protected:
    MockObstacleSensorInterface* osi;
    DustSensorInterface* dsi;
    PathPlanner* planner;
    DriveManager* driver;
    CleanerManager* cleaner;
    Controller* controller;

    void SetUp() override {
        osi = new MockObstacleSensorInterface();
        dsi = new DustSensorInterface();
        planner = new PathPlanner(osi);
        driver = new DriveManager(planner);
        cleaner = new CleanerManager();
        controller = new Controller(driver, cleaner, dsi, osi);
    }

    void TearDown() override {
        delete controller;
        delete cleaner;
        delete driver;
        delete planner;
        delete dsi;
        delete osi;
    }
};

TEST_F(RvcControllerTest, InitialStateIsOff) {
    EXPECT_EQ(cleaner->getCurrentMode(), CleanerMode::OFF);
    EXPECT_EQ(driver->getCurrentState(), Driving::STOP);
}

TEST_F(RvcControllerTest, StartsCleaningCorrectly) {
    controller->turnOn();
    EXPECT_EQ(cleaner->getCurrentMode(), CleanerMode::ON);
    EXPECT_EQ(driver->getCurrentState(), Driving::MOVEFORWARD);
}

TEST_F(RvcControllerTest, PathPlannerLogic) {
    // Front blocked, left/right clear -> Turn Left
    osi->front = true;
    osi->left = false;
    osi->right = false;
    EXPECT_EQ(planner->decisionPath(), Location::LEFT);
    
    // Front and Left blocked -> Turn Right
    osi->left = true;
    EXPECT_EQ(planner->decisionPath(), Location::RIGHT);
    
    // All front/sides blocked -> Move Backward (REAR)
    osi->right = true;
    EXPECT_EQ(planner->decisionPath(), Location::REAR);
}

TEST_F(RvcControllerTest, PowerOffStopsEverything) {
    controller->turnOn();
    controller->turnOff();
    EXPECT_EQ(cleaner->getCurrentMode(), CleanerMode::OFF);
    EXPECT_EQ(driver->getCurrentState(), Driving::STOP);
}
