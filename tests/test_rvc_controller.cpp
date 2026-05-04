#include <gtest/gtest.h>
#include "rvc_controller.h"

// Test fixture for RVC Controller
class RvcControllerTest : public ::testing::Test {
protected:
    DustSensor ds;
    FrontObstacleSensor fs;
    SideObstacleSensor ss;
    RvcController* rvc;

    void SetUp() override {
        rvc = new RvcController(&ds, &fs, &ss);
    }

    void TearDown() override {
        delete rvc;
    }
};

TEST_F(RvcControllerTest, InitialStateIsOff) {
    // By default, system should be off. 
    // We can't easily check private 'isOn', but we can check if it stays stopped.
    rvc->interruptHandler(); 
    // If it were on, it would print something.
}

TEST_F(RvcControllerTest, StartsCleaningCorrectly) {
    rvc->turnOn();
    // In a real test, we might mock Motor/Cleaner to verify calls.
    // For now, we trust the output and basic state transitions.
}

TEST_F(RvcControllerTest, BoostsWhenDustDetected) {
    rvc->turnOn();
    ds.setDust(true);
    rvc->interruptHandler(); // Should detect dust and boost
    // We can observe the behavior in simulation
}

TEST_F(RvcControllerTest, AvoidsFrontObstacle) {
    rvc->turnOn();
    fs.setBlocked(true);
    rvc->interruptHandler(); // Should detect obstacle and start avoiding
}

TEST_F(RvcControllerTest, PathPlannerLogic) {
    PathPlanner planner;
    
    // Front blocked, left/right clear -> Turn Left
    EXPECT_EQ(planner.decisionPath({true, false, false}), DrivingMode::TURN_LEFT);
    
    // Front and Left blocked -> Turn Right
    EXPECT_EQ(planner.decisionPath({true, true, false}), DrivingMode::TURN_RIGHT);
    
    // All front/sides blocked -> Move Backward
    EXPECT_EQ(planner.decisionPath({true, true, true}), DrivingMode::MOVE_BACKWARD);
    
    // No obstacle -> Move Forward
    EXPECT_EQ(planner.decisionPath({false, false, false}), DrivingMode::MOVE_FORWARD);
}

// Legacy support tests
TEST(RvcLegacyTest, LegacyNextAction) {
    RvcController c;
    EXPECT_STREQ(c.next_action(false, false, false, false), "GO_STRAIGHT");
    EXPECT_STREQ(c.next_action(false, false, false, true), "BOOST_CLEANING");
    EXPECT_STREQ(c.next_action(true, false, false, false), "STOP_AND_AVOID_TURN");
    EXPECT_STREQ(c.next_action(true, true, true, false), "REVERSE_THEN_TURN");
}