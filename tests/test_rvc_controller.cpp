#include <gtest/gtest.h>
#include "rvc_controller.h"

// Test fixture for RVC Controller
// class RvcControllerTest : public ::testing::Test {
// protected:
//     Motor leftMotor, rightMotor;
//     FrontObstacleSensor fs;
//     SideObstacleSensor ls, rs; 
//     DustSensor ds;
    
//     ObstacleSensorInterface* osi;
//     DustSensorInterface* dsi;
//     PathPlanner* planner;
//     DriveManager* driver;
//     CleanerManager* cleaner;
//     Controller* controller;

//     void SetUp() override {
//         osi = new ObstacleSensorInterface(&fs, &ls, &rs);
//         dsi = new DustSensorInterface(&ds);
//         planner = new PathPlanner(osi);
//         driver = new DriveManager(&leftMotor, &rightMotor, planner, Driving::STOP);
//         cleaner = new CleanerManager();
//         controller = new Controller(driver, cleaner, dsi);
//         controller->setObstacleSensorInterface(osi);
//     }

//     void TearDown() override {
//         delete controller;
//         delete cleaner;
//         delete driver;
//         delete planner;
//         delete dsi;
//         delete osi;
//     }
// };

// TEST_F(RvcControllerTest, InitialStateIsOff) {
//     // By default, system should be off. 
//     controller->interruptHandler(); 
//     EXPECT_EQ(cleaner->getCurrentMode(), CleanerMode::OFF);
// }

// TEST_F(RvcControllerTest, StartsCleaningCorrectly) {
//     controller->turnOn();
//     EXPECT_EQ(cleaner->getCurrentMode(), CleanerMode::ON);
//     EXPECT_EQ(driver->getCurrentState(), Driving::MOVEFORWARD);
// }

// TEST_F(RvcControllerTest, BoostsWhenDustDetected) {
//     controller->turnOn();
//     ds.setDust(true);
//     controller->interruptHandler(); // Should detect dust and boost
//     EXPECT_EQ(cleaner->getCurrentMode(), CleanerMode::UP);
// }

// TEST_F(RvcControllerTest, AvoidsFrontObstacle) {
//     controller->turnOn();
//     fs.setBlocked(true);
//     controller->interruptHandler(); // Should detect obstacle and start avoiding
//     EXPECT_NE(driver->getCurrentState(), Driving::MOVEFORWARD);
// }

// TEST_F(RvcControllerTest, PathPlannerLogic) {
//     // Front blocked, left/right clear -> Turn Left
//     fs.setBlocked(true);
//     ls.setStatus(false, false);
//     rs.setStatus(false, false);
//     planner->decisionPath();
//     EXPECT_EQ(planner->getSelectedDriving(), Driving::TURNLEFT);
    
//     // Front and Left blocked -> Turn Right
//     ls.setStatus(true, false);
//     planner->decisionPath();
//     EXPECT_EQ(planner->getSelectedDriving(), Driving::TURNRIGHT);
    
//     // All front/sides blocked -> Move Backward
//     rs.setStatus(false, true); // right side sensor
//     planner->decisionPath();
//     EXPECT_EQ(planner->getSelectedDriving(), Driving::MOVEBACKWARD);
    
//     // No obstacle -> Move Forward
//     fs.setBlocked(false);
//     planner->decisionPath();
//     EXPECT_EQ(planner->getSelectedDriving(), Driving::MOVEFORWARD);
// }
