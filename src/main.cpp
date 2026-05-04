#include <iostream>
#include <thread>
#include <chrono>
#include "rvc_controller.h"

int main() {
    // 1. Initialize Basic Components
    Motor leftMotor, rightMotor;
    FrontObstacleSensor fs;
    SideObstacleSensor ls, rs; 
    DustSensor ds;

    // 2. Initialize Class-Diagram-Specified Interfaces
    ObstacleSensorInterface osi(&fs, &ls, &rs);
    DustSensorInterface dsi(&ds);

    // 3. Initialize Logic & Managers
    PathPlanner planner(&osi);
    DriveManager driver(&leftMotor, &rightMotor, &planner, Driving::STOP);
    CleanerManager cleaner;

    // 4. Initialize Controller & Button
    Controller controller(&driver, &cleaner, &dsi);
    controller.setObstacleSensorInterface(&osi);
    
    Button button(&controller);

    // 5. Simulation start
    std::cout << "--- Starting RVC Simulation (Detailed Class Diagrams) ---\n";
    
    button.pushButtonOn();

    // Simulate events
    for (int i = 0; i < 20; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        if (i == 5) {
            std::cout << "[Event] Dust detected!\n";
            ds.setDust(true);
        }
        if (i == 8) {
            ds.setDust(false);
        }
        if (i == 12) {
            std::cout << "[Event] Front obstacle detected!\n";
            fs.setBlocked(true);
        }
        if (i == 15) {
            fs.setBlocked(false);
        }

        controller.interruptHandler();
    }

    button.pushButtonOff();
    
    std::cout << "--- RVC Simulation Finished ---\n";

    return 0;
}
