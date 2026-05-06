#include <iostream>
#include <thread>
#include <chrono>
#include "rvc_controller.h"
#include <csignal>

int main() {
    
    std::cout << "RVC Software System Starting..." << std::endl;
    std::cout << "Connecting to Simulator on localhost:12345..." << std::endl;

    sigset_t my_sigset;
    sigemptyset(&my_sigset);
    sigaddset(&my_sigset, SIGALRM);
    pthread_sigmask(SIG_BLOCK, &my_sigset, nullptr);

    // 1. Initialize Boundary Interfaces
    ObstacleSensorInterface osi;
    DustSensorInterface dsi;
    
    // 2. Initialize Logic Components
    PathPlanner planner(&osi);
    DriveManager driver(&planner);
    CleanerManager cleaner;
    
    // 3. Initialize Central Controller
    Controller controller(&driver, &cleaner, &dsi, &osi);
    
    // 4. Initialize External Input (Button)
    ButtonInterface button(&controller);

    std::cout << "System Ready. Please press POWER ON in the Simulator UI." << std::endl;

    // Keep the main thread alive while background threads handle interrupts and buttons
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}
