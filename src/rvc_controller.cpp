#include "rvc_controller.h"
#include "../simulator/simulator_interface.h"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <sstream>
#include <sys/time.h>
#include <csignal>


// --- Timer ---
Timer::Timer(unsigned long ms) : duration(ms) {}
void Timer::setTimer() {
    std::this_thread::sleep_for(std::chrono::milliseconds(duration));
}

void Timer::setAlarmTimer() {

    struct itimerval timer_struct;

    timer_struct.it_value.tv_sec = 1;            // 1초
    timer_struct.it_value.tv_usec = 0;       // 0 마이크로초

    timer_struct.it_interval.tv_sec = 0;
    timer_struct.it_interval.tv_usec = 0;

    //타이머 설정
    std::cout << "1000ms timer" << std::endl;
    if (setitimer(ITIMER_REAL, &timer_struct, nullptr) == -1) {
        std::cerr << "Timer setup failed!" << std::endl;
    }
}

void Timer::removeTimer() {
    struct itimerval stop_timer;

    stop_timer.it_value.tv_sec = 0;
    stop_timer.it_value.tv_usec = 0;
    stop_timer.it_interval.tv_sec = 0;
    stop_timer.it_interval.tv_usec = 0;

    setitimer(ITIMER_REAL, &stop_timer, nullptr);

}

// --- SensorInterface ---
SensorInterface::SensorInterface() {}
SensorInterface::~SensorInterface() {}

bool SensorInterface::isFrontBlocked() {
    // 💡 팁: 통신 요청, 딜레이(sleep), 뮤텍스 락은 모두 Simulator 내부에 숨겨져 있습니다.
    return Simulator::getSensorData().front <= threshold;
}

bool SensorInterface::isLeftBlocked() {
    return Simulator::getSensorData().left <= thresholdside;
}

bool SensorInterface::isRightBlocked() {
    return Simulator::getSensorData().right <= thresholdside;
}

bool SensorInterface::isDustExistence() {
    return Simulator::getSensorData().dust > 0;
}

// (참고: 만약 isObstacleExist() 함수가 있다면 아래처럼 작성하시면 됩니다)
ObstacleStatus SensorInterface::isObstacleExist() {
    SensorData data = Simulator::getSensorData();
    return {data.front <= threshold, data.left <= thresholdside, data.right <= thresholdside};
}


// --- PathPlanner ---
PathPlanner::PathPlanner(SensorInterface* o) : osi(o) {}
Location PathPlanner::decisionPath() { 
    if (osi->isLeftBlocked()){
        if (osi->isRightBlocked()) return Location::REAR;
        else return Location::RIGHT;
    } else return Location::LEFT;

    return Location::LEFT; 
}

// --- DriveManager ---
DriveManager::DriveManager(PathPlanner* pp) : pathPlanner(pp), currentDriveState(Driving::STOP), turnTimer(1020) {}


Location DriveManager::avoidObstacle() {
    stopMotor();
    Location turn = pathPlanner->decisionPath();
    if (turn == Location::LEFT) {
        std::cout<<"trun LEFT start"<<std::endl;
        rotateLeft();
        turnTimer.setTimer();
        stopMotor();
        rotateForward();
        return turn;
    }
    else if (turn == Location::RIGHT) {
        std::cout<<"trun RIGHT start"<<std::endl;
        rotateRight();
        turnTimer.setTimer();
        stopMotor();
        rotateForward();
        return turn;
    }
    else {
        std::cout<<"trun backward start"<<std::endl;
        rotateBackward();
        return turn;
    }
    
}
void DriveManager::rotateForward() { 
    std::cout<<"start moveforward"<<std::endl;
    Simulator::sendDriveCommand(Driving::MOVEFORWARD); 
}
void DriveManager::rotateLeft() { Simulator::sendDriveCommand(Driving::TURNLEFT); }
void DriveManager::rotateRight() { Simulator::sendDriveCommand(Driving::TURNRIGHT); }
void DriveManager::rotateBackward() { Simulator::sendDriveCommand(Driving::MOVEBACKWARD); }
void DriveManager::stopMotor() { Simulator::sendDriveCommand(Driving::STOP); }
void DriveManager::rotateLeftb() {
    std::cout<<"trun LEFT start"<<std::endl;
    Simulator::sendDriveCommand(Driving::TURNLEFT);
    turnTimer.setTimer();
    stopMotor();
    rotateForward();
 }
void DriveManager::rotateRightb() { 
    std::cout<<"trun RIGHT start"<<std::endl;
    Simulator::sendDriveCommand(Driving::TURNRIGHT);
    turnTimer.setTimer();
    stopMotor();
    rotateForward(); 
}
// --- CleanerManager ---
CleanerManager::CleanerManager() : currentMode(CleanerMode::OFF), boostTimer(3000) {}

void CleanerManager::cleanerMode(CleanerMode mode) {
    if (currentMode == mode && mode != CleanerMode::UP) return;
    currentMode = mode;
    Simulator::sendCleanerCommand(mode);


    if (mode == CleanerMode::ON) {
        std::cout << "CleanerMode:ON"<<std::endl;
    } else if (mode == CleanerMode::OFF) {
        std::cout << "CleanerMode:OFF"<<std::endl;
        boostTimer.removeTimer();
    } else if (mode == CleanerMode::UP) {
        std::cout << "CleanerMode:UP"<<std::endl;
        boostTimer.setAlarmTimer();
    }
}
bool CleanerManager::iscleanerOn() {
    return currentMode != CleanerMode::OFF;
}
bool CleanerManager::isBoosterOn() {
    return currentMode == CleanerMode::UP;
}

// --- Controller ---/
Controller::Controller(DriveManager* d, CleanerManager* c, SensorInterface* os)
    : driveManager(d), cleanerManager(c), sensorInterface(os) {
    Simulator::registerObstacleInterruptCallback([this]() {
        this->interruptHandler(); 
    });
    sensorInterface->setController(this);
}

Controller::~Controller() { 
    onOff = false; 
    if (dustThread.joinable() && dustThread.get_id() != std::this_thread::get_id()) dustThread.join(); 
}

void Controller::interruptHandler() {
    std::cout << "Interrupt Occured"<<std::endl;
    driveManager->stopMotor();
    this->frontObstacleTriggered.store(true);
}

void Controller::avoidanceAction() {

    driveManager->stopMotor();
    std::cout<<"front Obstacle found"<<std::endl;
    usleep(1000);

    std::cout << "Starting Avoidance..." << std::endl;
    cleanerManager->cleanerMode(CleanerMode::OFF);
    
    Location turnLocation = driveManager->avoidObstacle();
    
    if (sensorInterface->isFrontBlocked()) {
        std::cout << "Still Blocked! Maintaining Flag..." << std::endl;
        return; 
    }

    this->frontObstacleTriggered.store(false);  //회피후, 다시 인터럽트를 받을 수 있는 상태가 되었기 때문에, 인터럽트 플래그 값을 다시 false로

    this->isAlarmSigExist.store(false); //혹시라도 removeTimer 하기 직전에 시그널을 받았을 경우
    if (turnLocation == Location::LEFT) {
        
        if (!sensorInterface->isRightBlocked()) {
            std::cout << "Input Error!, turn off" << std::endl;
            errorturnOff();
        } else {
            cleanerManager->cleanerMode(CleanerMode::ON);
        }
    } else if (turnLocation == Location::RIGHT) {
        if (!sensorInterface->isLeftBlocked()) {
            std::cout << "Input Error!, turn off" << std::endl;
            errorturnOff();
        } else {
            cleanerManager->cleanerMode(CleanerMode::ON);
        }
    } else {
        while(onOff) {
            sensorInterface->isFrontBlocked(); 
            if (!sensorInterface->isLeftBlocked()){
                std::this_thread::sleep_for(std::chrono::milliseconds(450));
                driveManager->rotateLeftb();
                break;
            }
            else if (!sensorInterface->isRightBlocked()){
                std::this_thread::sleep_for(std::chrono::milliseconds(450));
                driveManager->rotateRightb();
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        cleanerManager->cleanerMode(CleanerMode::ON);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}
void Controller::boosterOverHandler() {
    this->isAlarmSigExist.store(true);
}

void Controller::dustDetect() {
    while(onOff) {
        if (frontObstacleTriggered.load()) {
            avoidanceAction(); // 회피 함수 호출
            continue;   //회피후, 다시 루프를 돌면서 회피동작을 체크해서 연속해서 회피를 해야할 케이스 커버
        }

        if (this->isAlarmSigExist.load() && cleanerManager->isBoosterOn() ) {
            cleanerManager->cleanerMode(CleanerMode::ON);  //타이머가 종료하는데, 부스터 모드일 경우
            this->isAlarmSigExist.store(false);
        }
        
        if (sensorInterface->isDustExistence() && onOff) {
            std::cout << "Dust Detected" << std::endl;
            if(cleanerManager->isBoosterOn() == true) {
                cleanerManager->cleanerMode(CleanerMode::UP);
            } else {
            
                cleanerManager->cleanerMode(CleanerMode::UP);
                std::thread([this]() {
                    int caught_signal;

                    sigset_t wait_set;
                    sigemptyset(&wait_set);
                    sigaddset(&wait_set, SIGALRM);
                    
                    std::cout << "Waiting for SIGALRM" << std::endl;
                    // 스레드가 여기서 대기
                    sigwait(&wait_set, &caught_signal);

                    // 시그널이 도착하면 아래 로직 실행
                    if (caught_signal == SIGALRM) {
                        std::cout << "[Wait-Thread] SIGALRM Caught! Routing to Controller..." << std::endl;
                        this->boosterOverHandler();
                    }
                    
                }).detach();
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void Controller::turnOn() {
    if (onOff) return;
    std::cout << "[System] POWER ON" << std::endl;
    onOff = true;

    if (sensorInterface->isFrontBlocked()) {        //처음 turn on이 되었을때, 정면센서 값이 threshold값보다 작을경우 인터럽트 발생을 못함, 때문에 시작후 한번 체크
        this->frontObstacleTriggered.store(true);
    }

    cleanerManager->cleanerMode(CleanerMode::ON);
    driveManager->rotateForward();
    
    if (dustThread.joinable() && dustThread.get_id() != std::this_thread::get_id()) dustThread.join();
    dustThread = std::thread(&Controller::dustDetect, this);
}

void Controller::turnOff() {
    if (!onOff) return;
    std::cout << "[System] POWER OFF" << std::endl;
    onOff = false;
    
    if (dustThread.joinable() && dustThread.get_id() != std::this_thread::get_id()) dustThread.join();
    

    cleanerManager->cleanerMode(CleanerMode::OFF);
    driveManager->stopMotor();
}

void Controller::errorturnOff() {
    turnOff(); 
}

// --- ButtonInterface ---
ButtonInterface::ButtonInterface(Controller* ctrl) : controller(ctrl) {}
void ButtonInterface::pushButtonOn() { if (controller) controller->turnOn(); }
void ButtonInterface::pushButtonOff() { if (controller) controller->turnOff(); }
