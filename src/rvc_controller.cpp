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

    timer_struct.it_value.tv_sec = 1;        // 1초
    timer_struct.it_value.tv_usec = 0;       // 0 마이크로초

    timer_struct.it_interval.tv_sec = 0;
    timer_struct.it_interval.tv_usec = 0;

    std::cout << "[System] Starting 1000ms timer" << std::endl;
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


// --- ObstacleSensorInterface ---
ObstacleSensorInterface::ObstacleSensorInterface() {}

ObstacleSensorInterface::~ObstacleSensorInterface() {}

SensorData ObstacleSensorInterface::readSensorData() {
    return Simulator::getSensorData();
}

bool ObstacleSensorInterface::isFrontBlockedTest() {
    return readSensorData().front <= threshold;
}

bool ObstacleSensorInterface::isLeftBlockedTest() {
    return readSensorData().left <= thresholdside;
}

bool ObstacleSensorInterface::isDustExistenceTest() {
    return readSensorData().dust > 0; 
}


bool ObstacleSensorInterface::isFrontBlocked() {
    return Simulator::getSensorData().front <= threshold;
}

bool ObstacleSensorInterface::isLeftBlocked() {
    return Simulator::getSensorData().left <= thresholdside;
}

bool ObstacleSensorInterface::isDustExistence() {
    return Simulator::getSensorData().dust > 0;
}


// --- PathPlanner ---
PathPlanner::PathPlanner(ObstacleSensorInterface* o) : osi(o) {}

// 1차 판단:
// - 왼쪽이 비어 있으면 LEFT
// - 왼쪽이 막혀 있으면 오른쪽 센서 없이 Clockwise 회전으로 탐색
Location PathPlanner::decisionPath() {
    if (osi->isLeftBlocked()) {
        return Location::Clockwise;
    }

    return Location::LEFT;
}

// Clockwise 회전 이후 2차 판단:
// - 전방이 비어 있으면 Forward
// - 전방이 아직 막혀 있으면 CounterClockwise로 복귀 후 후진
Location PathPlanner::decisionAfterClockwise() {
    if (osi->isFrontBlocked()) {
        return Location::CounterClockwise;
    }

    return Location::Forward;
}


// --- DriveManager ---
DriveManager::DriveManager(PathPlanner* pp)
    : pathPlanner(pp), currentDriveState(Driving::STOP), turnTimer(1000) {}

Location DriveManager::avoidObstacle() {
    while (true) {
        stopMotor();

        Location turn = pathPlanner->decisionPath();

        // Sequence - avoid obstacle - turn Left
        if (turn == Location::LEFT) {
            std::cout << "[System] Left side cleared! Escaping to Left." << std::endl;

            rotateLeft();
            turnTimer.setTimer();
            stopMotor();

            rotateForward();
            return Location::LEFT;
        }

        // Sequence - avoid obstacle - turn Right(Clockwise search)
        if (turn == Location::Clockwise) {
            std::cout << "[System] Left side blocked. Rotate Clockwise." << std::endl;

            rotateRight();          // Clockwise
            turnTimer.setTimer();
            stopMotor();

            Location afterClockwise = pathPlanner->decisionAfterClockwise();

            if (afterClockwise == Location::Forward) {
                std::cout << "[System] Front is clear after Clockwise. Go Forward." << std::endl;

                rotateForward();
                return Location::Forward;
            }

            if (afterClockwise == Location::CounterClockwise) {
                std::cout << "[System] Front is still blocked. Return and move backward." << std::endl;

                rotateLeft();       // CounterClockwise: 초기 방향으로 복귀
                turnTimer.setTimer();
                stopMotor();

                rotateBackward();
                turnTimer.setTimer();
                stopMotor();

                // 후진 후 다시 처음 판단으로 돌아감
                continue;
            }
        }

        // 예상하지 못한 Location 값이 들어온 경우 안전 정지
        std::cout << "[System] Unknown path decision. Stop motor." << std::endl;
        stopMotor();
        return turn;
    }
}

void DriveManager::rotateForward() {
    std::cout << "start moveforward" << std::endl;
    Simulator::sendDriveCommand(Driving::MOVEFORWARD);
    currentDriveState = Driving::MOVEFORWARD;
}

void DriveManager::rotateLeft() {
    Simulator::sendDriveCommand(Driving::TURNLEFT);
    currentDriveState = Driving::TURNLEFT;
}

void DriveManager::rotateRight() {
    Simulator::sendDriveCommand(Driving::TURNRIGHT);
    currentDriveState = Driving::TURNRIGHT;
}

void DriveManager::rotateBackward() {
    Simulator::sendDriveCommand(Driving::MOVEBACKWARD);
    currentDriveState = Driving::MOVEBACKWARD;
}

void DriveManager::stopMotor() {
    Simulator::sendDriveCommand(Driving::STOP);
    currentDriveState = Driving::STOP;
}

// 기존 rotateLeftb()/rotateRightb()는 오른쪽 센서 기반 후처리 로직에서 사용되던 보조 함수입니다.
// 새 시퀀스 다이어그램에서는 avoidObstacle() 내부에서 회피 동작을 모두 처리하므로 더 이상 호출하지 않습니다.
// 헤더에 선언되어 있고 다른 파일에서 참조할 가능성을 고려해 함수 정의는 유지하되, 현재 cpp 내부에서는 사용하지 않습니다.
void DriveManager::rotateLeftb() {
    Simulator::sendDriveCommand(Driving::TURNLEFT);
    turnTimer.setTimer();
    stopMotor();
    rotateForward();
}

void DriveManager::rotateRightb() {
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

    if (mode == CleanerMode::OFF) {
        boostTimer.removeTimer();
    }

    if (mode == CleanerMode::UP) {
        boostTimer.setAlarmTimer();
    }
}

bool CleanerManager::iscleanerOn() {
    return currentMode != CleanerMode::OFF;
}

bool CleanerManager::isBoosterOn() {
    return currentMode == CleanerMode::UP;
}


// --- Controller ---
Controller::Controller(DriveManager* d, CleanerManager* c, ObstacleSensorInterface* os)
    : driveManager(d), cleanerManager(c), obstacleSensorInterface(os) {
    Simulator::registerObstacleInterruptCallback([this]() {
        this->interruptHandler();
    });

    obstacleSensorInterface->setController(this);
}

Controller::~Controller() {
    onOff = false;

    if (dustThread.joinable() && dustThread.get_id() != std::this_thread::get_id()) {
        dustThread.join();
    }
}

void Controller::interruptHandler() {
    // 회피 진행 중에는 회전 sweep 으로 인한 추가 INTERRUPT 를 무시
    // (다시 전진+청소가 재개되는 시점에 isAvoiding 이 false 로 풀림)
    if (this->isAvoiding.load()) return;

    driveManager->stopMotor();
    this->frontObstacleTriggered.store(true);
}

void Controller::avoidanceAction() {
    // 회피 진입 시점부터 추가 인터럽트를 무시 (회전 중 sweep 으로 재발화되는 INTERRUPT 차단)
    this->isAvoiding.store(true);

    driveManager->stopMotor();

    std::cout << "front sensor interrupt" << std::endl;
    usleep(1000);

    std::cout << "[System] Obstacle Detected! Starting Avoidance..." << std::endl;

    cleanerManager->cleanerMode(CleanerMode::OFF);

    // 회피 동작은 DriveManager::avoidObstacle() 내부에서
    // LEFT / Clockwise / Forward / CounterClockwise / Backward 흐름까지 모두 처리합니다.
    driveManager->avoidObstacle();

    // 회피 후에도 전방이 막혀 있다면 인터럽트 플래그를 유지해서 다음 루프에서 다시 회피합니다.
    // 이 분기에서는 isAvoiding 도 true 로 유지해 다음 사이클에서도 인터럽트를 계속 무시합니다.
    if (obstacleSensorInterface->isFrontBlocked()) {
        std::cout << "[System] Still Blocked! Maintaining Flag..." << std::endl;
        return;
    }

    // 회피 성공 후 다시 인터럽트를 받을 수 있도록 플래그 초기화
    this->frontObstacleTriggered.store(false);

    // removeTimer 직전에 SIGALRM이 들어온 경우를 대비해 알람 플래그 초기화
    this->isAlarmSigExist.store(false);

    cleanerManager->cleanerMode(CleanerMode::ON);

    // 전진 + 청소 재개 시점에 회피 종료 처리: 이후 INTERRUPT 를 다시 받음
    this->isAvoiding.store(false);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void Controller::boosterOverHandler() {
    this->isAlarmSigExist.store(true);
}

void Controller::dustDetect() {
    while (onOff) {
        if (frontObstacleTriggered.load()) {
            avoidanceAction();
            continue;
        }

        if (this->isAlarmSigExist.load() && cleanerManager->isBoosterOn()) {
            cleanerManager->cleanerMode(CleanerMode::ON);
            this->isAlarmSigExist.store(false);
        }

        if (obstacleSensorInterface->isDustExistence() && onOff) {
            if (cleanerManager->isBoosterOn() == true) {
                cleanerManager->cleanerMode(CleanerMode::UP);
            } else {
                cleanerManager->cleanerMode(CleanerMode::UP);

                std::thread([this]() {
                    int caught_signal;

                    sigset_t wait_set;
                    sigemptyset(&wait_set);
                    sigaddset(&wait_set, SIGALRM);

                    std::cout << "Waiting for SIGALRM" << std::endl;

                    sigwait(&wait_set, &caught_signal);

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

    // 이전 전원 사이클에서 회피 도중 turnOff 된 경우를 대비해 회피 플래그 초기화
    this->isAvoiding.store(false);

    // 처음 turn on 되었을 때 정면 센서가 이미 threshold보다 작으면
    // interrupt가 발생하지 못할 수 있으므로 시작 시 한 번 체크합니다.
    if (obstacleSensorInterface->isFrontBlocked()) {
        this->frontObstacleTriggered.store(true);
    }

    cleanerManager->cleanerMode(CleanerMode::ON);
    driveManager->rotateForward();

    if (dustThread.joinable() && dustThread.get_id() != std::this_thread::get_id()) {
        dustThread.join();
    }

    dustThread = std::thread(&Controller::dustDetect, this);
}

void Controller::turnOff() {
    if (!onOff) return;

    std::cout << "[System] POWER OFF" << std::endl;
    onOff = false;

    // 회피 도중 전원이 꺼지더라도 다음 부팅에서 인터럽트가 영구 차단되지 않도록 초기화
    this->isAvoiding.store(false);

    if (dustThread.joinable() && dustThread.get_id() != std::this_thread::get_id()) {
        dustThread.join();
    }

    cleanerManager->cleanerMode(CleanerMode::OFF);
    driveManager->stopMotor();
}

void Controller::errorturnOff() {
    turnOff();
}


// --- ButtonInterface ---
ButtonInterface::ButtonInterface(Controller* ctrl) : controller(ctrl) {}

void ButtonInterface::pushButtonOn() {
    if (controller) controller->turnOn();
}

void ButtonInterface::pushButtonOff() {
    if (controller) controller->turnOff();
}
