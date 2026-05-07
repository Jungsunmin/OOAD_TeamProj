#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>
#include "rvc_controller.h"
#include "../simulator/simulator_interface.h" // 경로에 맞게 수정해주세요

int main() {
    std::cout << "RVC Software System Starting..." << std::endl;
    std::cout << "Connecting to Simulator on localhost:12345..." << std::endl;

    // 시그널 마스킹
    sigset_t my_sigset;
    sigemptyset(&my_sigset);
    sigaddset(&my_sigset, SIGALRM);
    pthread_sigmask(SIG_BLOCK, &my_sigset, nullptr);

    // 시뮬레이터 통신 스레드 가동
    if (!Simulator::init()) {
        std::cerr << "[Error] Failed to initialize Simulator Interface!" << std::endl;
        return -1;
    }

    ObstacleSensorInterface osi; //여기가 문제였음. 일단 obsSI로 바꿈.
    
    PathPlanner planner(&o브si);
    DriveManager driver(&planner);
    CleanerManager cleaner;
    
    // 4. 중앙 컨트롤러 및 외부 입력(버튼) 객체 생성
    Controller controller(&driver, &cleaner, &osi);
    ButtonInterface button(&controller);

    // 💡 5. 시뮬레이터의 버튼 이벤트를 ButtonInterface와 연결
    // 시뮬레이터 UI에서 버튼을 누르면, 실제 하드웨어 버튼(ButtonInterface)을 누른 것처럼 맵핑해줍니다.
    Simulator::registerButtonEventCallback([&button](const std::string& btn_name) {
        if (btn_name == "BUTTON_ON") {
            button.pushButtonOn();
        } else if (btn_name == "BUTTON_OFF") {
            button.pushButtonOff();
        }
    });

    std::cout << "System Ready. Please press POWER ON in the Simulator UI." << std::endl;

    // 6. 메인 스레드 유지 (백그라운드 통신/센서 스레드가 일하도록 프로그램 종료 방지)
    // 추후 'exit' 입력 시 while문을 빠져나가게 수정할 수도 있습니다.
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // 💡 7. 프로그램 종료 시 안전하게 자원 정리 (현재는 무한루프라 도달하지 않지만 안전을 위해 작성)
    Simulator::stop();
    
    return 0;
}