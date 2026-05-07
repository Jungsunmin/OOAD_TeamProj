#pragma once

#include <string>
#include <functional>
#include "../src/common_types.h"



namespace Simulator {


    // --- 통신 및 스레드 관리 ---
    // 시뮬레이터와의 소켓 연결을 맺고 백그라운드 수신 스레드를 시작합니다.
    bool init();
    
    // 수신 스레드를 종료하고 소켓 연결을 안전하게 닫습니다.
    void stop();

    // --- 제어(Action) API ---
    // 모터의 구동 상태(전진, 후진, 좌회전, 우회전, 정지)를 변경합니다.
    void sendDriveCommand(Driving state);
    
    // 청소 모듈의 상태(OFF, ON, UP(부스터))를 변경합니다.
    void sendCleanerCommand(CleanerMode mode);

    // --- 센서(Sensor) API ---
    // 시뮬레이터에 센서값을 요청하고 최신 데이터를 반환받습니다.
    SensorData getSensorData();

    // --- 인터럽트 및 이벤트 콜백 등록 API ---
    // 장애물(INTERRUPT) 발생 시 호출될 함수를 예약해둡니다.
    void registerObstacleInterruptCallback(std::function<void()> callback);
    
    // 버튼(EVENT) 클릭 시 호출될 함수를 예약해둡니다.
    void registerButtonEventCallback(std::function<void(const std::string&)> callback);

} // namespace Simulator