#include "simulator_interface.h"
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>
#include <mutex>
#include <iostream>
#include <cstring>

// 💡 1. 익명 네임스페이스: 외부(다른 cpp 파일)에서는 절대 접근할 수 없는 내부 전용 변수/함수들
namespace {
    // --- 소켓 및 스레드 관리 변수 ---
    int sock = -1;
    std::mutex sock_mtx;         // 기존 SimulatorBridge의 mtx 역할
    
    bool is_running = false;
    std::thread listener_thread;
    
    // --- 센서 및 콜백 관리 변수 ---
    std::mutex sensor_mtx;
    SensorData current_sensor_data = {127, 127, 127, 0};

    std::function<void()> on_obstacle_interrupt = nullptr;
    std::function<void(const std::string&)> on_button_event = nullptr;

    // --- JSON 파서 (사용자 제공 코드) ---
    std::string get_json_value(const std::string& json, const std::string& key) {
        size_t key_pos = json.find("\"" + key + "\"");
        if (key_pos == std::string::npos) return "";
        size_t colon_pos = json.find(":", key_pos);
        if (colon_pos == std::string::npos) return "";
        size_t start = json.find_first_not_of(" \t\n\r\"", colon_pos + 1);
        size_t end = json.find_first_of(",}", start);
        if (start == std::string::npos || end == std::string::npos) return "";
        std::string val = json.substr(start, end - start);
        if (!val.empty() && val.back() == '"') val.pop_back();
        return val;
    }

    // --- 소켓 연결 관리 (기존 SimulatorBridge::get_sock) ---
    int get_sock() {
        std::lock_guard<std::mutex> lock(sock_mtx);
        if (sock != -1) return sock; // 이미 연결되어 있으면 그대로 반환
        
        sock = socket(AF_INET, SOCK_STREAM, 0);
        struct sockaddr_in addr;
        addr.sin_family = AF_INET;
        addr.sin_port = htons(12345);
        inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
        
        if (connect(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) { 
            close(sock); 
            sock = -1; 
        } else {
            // 💡 [버그 픽스] 소켓 타임아웃 설정 추가 (데이터가 없어도 0.1초 뒤에 recv가 풀림)
            struct timeval tv;
            tv.tv_sec = 0;
            tv.tv_usec = 100000; // 100ms
            setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);
        }
        return sock;
    }

    // --- 내부 명령어 전송 함수 (기존 SimulatorBridge::send_cmd) ---
    void send_cmd(const std::string& cmd) {
        int s = get_sock();
        if (s != -1) {
            send(s, (cmd + "\n").c_str(), cmd.length() + 1, 0);
        }
    }

    // --- 백그라운드 리스너 스레드 (기존 ObstacleSensorInterface::listen) ---
    void background_listener() {
        std::cout << "[Simulator] Listening for Hardware Interrupts..." << std::endl;
        std::string buffer = "";
        
        while (is_running) {
            int current_sock = get_sock();
            if (current_sock == -1) { 
                std::this_thread::sleep_for(std::chrono::seconds(1)); 
                continue; 
            }
            
            char chunk[4096] = {0};
            int valread = recv(current_sock, chunk, 4096, 0);
            
            // 💡 통신 끊김 처리 (valread <= 0): 소켓을 닫고 -1로 만들면 다음 루프에서 get_sock()이 재연결 시도
            if (valread <= 0) { 
                std::lock_guard<std::mutex> lock(sock_mtx);
                close(sock);
                sock = -1;
                std::this_thread::sleep_for(std::chrono::milliseconds(50)); 
                continue; 
            }
            
            buffer += std::string(chunk, valread);
            
            size_t pos;
            while ((pos = buffer.find('\n')) != std::string::npos) {
                std::string msg = buffer.substr(0, pos);
                buffer.erase(0, pos + 1);
                if (msg.empty()) continue;

                std::string type = get_json_value(msg, "type");
                
                if (type == "INTERRUPT") {
                    if (on_obstacle_interrupt) on_obstacle_interrupt();
                } 
                else if (type == "EVENT") {
                    std::string name = get_json_value(msg, "name");
                    std::cout << "[Simulator] Received Event: " << name << std::endl;
                    if (on_button_event) on_button_event(name);
                } 
                else { // 센서 값 갱신
                    std::string front_val = get_json_value(msg, "front");
                    std::string left_val  = get_json_value(msg, "left");
                    std::string right_val = get_json_value(msg, "right");
                    std::string dust_val  = get_json_value(msg, "dust");
                    
                    std::lock_guard<std::mutex> lock(sensor_mtx);
                    if (!front_val.empty()) current_sensor_data.front = std::stoi(front_val);
                    if (!left_val.empty())  current_sensor_data.left  = std::stoi(left_val);
                    if (!right_val.empty()) current_sensor_data.right = std::stoi(right_val);
                    if (!dust_val.empty())  current_sensor_data.dust  = std::stoi(dust_val);
                }
            }
        }
    }
} // namespace 끝

// 💡 2. 외부로 노출되는 Simulator API 구현부
namespace Simulator {

    bool init() {
        is_running = true;
        listener_thread = std::thread(background_listener);
        return true;
    }

    void stop() {
        is_running = false;
        if (listener_thread.joinable()) {
            listener_thread.join();
        }
        std::lock_guard<std::mutex> lock(sock_mtx);
        if (sock != -1) {
            close(sock);
            sock = -1;
        }
    }

    void sendDriveCommand(Driving state) {
        std::string is_forward = (state == Driving::MOVEFORWARD) ? "true" : "false";
        std::string is_backward = (state == Driving::MOVEBACKWARD) ? "true" : "false";
        std::string turn_str = "0";

        if (state == Driving::TURNLEFT) turn_str = "-1";
        else if (state == Driving::TURNRIGHT) turn_str = "1";

        std::string cmd = "{\"type\": \"SET_CONTROL\", \"move\": " + is_forward + 
                          ", \"backward\": " + is_backward +
                          ", \"turn\": " + turn_str + "}";
        send_cmd(cmd); // ⬅️ 내부 함수로 명령어 전송!
    }

    void sendCleanerCommand(CleanerMode mode) {
        std::string m = (mode == CleanerMode::OFF) ? "OFF" : (mode == CleanerMode::ON ? "ON" : "UP");
        std::string cmd = "{\"type\": \"SET_CONTROL\", \"clean\": " + 
                          std::string(mode != CleanerMode::OFF ? "true" : "false") + 
                          ", \"mode\": \"" + m + "\"}";
        send_cmd(cmd); // ⬅️ 내부 함수로 명령어 전송!
    }

    SensorData getSensorData() {
        // 1. 센서 최신값 요청
        send_cmd("{\"type\": \"GET_SENSORS\"}");
        
        // 2. 백그라운드 스레드가 값을 받아올 때까지 잠깐 대기 (기존 로직 유지)
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        
        // 3. 안전하게 값 반환
        std::lock_guard<std::mutex> lock(sensor_mtx);
        return current_sensor_data;
    }

    void registerObstacleInterruptCallback(std::function<void()> callback) {
        on_obstacle_interrupt = callback;
    }

    void registerButtonEventCallback(std::function<void(const std::string&)> callback) {
        on_button_event = callback;
    }
}