#pragma once

// 로봇 상태 관련 enum
enum class Driving { STOP, MOVEFORWARD, MOVEBACKWARD, TURNLEFT, TURNRIGHT };
enum class CleanerMode { OFF, ON, UP };
enum class Location {Forward, LEFT, REAR, Clockwise, CounterClockwise }; // Location도 있다면 여기에!

// 센서 상태 구조체
struct ObstacleStatus {
    bool front;
    bool left;
    bool right;
};

struct SensorData {
    int front;
    int left;
    int right;
    int dust;
};