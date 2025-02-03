#include <chrono>
#include <memory>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/fcntl.h>
#include <time.h>
#include <sys/types.h>
#include <vector>
#include <iostream>
#include <dirent.h>
#include <signal.h>
#include <algorithm>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace dynamixel;

#define PROTOCOL_VERSION      1.0
#define ADDR_TORQUE_ENABLE    24
#define ADDR_GOAL_VELOCITY    32
#define DXL1_ID               3              // 모터 1 ID
#define DXL2_ID               5              // 모터 2 ID
#define BAUDRATE              1000000
#define DEVICE_NAME           "/dev/ttyUSB0"

// Dynamixel SDK 객체
PortHandler *portHandler;
PacketHandler *packetHandler;

class IMUMotorController : public rclcpp::Node
{
public:
    IMUMotorController() : Node("dynamixel_IMU")
    {
        // IMU 데이터를 구독
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", rclcpp::SensorDataQoS(),
    std::bind(&IMUMotorController::imu_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Subscribed to /imu/data");

        // Dynamixel 설정
        portHandler = PortHandler::getPortHandler(DEVICE_NAME);
        packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

        if (!portHandler->openPort()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the port!");
            rclcpp::shutdown();
        }

        if (!portHandler->setBaudRate(BAUDRATE)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set the baudrate!");
            rclcpp::shutdown();
        }

        uint8_t dxl_error = 0;

        // Torque Enable
        packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
        packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);

        RCLCPP_INFO(this->get_logger(), "Motor controller initialized.");
    }

    ~IMUMotorController()
    {
        uint8_t dxl_error = 0;

        // Torque Disable
        packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);
        packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);

        portHandler->closePort();
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "IMU Callback triggered");


        if (!msg) {
            RCLCPP_ERROR(this->get_logger(), "Received NULL IMU data!");
            return;
        }
        
        
        if (msg == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "Received NULL IMU data!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "IMU Callback invoked with data");

        // IMU 데이터를 가져와 Roll과 Pitch 계산
        double roll, pitch, yaw;
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);

        RCLCPP_INFO(this->get_logger(), "IMU Data Received - Roll: %.2f, Pitch: %.2f, Yaw: %.2f", roll, pitch, yaw);

        //RCLCPP_INFO(this->get_logger(), "Publishing IMU Data: orientation=(%.2f, %.2f, %.2f, %.2f)",imu_data_msg.orientation.x,imu_data_msg.orientation.y,imu_data_msg.orientation.z,imu_data_msg.orientation.w);


        // 모터 속도 초기값
        int16_t motor1_speed = 200;   // DXL1_ID 기본 속도
        int16_t motor2_speed = 1224; // DXL2_ID 기본 속도

        // Roll에 따른 속도 조정 (왼쪽 기울이면 모터1 속도 증가)
        motor1_speed += static_cast<int16_t>(roll / 0.05);  // 부호 반대로 변경
        motor2_speed -= static_cast<int16_t>(roll / 0.05);  // 부호 반대로 변경

        // Pitch에 따른 속도 조정 (앞으로 기울이면 속도 감소)
        motor1_speed -= static_cast<int16_t>(pitch / 0.05);  // 부호 반대로 변경
        motor2_speed -= static_cast<int16_t>(pitch / 0.05);  // 부호 반대로 변경


        RCLCPP_INFO(this->get_logger(), "Calculated Speeds - Motor1: %d, Motor2: %d", motor1_speed, motor2_speed);

        // 모터 속도 설정
        uint8_t dxl_error = 0;
        packetHandler->write2ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_VELOCITY, motor1_speed, &dxl_error);
        packetHandler->write2ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_VELOCITY, motor2_speed, &dxl_error);

        RCLCPP_INFO(this->get_logger(), "Motors updated.");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initializing the IMU Motor Controller...");

    auto node = std::make_shared<IMUMotorController>();

    // 디버깅: IMU 구독 및 콜백 연결 상태 확인
    RCLCPP_INFO(node->get_logger(), "Node initialized. Waiting for IMU data...");

    RCLCPP_INFO(node->get_logger(), "Node is spinning...");
    rclcpp::spin(node);
    RCLCPP_INFO(node->get_logger(), "Spin terminated.");


    RCLCPP_INFO(node->get_logger(), "Shutting down IMU Motor Controller.");
    rclcpp::shutdown();

    return 0;
}

