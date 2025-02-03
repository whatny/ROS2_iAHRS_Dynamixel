#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int16.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

rclcpp::Node::SharedPtr node = nullptr;
using namespace dynamixel;

// Protocol version
#define PROTOCOL_VERSION      2.0             // Default Protocol version of DYNAMIXEL X series.

// Control table address
#define ADDR_OPERTAING_MODE   11
#define ADDR_TORQUE_ENABLE    64              // Torque on/off 제어
#define ADDR_GOAL_VELOCITY    104             // 속도 제어
#define ADDR_GOAL_POSITION    116             // 위치 제어
#define ADDR_PRESENT_POSITION 132             // 현재 위치


// Default setting
#define DXL1_ID               1               // DXL1 ID
#define DXL2_ID               2               // DXL2 ID
#define BAUDRATE              1000000           // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME           "/dev/ttyUSB0"  


#define OPERATING_VALUE       3               // 제어모드, 위치제어: 3, 속도제어: 1

PortHandler *portHandler;
PacketHandler *packetHandler;

int getKey() {
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF) {
        return ch;
    }

    return 0;
}


int main(int argc, char **argv)
{
  uint8_t dxl_error = 0;
  int16_t dxl1_position = 100;
  int16_t dxl2_position = 100;
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("Dynamixel");

  portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if(portHandler->openPort() == false)
  {
    RCLCPP_INFO(node->get_logger(),"Failed to open the port!");
  }
  if(portHandler->setBaudRate(BAUDRATE) == false)
  {
    RCLCPP_INFO(node->get_logger(),"Failed to set the baudrate!");
  }

  packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_OPERTAING_MODE, OPERATING_VALUE, &dxl_error);
  packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_OPERTAING_MODE, OPERATING_VALUE, &dxl_error);
  
  packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);


  puts("Motors initialized.");
  puts("Motor 1 position: 100 , Motor 2 position: 100");
  puts("'• u': Motor 1 position increases by 1");
  puts("'• j': Motor 1 position decreases by 1");
  puts("'• y': Motor 1 position increases by 100");
  puts("'• h': Motor 1 position decreases by 100");
  puts("'• i': Motor 2 position increases by 1");
  puts("'• k': Motor 2 position decreases by 1");
  puts("'• o': Motor 2 position increases by 100");
  puts("'• l': Motor 2 position decreases by 100");
  puts("   y u i o");
  puts("    h j k l");

while (rclcpp::ok()) {
    int key = getKey();

    switch (key) {
            case 'u':
                dxl1_position++;
                RCLCPP_INFO(node->get_logger(), "Motor 1 position increased by 1. Current position: %d", dxl1_position);
                break;
            case 'j':
                dxl1_position--;
                RCLCPP_INFO(node->get_logger(), "Motor 1 position decreased by 1. Current position: %d", dxl1_position);
                break;
            case 'y':
                dxl1_position += 100;
                RCLCPP_INFO(node->get_logger(), "Motor 1 position increased by 100. Current position: %d", dxl1_position);
                break;
            case 'h':
                dxl1_position -= 100;
                RCLCPP_INFO(node->get_logger(), "Motor 1 position decreased by 100. Current position: %d", dxl1_position);
                break;
            case 'i':
                dxl2_position++;
                RCLCPP_INFO(node->get_logger(), "Motor 2 position increased by 1. Current position: %d", dxl2_position);
                break;
            case 'k':
                dxl2_position--;
                RCLCPP_INFO(node->get_logger(), "Motor 2 position decreased by 1. Current position: %d", dxl2_position);
                break;
            case 'o':
                dxl2_position += 100;
                RCLCPP_INFO(node->get_logger(), "Motor 2 position increased by 100. Current position: %d", dxl2_position);
                break;
            case 'l':
                dxl2_position -= 100;
                RCLCPP_INFO(node->get_logger(), "Motor 2 position decreased by 100. Current position: %d", dxl2_position);
                break;
            default:
                break;
        }  
        if (key != 0) {
            // Set goal velocities
            packetHandler->write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_POSITION, dxl1_position, &dxl_error);
            packetHandler->write4ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_POSITION, dxl2_position, &dxl_error);

            // Display current velocities
            RCLCPP_INFO(node->get_logger(), "Motor 1 position: %d, Motor 2 position: %d", dxl1_position, dxl2_position);
        }

        rclcpp::spin_some(node);
        usleep(10000); // Sleep for 10ms
    }
  
    
  rclcpp::spin(node);
  
  packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);
  packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);
  
  rclcpp::shutdown();
  
 return 0;
}