#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int16.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using namespace dynamixel;

// Protocol version
#define PROTOCOL_VERSION      1.0

#define ADDR_TORQUE_ENABLE    24              // Torque Enable
#define ADDR_GOAL_VELOCITY    32              // Goal Velocity
#define DXL1_ID               11                // 모터 1 ID
#define DXL2_ID               12                // 모터 2 ID
#define BAUDRATE              1000000          // Baudrate
#define DEVICE_NAME           "/dev/ttyUSB0"   // Port name

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
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("Dynamixel_Control");

    portHandler = PortHandler::getPortHandler(DEVICE_NAME);
    packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!portHandler->openPort()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to open the port!");
        return 1;
    }

    if (!portHandler->setBaudRate(BAUDRATE)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to set the baudrate!");
        return 1;
    }

    uint8_t dxl_error = 0;
    int16_t dxl1_velocity = 1624;
    int16_t dxl2_velocity = 600;

    // Enable Torque for both motors
    packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);

    puts("Motors initialized.");
    puts("Motor 1 velocity: 200 , Motor 2 velocity: 1224");
    puts("'• u': Motor 1 velocity increases by 1");
    puts("'• j': Motor 1 velocity decreases by 1");
    puts("'• y': Motor 1 velocity increases by 100");
    puts("'• h': Motor 1 velocity decreases by 100");
    puts("'• i': Motor 2 velocity increases by 1");
    puts("'• k': Motor 2 velocity decreases by 1");
    puts("'• o': Motor 2 velocity increases by 100");
    puts("'• l': Motor 2 velocity decreases by 100");
    puts("   y u i o");
    puts("    h j k l");
    puts(" S to stop");

    while (rclcpp::ok()) {
        int key = getKey();

        switch (key) {
            case 'u':
                dxl1_velocity++;
                RCLCPP_INFO(node->get_logger(), "Motor 1 velocity increased by 1. Current velocity: %d", dxl1_velocity);
                break;
            case 'j':
                dxl1_velocity--;
                RCLCPP_INFO(node->get_logger(), "Motor 1 velocity decreased by 1. Current velocity: %d", dxl1_velocity);
                break;
            case 'y':
                dxl1_velocity += 100;
                RCLCPP_INFO(node->get_logger(), "Motor 1 velocity increased by 100. Current velocity: %d", dxl1_velocity);
                break;
            case 'h':
                dxl1_velocity -= 100;
                RCLCPP_INFO(node->get_logger(), "Motor 1 velocity decreased by 100. Current velocity: %d", dxl1_velocity);
                break;
            case 'i':
                dxl2_velocity++;
                RCLCPP_INFO(node->get_logger(), "Motor 2 velocity increased by 1. Current velocity: %d", dxl2_velocity);
                break;
            case 'k':
                dxl2_velocity--;
                RCLCPP_INFO(node->get_logger(), "Motor 2 velocity decreased by 1. Current velocity: %d", dxl2_velocity);
                break;
            case 'o':
                dxl2_velocity += 100;
                RCLCPP_INFO(node->get_logger(), "Motor 2 velocity increased by 100. Current velocity: %d", dxl2_velocity);
                break;
            case 'l':
                dxl2_velocity -= 100;
                RCLCPP_INFO(node->get_logger(), "Motor 2 velocity decreased by 100. Current velocity: %d", dxl2_velocity);
                break;
            case 's':
                dxl1_velocity = 0;
                dxl2_velocity = 0;
                RCLCPP_INFO(node->get_logger(), "Motor 1, 2 STOP!. Current Motor1 velocity: %d  Motor2 velocity: %d",dxl1_velocity, dxl2_velocity);
                break;
            default:
                break;
        }

        if (key != 0) {
            // Set goal velocities
            packetHandler->write2ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_VELOCITY, dxl1_velocity, &dxl_error);
            packetHandler->write2ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_VELOCITY, dxl2_velocity, &dxl_error);

            // Display current velocities
            RCLCPP_INFO(node->get_logger(), "Motor 1 Velocity: %d, Motor 2 Velocity: %d", dxl1_velocity, dxl2_velocity);
        }

        rclcpp::spin_some(node);
        usleep(10000); // Sleep for 10ms
    }

    // Disable Torque when shutting down
    packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);

    rclcpp::shutdown();

    return 0;
}
