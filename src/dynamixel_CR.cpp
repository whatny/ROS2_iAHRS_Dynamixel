#include <rclcpp/rclcpp.hpp>
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace dynamixel;

#define PROTOCOL_VERSION      1.0
#define ADDR_TORQUE_ENABLE    24              
#define ADDR_GOAL_VELOCITY    32              
#define DXL1_ID               1                // 모터 1 ID
#define DXL2_ID               5                // 모터 2 ID
#define BAUDRATE              1000000          
#define DEVICE_NAME           "/dev/ttyACM0"  

PortHandler *portHandler;
PacketHandler *packetHandler;

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

  // Enable Torque
  packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  
  // Set maximum velocity (1023 is the maximum value for EX-106+ in velocity control mode)
  packetHandler->write2ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_VELOCITY, 1023, &dxl_error);   // 모터 1: 최대 속도
  packetHandler->write2ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_VELOCITY, 1023, &dxl_error);   // 모터 2: 최대 속도
  
  RCLCPP_INFO(node->get_logger(), "Motors are set to rotate at maximum velocities.");

  rclcpp::spin(node);

  // Disable Torque
  packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);
  packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);

  rclcpp::shutdown();
  
  return 0;
}
