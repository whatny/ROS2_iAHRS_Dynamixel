#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int16.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

rclcpp::Node::SharedPtr node = nullptr;
using namespace dynamixel;

// Protocol version
#define PROTOCOL_VERSION      1.0


// Control table address
#define ADDR_TORQUE_ENABLE    24              // Torque Enable
#define ADDR_GOAL_VELOCITY    32              // Goal Velocity
#define DXL1_ID               1                // 모터 1 ID
#define DXL2_ID               5                // 모터 2 ID
#define BAUDRATE              1000000          // Baudrate
#define DEVICE_NAME           "/dev/ttyUSB0"   // Port name




#define VALUE                 100
#define OPERATING_VALUE       1               // 제어모드, 위치제어: 3, 속도제어: 1

PortHandler *portHandler;
PacketHandler *packetHandler;

void topic_callback(const std_msgs::msg::Int16::SharedPtr msg)
{
  uint8_t dxl_error = 0;
  packetHandler->write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_VELOCITY, msg->data,&dxl_error);
  RCLCPP_INFO(node->get_logger(), "velocity: %d", msg->data);
}


 int main(/*int argc, char **argv*/)
{
//   uint8_t dxl_error = 0;
//   rclcpp::init(argc, argv);
//   node = rclcpp::Node::make_shared("Dynamixel");

//   portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
//   packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

//   if(portHandler->openPort() == false)
//   {
//     RCLCPP_INFO(node->get_logger(),"Failed to open the port!");
//   }
//   if(portHandler->setBaudRate(BAUDRATE) == false)
//   {
//     RCLCPP_INFO(node->get_logger(),"Failed to set the baudrate!");
//   }

//   packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_OPERTAING_MODE, OPERATING_VALUE, &dxl_error);
//   packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
//   RCLCPP_INFO(node->get_logger(),"start setting");
//   packetHandler->write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_VELOCITY, VALUE);

//   auto subscription =
//     node->create_subscription<std_msgs::msg::Int16>("/cmd_velocty", 10, topic_callback);

//   rclcpp::spin(node);
//   packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);
//   rclcpp::shutdown();
  
//  return 0;
     }