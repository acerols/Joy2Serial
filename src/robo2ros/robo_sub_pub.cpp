#include <rclcpp/rclcpp.hpp>
#include <robo2ros/robo_sub_pub_node.hpp>
#include "serial.hpp"

#include <chrono>
#include <thread>
#include <iostream>

#define SERIALPORT "/dev/ttyACM0"
const int COM_SIZE = 12;

const int BAUDRATE = 115200;

using namespace std;
using namespace OkaRobo;

int main(int argc, char *argv[])
{
    
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RoboSubPub>();
    if(!node->arduino->isConnected()){
        return 0;
    }
    std::cout << node->arduino->isConnected();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}