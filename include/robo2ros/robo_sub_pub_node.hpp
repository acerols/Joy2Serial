#ifndef __ROBO_SUB_PUB_NODE_HPP__
#define __ROBO_SUB_PUB_NODE_HPP__

#include <rclcpp/rclcpp.hpp>
#include <agent_msgs/msg/agent.hpp>
#include <okarobo_msgs/msg/sensor.hpp>
#include <serial.hpp>

#define SERIAL_PORT "/dev/ttyACM0"

namespace OkaRobo{
class RoboSubPub : public rclcpp::Node{
private:
    rclcpp::Subscription<agent_msgs::msg::Agent>::SharedPtr agentorder_sub_;
    rclcpp::Publisher<okarobo_msgs::msg::Sensor>::SharedPtr sensor_pub_;
    void _agentorder_callback(const agent_msgs::msg::Agent::SharedPtr Agent);
    void _sensor_callback();
    void _serial_callback();
    rclcpp::TimerBase::SharedPtr pub_timer_;
    rclcpp::TimerBase::SharedPtr serial_timer_;
    void short2byte(int16_t src, uint8_t *dst);

    Serial *arduino;
    //send to Robo data
    int16_t velocity;
    int16_t omega;
    int16_t nowAngle;
    int16_t targetAngle;

    //Recieve from Robo data
    uint16_t ussleft;
    uint16_t ussright;
    uint16_t bsfront;
    uint16_t bsleft;
    uint16_t bsright;
    uint16_t bsrear;

public:
    
    RoboSubPub(
        const std::string& name_space="",
        const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
    );

};
}

#endif