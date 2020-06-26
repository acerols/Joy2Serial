#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "joydata.hpp"

class JoystickSubscriber : public rclcpp::Node{
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    void joy_callback_(const sensor_msgs::msg::Joy::SharedPtr msg);
    SendData *sd;

public:
    JoystickSubscriber(SendData *bts);
};