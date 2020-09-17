#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <chrono>
#include <joy2serial/joystick_node.hpp>

void JoystickSubscriber::joy_callback_(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    //RCLCPP_INFO(this->get_logger(), "A : %d, B : %d, X : %d, Y : %d", msg->buttons[0], msg->buttons[1], msg->buttons[2], msg->buttons[3]);
    this->sd->Button1 = 0;
    this->sd->Button2 = 0;
    this->sd->CrossButton = 0;
    this->sd->LeftStick[0] = this->sd->LeftStick[1] = 0;
    this->sd->RightStick[0] = this->sd->RightStick[1] = 0;
    for(int i = 0; i < 2; i++){
        this->sd->LeftStick[i] = msg->axes[i] * 127 + 127;
        this->sd->RightStick[i] = msg->axes[i+3] * 127 + 127;
    }

    for(int i = 0; i < 8; i++){
        if(msg->buttons[i] == 1){
            this->sd->Button1 |= (1 << i);
        }
        if(msg->buttons[i+8] == 1){
            this->sd->Button2 |= (1 << i);
        }
    }


}

JoystickSubscriber::JoystickSubscriber(SendData *bts)
: Node("joystick_test_node")
{
    sd = bts;
    subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10,
        std::bind(&JoystickSubscriber::joy_callback_, this, std::placeholders::_1)
    );
}