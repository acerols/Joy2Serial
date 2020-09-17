#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <agent_msgs/msg/agent.hpp>
#include <okarobo_msgs/msg/sensor.hpp>

#include <robo2ros/robo_sub_pub_node.hpp>

namespace OkaRobo{
RoboSubPub::RoboSubPub(
    const std::string& name_space,
    const rclcpp::NodeOptions& options
): Node("robo2ros", name_space, options)
{
    arduino = new Serial(SERIAL_PORT, 115200);

    agentorder_sub_ = this->create_subscription<okarobo_msgs::msg::Sensor>(
        "agnet",
        rclcpp::QoS(10),
        std::bind(&RoboSubPub::_agentorder_callback, this, std::placeholders::_1)
    );
    sensor_pub_ = this->create_publisher<okarobo_msgs::msg::Sensor>(
        "robo_sensor",
        rclcpp::QoS(10)
    );
    
    pub_timer_ = this->create_wall_timer(
        50ms,
        std::bind(&RoboSubPub::_sensor_callback, this)
    );
    serial_timer_ = this->create_wall_timer(
        50ms,
        std::bind(&RoboSubPub::_serial_callback, this)
    );

}

void RoboSubPub::_agentorder_callback(const agent_msgs::msg::Agent::SharedPtr Agent)
{
    this->velocity = Agent->velocity;
    this->omega = Agent->omega;
    this->nowAngle = Agent->nowangle;
    this->targetAngle = Agent->targetangle;
}

void RoboSubPub::_sensor_callback()
{
    auto Sensor = std::make_shared<okarobo_msgs::msg::Sensor>();

    Sensor->ussl = ussleft;
    Sensor->ussr = ussright;
    Sensor->bsfront = bsfront;
    Sensor->bsleft = bsleft;
    Sensor->bsright = bsright;
    Sensor->bsrear = bsrear;

    sensor_pub_->publish(*Sensor);

}  

void RoboSubPub::_serial_callback()
{
    uint8_t checksum = 0;
    uint8_t transmit[12] = {0};

    transmit[0] = 0xff;
    transmit[1] = 0x20;
    transmit[2] = 0x08;
    short2byte(velocity, &transmit[3]);
    short2byte(omega, &transmit[5]);
    short2byte(nowAngle, &transmit[7]);
    short2byte(targetAngle, &transmit[9]);
    for(int index = 0; index < 8; index++){
        checksum ^= transmit[index+3];
    }
    transmit[11] = checksum;
    RCLCPP_INFO(this->get_logger(), "velocity %d", this->velocity);
    arduino->writeSerial(transmit, 12);
    
}

void RoboSubPub::short2byte(int16_t src, uint8_t *dst)
{
    dst[0] = (uint8_t)(src & 0xff);
    dst[1] = (uint8_t)((src >> 8) & 0xff);
}

}