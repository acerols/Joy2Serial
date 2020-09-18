#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <agent_msgs/msg/agent.hpp>
#include <okarobo_msgs/msg/sensor.hpp>

#include <robo2ros/robo_sub_pub_node.hpp>

#include <chrono>
#include <thread>

namespace OkaRobo{
RoboSubPub::RoboSubPub(
    const std::string& name_space,
    const rclcpp::NodeOptions& options
): Node("robo2ros", name_space, options)
{
    int out = 0;
    arduino = new Serial(SERIAL_PORT, 115200);

    if(arduino->isConnected()){
        out = 1;
    }

    RCLCPP_INFO(this->get_logger(), "Serial is %d", out);

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

    RCLCPP_INFO(this->get_logger(), "ussl : %d, ussr : %d", this->ussleft, this->ussright);

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
    //RCLCPP_INFO(this->get_logger(), "velocity %d", this->velocity);
    arduino->writeSerial(transmit, 12);
    /*
    using namespace std::chrono_literals;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    */
    RCLCPP_INFO(this->get_logger(), "is Recieved %d", _robo_recieve());

}

int RoboSubPub::_robo_recieve()
{
    uint8_t rxd[256];
    uint32_t readNum = arduino->available();
    if(readNum > 256){
        readNum = arduino->readSerial(rxd, 256);
    }
    else if(readNum > 0){
        readNum = arduino->readSerial(rxd, readNum);
    }
    else{
        return -2;
    }

    if(read_check(rxd, readNum)){
        return 1;
    }
    else{
        return -1;
    }

    return (read_check(rxd, readNum)) ? 1 : -1;
 
}

bool RoboSubPub::read_check(uint8_t data[], int32_t len)
{
    for(int i = 0; i < len; i++){
        if(data[i] == 0x64){
            uint8_t func = data[++i], size = data[++i];
            uint8_t checksum, checksumCulc = 0;
            uint16_t ussl, ussr, bsfr, bsle, bsri, bsre;
            //RCLCPP_INFO(this->get_logger(), "Func %x : Size %d", func, size);
            if(func == 0x01 && size == (uint8_t)12){
                checksumCulc ^= byte2short(&data[++i], ussl);
                checksumCulc ^= byte2short(&data[(i+=2)], ussr);
                checksumCulc ^= byte2short(&data[(i+=2)], bsfr);
                checksumCulc ^= byte2short(&data[(i+=2)], bsle);
                checksumCulc ^= byte2short(&data[(i+=2)], bsri);
                checksumCulc ^= byte2short(&data[(i+=2)], bsre);
                checksum = data[++i];
                RCLCPP_INFO(this->get_logger(), "Checksum recieve %x : Culc %x", data[i], checksumCulc);
                //if(checksumCulc == checksum){
                    ussleft = ussl;
                    ussright = ussr;
                    bsfront = bsfr;
                    bsleft = bsle;
                    bsright = bsri;
                    bsrear = bsre;
                    return true;
                //}
                i++;
            }
            i -= 2;
        }
    }
    return false;
}

void RoboSubPub::short2byte(int16_t src, uint8_t *dst)
{
    dst[0] = (uint8_t)(src & 0xff);
    dst[1] = (uint8_t)((src >> 8) & 0xff);
}

uint8_t RoboSubPub::byte2short(uint8_t *src, uint16_t &dst)
{
    uint8_t data1 = src[0], data2 = src[1];
    dst = (uint16_t)(((data2 << 8) & 0xff00) | (data1 & 0xff));
    return (data1 ^ data2);
}

}