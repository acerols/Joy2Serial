#include <rclcpp/rclcpp.hpp>
#include "joystick_node.hpp"
#include <boost/asio.hpp>
#include <chrono>
#include "joydata.hpp"

#include <thread>
#include <iostream>
#include <boost/asio.hpp>

#define SERIALPORT "/dev/ttyACM0"

const int BAUDRATE = 115200;

using namespace std;

int main(int argc, char *argv[])
{
    using namespace std::chrono_literals;
    rclcpp::init(argc, argv);
    SendData sd;

    auto node = std::make_shared<JoystickSubscriber>(&sd);

    boost::asio::io_service io;
    boost::asio::serial_port serial(io, SERIALPORT);
    serial.set_option(boost::asio::serial_port_base::baud_rate(BAUDRATE));
    
    std::this_thread::sleep_for(std::chrono::seconds(5));

    auto timer1 = node->create_wall_timer(
        100ms,
        [&](){
            boost::asio::streambuf response;
            boost::asio::read_until(serial, response, '\n');
            uint8_t Serialdata[13] = {0};
            Serialdata[0] = 0xff;
            Serialdata[1] = 0x10;
            Serialdata[2] = 0x08;
            Serialdata[3] = sd.Button1;
            Serialdata[4] = sd.Button2;
            Serialdata[5] = sd.CrossButton;
            Serialdata[6] = sd.LeftStick[1];
            Serialdata[7] = sd.LeftStick[0];
            Serialdata[8] = sd.RightStick[1];
            Serialdata[9] = sd.RightStick[0];
            Serialdata[10] = 0;
            uint8_t checksum;
            for(int ind = 0; ind < 8; ind++){
                checksum = checksum ^ Serialdata[ind+3];
            }
            Serialdata[11] = checksum;

            auto buffer = boost::asio::buffer(Serialdata);
            boost::asio::write(serial, buffer);

            RCLCPP_INFO(node->get_logger(), "Button1 : %d, Button2: %d\nLeftX  : %d, LeftY  : %d\nRightX : %d, RightY : %d\nchecksum : %d\n RES : %s", 
            sd.Button1, sd.Button2,
            sd.LeftStick[0], sd.LeftStick[1],
            sd.RightStick[0], sd.RightStick[1],
            checksum,
            boost::asio::buffer_cast<const char*>(response.data()));
            
        }
    );
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}