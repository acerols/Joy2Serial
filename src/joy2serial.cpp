#include <rclcpp/rclcpp.hpp>
#include "joystick_node.hpp"
#include "serial.hpp"
#include "joydata.hpp"

#include <chrono>
#include <thread>
#include <iostream>

#define SERIALPORT "/dev/ttyACM0"
const int COM_SIZE = 12;

const int BAUDRATE = 115200;

using namespace std;

uint32_t unoData[13];

int recdatacheck(uint8_t *rdata, uint8_t rsize)
{
    int readIndex = 0;
    uint8_t buf1, buf2;
    for(; readIndex < rsize - 12; readIndex++){
        if(rdata[readIndex] == 0x64){
            uint8_t checksum = 0, ChecksumCulc = 0;
            int i = readIndex + 1;
            if(rdata[i] == 0x01){
                buf1 = rdata[++i];
                buf2 = rdata[++i];
                unoData[0] = ((buf2 << 8) & 0xff00) | (buf1 & 0xff);
                ChecksumCulc ^= buf1;
                ChecksumCulc ^= buf2;
                buf1 = rdata[++i];
                buf2 = rdata[++i];
                unoData[1] = ((buf2 << 8) & 0xff00) | (buf1 & 0xff);
                ChecksumCulc ^= buf1;
                ChecksumCulc ^= buf2;
                buf1 = rdata[++i];
                buf2 = rdata[++i];
                unoData[2] = ((buf2 << 8) & 0xff00) | (buf1 & 0xff);
                ChecksumCulc ^= buf1;
                ChecksumCulc ^= buf2;
                buf1 = rdata[++i];
                buf2 = rdata[++i];
                unoData[3] = ((buf2 << 8) & 0xff00) | (buf1 & 0xff);
                ChecksumCulc ^= buf1;
                ChecksumCulc ^= buf2;
                buf1 = rdata[++i];
                buf2 = rdata[++i];
                unoData[4] = ((buf2 << 8) & 0xff00) | (buf1 & 0xff);
                ChecksumCulc ^= buf1;
                ChecksumCulc ^= buf2;
                buf1 = rdata[++i];
                buf2 = rdata[++i];
                unoData[5] = ((buf2 << 8) & 0xff00) | (buf1 & 0xff);
                ChecksumCulc ^= buf1;
                ChecksumCulc ^= buf2;
                checksum = rdata[++i];
                if(checksum == ChecksumCulc){
                    return 12;
                }
            }
        }
    }
    return -1;
}

int main(int argc, char *argv[])
{
    using namespace std::chrono_literals;
    rclcpp::init(argc, argv);
    SendData sd;

    auto node = std::make_shared<JoystickSubscriber>(&sd);

    SerialInit(SERIALPORT, BAUDRATE);

    std::this_thread::sleep_for(std::chrono::seconds(2));

    auto timer1 = node->create_wall_timer(
        100ms,
        [&](){
            uint8_t checksum = 0;
            uint8_t rsize = 0;
            uint8_t rdata[256] = {0};
            uint8_t Serialdata[12] = {0};
            int readAvailable = 0;
            Serialdata[0] = 0xff;
            Serialdata[1] = 0x10;
            Serialdata[2] = 0x08;
            Serialdata[3] = sd.Button1;
            Serialdata[4] = sd.Button2;
            Serialdata[5] = sd.CrossButton;
            Serialdata[6] = sd.LeftStick[0];
            Serialdata[7] = sd.LeftStick[1];
            Serialdata[8] = sd.RightStick[0];
            Serialdata[9] = sd.RightStick[1];
            Serialdata[10] = 0;   
            for(int ind = 0; ind < 8; ind++){
                checksum = checksum ^ Serialdata[ind+3];
            }
            Serialdata[11] = checksum;

            writePort(Serialdata, COM_SIZE);

            ioctl(serial_fd, FIONREAD, &readAvailable);
            if(readAvailable > 0){
                if(readAvailable > 256){
                    rsize = readPort(rdata, 256);
                    ioctl(serial_fd, FIONREAD, &readAvailable);
                    readPort(NULL, readAvailable);
                }
                else{
                    rsize = readPort(rdata, readAvailable);
                }
            }

            int isGet = recdatacheck(rdata, rsize);

            if(isGet > 0){
                RCLCPP_INFO(node->get_logger(), "Button1 : %d, Button2: %d\nLeftX  : %d, LeftY  : %d\nRightX : %d, RightY : %d\nchecksum : %d\n RES : %s", 
                    sd.Button1, sd.Button2,
                    sd.LeftStick[0], sd.LeftStick[1],
                    sd.RightStick[0], sd.RightStick[1],
                    checksum,
                    "Received");
            }
            else{
                RCLCPP_INFO(node->get_logger(), "Button1 : %d, Button2: %d\nLeftX  : %d, LeftY  : %d\nRightX : %d, RightY : %d\nchecksum : %d\n RES : %s", 
                    sd.Button1, sd.Button2,
                    sd.LeftStick[0], sd.LeftStick[1],
                    sd.RightStick[0], sd.RightStick[1],
                    checksum,
                    "Failed to Received");
            }
        }
    );
    rclcpp::spin(node);
    closePort();
    rclcpp::shutdown();
    return 0;
}