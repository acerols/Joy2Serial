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