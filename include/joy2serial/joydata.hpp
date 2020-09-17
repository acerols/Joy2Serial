#ifndef __JOYDATA_HPP__
#define __JOYDATA_HPP__
#include <stdint.h>
#include <vector>

struct CommandData{
    uint8_t magic;         //data 0xff
    uint8_t func;           //function select contoller command is 0x10
    uint8_t size;           //data size
    uint8_t Button1;        //Button0~7 data
    uint8_t Button2;        //Button8~15 data
    uint8_t CrossButton;    //CrossButtton index(b0:up, b2:right, b4:down, b6:left)
    uint8_t LeftStick[2];   //analogStickLeft index(0:Y, 1:X)
    uint8_t RightStick[2];  //analogStickRight index(0:Y, 1:X)
    uint8_t Key;            //Keyboard data
    uint8_t checksum;       //data checksum (without magic, func, size)
};

using SendData = struct CommandData;

#endif
