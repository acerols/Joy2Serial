#ifndef __SERIAL_HPP__
#define __SERIAL_HPP__

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <strings.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>
#include <time.h>
#include <sys/ioctl.h>
#include <linux/serial.h>

#include <iostream>
#include <thread>
#include <chrono>

using namespace std;

#define SERIAL_PORT "/dev/ttyACM0"

class Serial{
private:
    int serial_fd;

public:
    Serial(const char *port, int baudrate);
    int writeSerial(uint8_t *data, int size);
    int readSerial(uint8_t *data, int size);
    bool closePort();
    int available();
    bool isConnected();
};

#endif
