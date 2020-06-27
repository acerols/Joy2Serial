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

int serial_fd;
char flag = 0;
double tx_time;

bool SerialInit(const char *serialport, int baudrate)
{
    	// // https://mcommit.hatenadiary.com/entry/2017/07/09/210840
	// // こちら参照コード
	
	int error_flag = 0;
	struct termios tio;    
    speed_t baud;
    
	if(baudrate==9600) baud=B9600;
    if(baudrate==38400) baud=B38400;
    if(baudrate==57600) baud=B57600;
	if(baudrate==115200) baud=B115200;
    if(baudrate==230400) baud=B230400;
    if(baudrate==460800) baud=B460800;

	bzero(&tio, sizeof(tio)); // clear struct for new port settings
	
	serial_fd = open(serialport, O_RDWR|O_NOCTTY|O_NONBLOCK);	// ポートオープンの仕方が重要
     if (serial_fd < 0) {
         printf("open error\n");
         return -1;
     }

     tio.c_cflag += CREAD;               // 受信有効
     tio.c_cflag += CLOCAL;              // ローカルライン（モデム制御なし）
     tio.c_cflag += CS8;                 // データビット:8bit
     tio.c_cflag += 0;                   // ストップビット:1bit
     tio.c_cflag += 0;                   // パリティ:None

    cfsetispeed( &tio, baud );
    cfsetospeed( &tio, baud );

	tio.c_oflag      = 0;
	tio.c_lflag      = 0;
	tio.c_cc[VTIME]  = 0;
	tio.c_cc[VMIN]   = 0;

	cfmakeraw(&tio);                    // RAWモード

 	error_flag = tcsetattr(serial_fd, TCSANOW, &tio ); // デバイスに設定を行う (成功したら0を返す)
     if (error_flag){ 
 		printf("serial device setting(tcsetattr) error\n");
 		return -1;
 	}

     error_flag = ioctl(serial_fd, TCSETS, &tio);            // ポートの設定を有効にする
     if (error_flag== -1 ){ 
 		printf("serial device setting(ioctl) error\n");
 		return -1;
 	}

 	error_flag = tcflush(serial_fd,TCIFLUSH);				// 入力バッファクリア（追加）
    if (error_flag == -1 ){ 
 	 	printf("serial device setting(tcflush) error\n");
 		return -1;
 	}

 	flag = 1;	// フラグを立てる
	tx_time = (1000.0 / (double)baudrate) * 10.0;

    return 0;
}

bool closePort()
{
    return close(serial_fd);
}

int writePort(uint8_t *data, int size)
{
    return write(serial_fd, data, size);
}

int readPort(uint8_t *data, int size)
{
    return read(serial_fd, data, size);
}
/*
int main(int argc, char *argv[])
{
    uint8_t checksum = 0;
    uint8_t rdata[256];
    int rsize;
    int avalales = 0;
    SerialInit(SERIAL_PORT, 115200);
    uint8_t data[13] = {0xff, 0x10, 0x08, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01};
    for(int i  = 0; i < data[2]; i++){
        checksum = checksum ^ data[i+3];
    }
    data[data[2] + 3] = checksum;
    cout << (int)checksum << endl;
    for(int i = 0; i < 20; i++){
        writePort(data, 12);
        this_thread::sleep_for(chrono::milliseconds(100));
    }

    ioctl(serial_fd, FIONREAD, &avalales);
    if(avalales > 0){
        rsize = readPort(rdata, avalales);
        cout << rdata << endl;
    }
    cout << "end" << endl;


    return 0;
}
*/