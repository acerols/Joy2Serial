#include <serial.hpp>

Serial::Serial(const char *port, int baudrate)
{
    // https://mcommit.hatenadiary.com/entry/2017/07/09/210840
	// こちら参照コード
	
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
	
	serial_fd = open(port, O_RDWR|O_NOCTTY|O_NONBLOCK);	// ポートオープンの仕方が重要
     if (serial_fd < 0) {
         printf("open error\n");
         serial_fd = -1;
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
        std::cerr << "serial device setting(tcsetattr) error" << std::endl;
 		serial_fd = -1;
 	}

    error_flag = ioctl(serial_fd, TCSETS, &tio);            // ポートの設定を有効にする
    if (error_flag== -1 ){ 
 	    std::cerr << "serial device setting(ioctl) error" << std::endl;;
        serial_fd = -1;
 	}

 	error_flag = tcflush(serial_fd,TCIFLUSH);				// 入力バッファクリア（追加）
    if (error_flag == -1 ){ 
 	 	std::cerr << "serial device setting(tcflush) error" << std::endl;
 		serial_fd =  -1;
 	}

}

bool Serial::closePort()
{
    return close(serial_fd);
}

int Serial::writeSerial(uint8_t *data, int size)
{
    return write(serial_fd, data, size);
}

int Serial::readSerial(uint8_t *data, int size)
{
    return read(serial_fd, data, size);
}

int Serial::available()
{
    int readnum;
    ioctl(serial_fd, FIONREAD, &readnum);
    return readnum;
}

bool Serial::isConnected()
{
    return (serial_fd >= 0) ? true : false;
}
