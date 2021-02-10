#include "SerialCom.h"

#include "stdio.h"
#include "stdlib.h"

// Open port function
// Initial attribute
//  8 bits - data size
//  1 bit - stop bit
//  Allow receive
//  Disable parity
//  ignore only the cd signal
bool SerialCom::set_port(std::string s_portname)
{
    // Open port
    _fd = open(s_portname.c_str(),O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(_fd == 0)
    {
        fprintf(stderr,"Failed to open port : %s",s_portname.c_str());
        return false;
    }

    // Set terminal control attributes
    tcflag_t ibaudrate = B57600;
    struct termios _option;
	memset( &_option, 0, sizeof(_option) );
	_option.c_cflag     = ibaudrate | CS8 | CLOCAL | CREAD;
	_option.c_iflag     = 0;              // Input parity check
	_option.c_oflag     = 0;
	_option.c_lflag     = 0;
	_option.c_cc[VTIME] = CTIME;    // Set time-out
	_option.c_cc[VMIN]  = CMIN;     // Set minimal returned characters
	tcflush(_fd,TCIFLUSH);               // flush file
	tcsetattr(_fd, TCSANOW, &_option);   // set attributes now
	fcntl(_fd, F_SETFL, FNDELAY);        // set flag as Non-delayed(Non-blocking)
    return true;
}
// Open port function
// Initial attribute
//  8 bits - data size
//  1 bit - stop bit
//  Allow receive
//  Disable parity
//  ignore only the cd signal
bool SerialCom::set_port(std::string s_portname, std::string s_baudrate)
{
    // Open port
    _fd = open(s_portname.c_str(),O_RDWR | O_NOCTTY | O_NONBLOCK);
    if(_fd == 0)
    {
        fprintf(stderr,"Failed to open port : %s",s_portname.c_str());
        return false;
    }

    // Set terminal control attributes
    tcflag_t ibaudrate = get_baudrate_from_string(s_baudrate);
    struct termios _option;
	memset( &_option, 0, sizeof(_option) );
	_option.c_cflag     = ibaudrate | CS8 | CLOCAL | CREAD;
	_option.c_iflag     = 0;              // Input parity check
	_option.c_oflag     = 0;
	_option.c_lflag     = 0;
	_option.c_cc[VTIME] = CTIME;    // Set time-out
	_option.c_cc[VMIN]  = CMIN;     // Set minimal returned characters
	tcflush(_fd,TCIFLUSH);               // flush file
	tcsetattr(_fd, TCSANOW, &_option);   // set attributes now
	fcntl(_fd, F_SETFL, FNDELAY);        // set flag as Non-delayed(Non-blocking)
    return true;
}


// Help for a function, set_parity
// Input argments : databits, stopbits and parity
//  databits - 7 or 8 bits
//  stopbits - 1 or 2 bits
//  parity setting
//      'n' : Disable parity bit
//      'o' : Use odd parity bit
//      'e' : Use Even parity bits
//      's' : stop 
bool SerialCom::set_parity(int databits, int stopbits, int parity)
{
    // c_cflag : control flag
    // c_iflag : input flag
    // c_oflag : output flag
    struct termios _option;
    tcgetattr(_fd,&_option);
    _option.c_cflag |= CLOCAL;   // Local line - do not change "owner" of port
    _option.c_cflag |= CREAD;    // Enable receiver
    switch(databits)
    {
        case 7: _option.c_cflag != CS7; break;
        case 8: _option.c_cflag != CS8; break;
        default: fprintf(stderr,"Unsupported data size"); break;
    }

    switch(stopbits)
    {
        // Use 1 stop bit
        case 1: _option.c_cflag &= ~CSTOPB; break;
        // Use 2 stop bits
        case 2: _option.c_cflag |= CSTOPB; break;
        default: fprintf(stderr,"Unsupported stopbits size"); break;
    }

    switch(parity)
    {
        case 'n':   // Disable parity
        case 'N':   // Enable parity checking
            _option.c_cflag &= ~PARENB;
            _option.c_iflag &= ~(INPCK | BRKINT | ICRNL | ISTRIP | IXON);
            break;
        case 'o':   // Enable parity and Use ODD parity bits
        case 'O':   // Enable parity check
            _option.c_cflag |= (PARENB | PARODD);
            _option.c_iflag |= INPCK;
            break;
        case 'e':   // Enable parity and Use EVEN parity bits
        case 'E':   // Enable parity check
            _option.c_cflag |= (PARENB);
            _option.c_cflag &= ~PARODD;  // Not odd, it's even
            _option.c_iflag |= INPCK;
            break;
        case 's':
        case 'S':
            _option.c_cflag &= ~PARENB;
            _option.c_cflag &= ~CSTOPB;
            break;
        default :
            fprintf(stderr, "Unsupported parity\n");
    }
    _option.c_cc[VTIME] = CTIME;
    _option.c_cc[VMIN]  = CMIN;

    tcflush(_fd, TCIFLUSH);
    if(tcsetattr(_fd, TCSANOW, &_option) != 0)
    {
        perror("Setup serial 3");
        return false;
    }
    tcflush(_fd, TCIOFLUSH);
    return true;
}

tcflag_t SerialCom::get_baudrate_from_string(std::string s_baudrate)
{
    int ibaudrate_in = atoi(s_baudrate.c_str());
    tcflag_t ibaudrate_out = 0;
    switch(ibaudrate_in)
    {
        case 50:        ibaudrate_out = 0000001; break;
        case 75:        ibaudrate_out = 0000002; break;
        case 110:       ibaudrate_out = 0000003; break;
        case 134:       ibaudrate_out = 0000004; break;
        case 150:       ibaudrate_out = 0000005; break;
        case 200:       ibaudrate_out = 0000006; break;
        case 300:       ibaudrate_out = 0000007; break;
        case 600:       ibaudrate_out = 0000010; break;
        case 1200:      ibaudrate_out = 0000011; break;
        case 1800:      ibaudrate_out = 0000012; break;
        case 2400:      ibaudrate_out = 0000013; break;
        case 4800:      ibaudrate_out = 0000014; break;
        case 9600:      ibaudrate_out = 0000015; break;
        case 19200:     ibaudrate_out = 0000016; break;
        case 38400:     ibaudrate_out = 0000017; break;
        case 57600:     ibaudrate_out = 0010001; break;
        case 115200:    ibaudrate_out = 0010002; break;
        case 230400:    ibaudrate_out = 0010003; break;
        case 460800:    ibaudrate_out = 0010004; break;
        case 500000:    ibaudrate_out = 0010005; break;
        case 576000:    ibaudrate_out = 0010006; break;
        case 921600:    ibaudrate_out = 0010007; break;
        case 1000000:   ibaudrate_out = 0010010; break;
        case 1152000:   ibaudrate_out = 0010011; break;
        case 1500000:   ibaudrate_out = 0010012; break;
        case 2000000:   ibaudrate_out = 0010013; break;
        case 2500000:   ibaudrate_out = 0010014; break;
        case 3000000:   ibaudrate_out = 0010015; break;
        case 3500000:   ibaudrate_out = 0010016; break;
        case 4000000:   ibaudrate_out = 0010017; break;
        default :       ibaudrate_out = 0000000; break;
    }
    return ibaudrate_out;
}

int SerialCom::get_fd(void)
{
    return _fd;
}