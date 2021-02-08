#include "ros/ros.h"
#include "std_msgs/String.h"

#include <unistd.h>		//Hard link and Symbolic link
#include <termios.h>	//UART Setting
#include <fcntl.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "serialcom");
  
	struct termios newtio;
	memset( &newtio, 0, sizeof(newtio) );  // �ø��� ��Ʈ ��� ȯ�� ����
	newtio.c_cflag       = Baudrate_d | CS8 | CLOCAL | CREAD;
	newtio.c_iflag       = 0;              // NO_PARITY
	newtio.c_oflag       = 0;
	newtio.c_lflag       = 0;
	newtio.c_cc[VTIME]   = C_VTIME;
	newtio.c_cc[VMIN]    = C_VMIN;
	tcflush(tranceiver,TCIFLUSH);		//RXD1��
	tcsetattr(tranceiver, TCSANOW, &newtio); //��� �����Ѵ�.
	fcntl(tranceiver, F_SETFL, FNDELAY);

  switch (argc)
  {
  case 1:
    break;
  case 2: // serial port
    break;
  case 3: // serial port & baud rate
    break;
  default:
    break;
  }

  ros::NodeHandle n;



  ros::spin();

  return 0;
}