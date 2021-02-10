#include "ros/ros.h"
#include <string.h>

#include <fcntl.h>  /* File Control Definitions          */
#include <termios.h>/* POSIX Terminal Control Definitions*/
#include <unistd.h> /* UNIX Standard Definitions         */
#include <errno.h>  /* ERROR Number Definitions          */

int main(int argc, char **argv)
{
  ros::init(argc, argv, "serialreceive");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);

  // Term path
  int fd;
  std::string port_path = "/dev/ttyUSB0"; 
  fd = open(port_path.c_str(),O_RDWR | O_NOCTTY | O_NONBLOCK);

  if(fd == 0)
  {
    ROS_ERROR("Failed to open %s",port_path.c_str());
    return 0;
  }

	struct termios newtermios;
	memset( &newtermios, 0, sizeof(newtermios) );
	newtermios.c_cflag       = B57600 | CS8 | CLOCAL | CREAD;
	newtermios.c_iflag       = 0;              // NO_PARITY
	newtermios.c_oflag       = 0;
	newtermios.c_lflag       = 0;
	newtermios.c_cc[VTIME]   = CTIME;
	newtermios.c_cc[VMIN]    = CMIN;
	tcflush(fd,TCIFLUSH);		//RXD1를
	tcsetattr(fd, TCSANOW, &newtermios); //즉시 적용한다.
	fcntl(fd, F_SETFL, FNDELAY);
  
  // cfsetispeed(&newtermios,B57600);  //set reading speed
  // cfsetospeed(&newtermios,B57600);  //set writing speed
  // newtermios.c_cflag &= ~PARENB;  //no parity
  // newtermios.c_cflag &= ~CSTOPB;  //Stop bit 1
  // newtermios.c_cflag &= ~CSIZE;
  // newtermios.c_cflag |= CS8;
  // newtermios.c_cflag &= ~CRTSCTS; //HW flow control off
  // newtermios.c_cflag |= CREAD | CLOCAL; //turn on receiver
  // newtermios.c_iflag &= ~(IXON | IXOFF | IXANY);
  // newtermios.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  //non-canonical
  // tcsetattr(fd,TCSANOW,&newtermios); //set
  // tcgetattr(fd, &newtermios);
  // newtermios.c_cc[VMIN] = CTIME; //number of char to read
  // newtermios.c_cc[VTIME]=CMIN;  //timeout 1 sec
  // tcsetattr(fd,TCSANOW,&newtermios); //set

  char read_buffer[512];
  int read_bytes = 0;

  char write_buffer[512];
  int write_bytes = 0;

  ros::Time last_request = ros::Time::now();
  ros::Time last_send_request = ros::Time::now();

  ROS_INFO("Loop start");
  while(ros::ok())
  {
    // Health Check
    if(ros::Time::now() - last_request > ros::Duration(1.0))
    {
      ROS_INFO("Healty");
      last_request = ros::Time::now();
    }
    
    // Read serial port
    read_bytes = read(fd,&read_buffer,sizeof(read_buffer));
    if(read_bytes>0)
    {
      ROS_INFO("Receive %d : %s",read_bytes,read_buffer);
      memset(read_buffer, 0, sizeof(read_buffer));  //clear buffer
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}