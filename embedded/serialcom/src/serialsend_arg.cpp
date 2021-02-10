// Node Input Argumentation
// 

#include "ros/ros.h"
#include <string.h>

#include <fcntl.h>  /* File Control Definitions          */
#include <termios.h>/* POSIX Terminal Control Definitions*/
#include <unistd.h> /* UNIX Standard Definitions         */
#include <errno.h>  /* ERROR Number Definitions          */

std::string s_portpath0 = std::string("/dev/ttyUSB0"); 
std::string s_baudrate0 = std::string("57600");

int main(int argc, char **argv)
{
  std::string s_nodename = std::string("serialsender_arg");
  ros::init(argc, argv, s_nodename.c_str());
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);

  // Define the properties of a serial port
  int fd;
  std::string s_portpath = s_portpath0; 
  std::string s_baudrate = s_baudrate0;

  bool bset = false;
  // Assign from function argumentations
  switch(argc)
  {
    case 1:
      ROS_INFO("Connect to default path %s...",s_portpath.c_str());
    break;
    case 2:
      s_portpath.assign(argv[1]); 
      ROS_INFO("Connect to %s...",s_portpath.c_str());
      bset = true;
      return 0;
    break;
    case 3:
      s_portpath.assign(argv[1]);
      s_baudrate.assign(argv[2]);
      ROS_INFO("Connect to %s... with %s bps",s_portpath.c_str(),s_baudrate.c_str());
      bset = true;
    break;
    default:
      ROS_ERROR("WRONG argumentations");
      return 0;
    break;
  }

  // Assign from ros param
  //roslaunch settings will be applied to rosparam server,then below will catch them.
  // Otherwise, the default settings will be applied.
  if(!bset)
  {
    nh.param(s_nodename + "/port", s_portpath, s_portpath0);
    nh.param(s_nodename + "/baud", s_baudrate, s_baudrate0);
    ROS_INFO("Set parameters via roslaunch settings");
    ROS_INFO("  Connect to %s...",s_portpath.c_str());
    ROS_INFO("  with %s",s_baudrate.c_str());
  }

  // Open a serial port
  // and set properties
  ROS_INFO("Now Connect to %s",s_portpath.c_str());
  fd = open(s_portpath.c_str(),O_RDWR | O_NOCTTY | O_NONBLOCK);
  if(fd == 0)
  {
    ROS_ERROR("Failed to open %s",s_portpath.c_str());
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


  // Action engage
  char read_buffer[512];
  int read_bytes = 0;

  char write_buffer[512];
  int write_bytes = 0;

  ros::Time last_request = ros::Time::now();
  ros::Time last_send_request = ros::Time::now();

  ROS_INFO("Loop start");
  int cnt = 0;
  while(ros::ok())
  {
    // Health Check
    if(ros::Time::now() - last_request > ros::Duration(1.0))
    {
      ROS_INFO("Healty");
      last_request = ros::Time::now();
    }
    // Write serial port
    if(ros::Time::now() - last_send_request > ros::Duration(1.0))
    {
      char strtemp[200];
      sprintf(strtemp,"Time to send %dth",cnt++);
      ROS_INFO("Send : %s",strtemp);
      write(fd,strtemp, strlen(strtemp)+1);
      last_send_request = ros::Time::now();
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}