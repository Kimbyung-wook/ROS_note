#include <std_msgs/String.h>
#include <unistd.h>		//Hard link and Symbolic link
#include <termios.h>	//UART Setting
#include <fcntl.h>

class SerialCom{
public :
  SerialCom(std_msgs::String SerialPort);
  ~SerialCom();

  void set_baudrate(uint32_t Baudrate);

  std_msgs::String _SerialPort;  // Serial Port
  uint32_t _Baudrate;            // Baudrate
  


}