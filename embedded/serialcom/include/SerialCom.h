#pragma once
#include <ros/ros.h>
#include <string.h>
// for terminal input/output
#include <unistd.h>		//Hard link and Symbolic link
#include <termios.h>	//UART Setting
#include <fcntl.h>

class SerialCom{
public:
  bool set_port(std::string portname);
  bool set_port(std::string portname,std::string baudrate);
  bool set_baudrate(int Baudrate);
  bool set_parity(int databits, int stopbits, int parity);
  int get_fd(void);   //

private:
  int           _fd;
  std::string   _SerialPort;    // Serial Port
  int           _Baudrate;      // Baudrate

  void initial_setting(void);
  tcflag_t get_baudrate_from_string(std::string baudrate);
};