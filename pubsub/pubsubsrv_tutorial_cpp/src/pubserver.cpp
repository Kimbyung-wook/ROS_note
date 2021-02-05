#include "ros/ros.h"
#include "std_msgs/String.h"
#include "pubsubsrv_tutorial_cpp/order.h"

#include <sstream>

bool OnOff = false;

bool Callback_srv(pubsubsrv_tutorial_cpp::order::Request &req,
                  pubsubsrv_tutorial_cpp::order::Response &res)
{
  if(req.order)
    ROS_INFO("Pub On");
  else
    ROS_INFO("Pub Off");
  OnOff = req.order;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pubserver");
  
  ros::NodeHandle n;
  
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::ServiceServer service = n.advertiseService("command", Callback_srv);
  ROS_INFO("Ready for the order");

  ros::Rate loop_rate(2);
  
  int count = 0;
  while (ros::ok())
  {
    if(OnOff)
    {
      std_msgs::String msg;

      std::stringstream ss;
      ss << "hello world " << count;
      msg.data = ss.str();

      ROS_INFO("%s", msg.data.c_str());
      
      chatter_pub.publish(msg);
      ++count;
    }
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}