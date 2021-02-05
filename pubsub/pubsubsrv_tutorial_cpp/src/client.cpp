#include "ros/ros.h"
#include "pubsubsrv_tutorial_cpp/order.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "client");
  if (argc != 2)
  {
    ROS_INFO("usage: Commander 1/0");
    return 1;
  }


  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<pubsubsrv_tutorial_cpp::order>("command");
  pubsubsrv_tutorial_cpp::order srv;
  srv.request.order = atoll(argv[1]);
  if (client.call(srv))
  {
    if(srv.request.order)
      ROS_INFO("Pub On");
    else
      ROS_INFO("Pub Off");
  }
  else
  {
    ROS_ERROR("Failed to call service Commander");
    return 1;
  }

  return 0;
}