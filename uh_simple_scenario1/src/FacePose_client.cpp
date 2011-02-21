#include "ros/ros.h"
#include "uh_study/FacePose.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "FacePose_client");
  if (argc != 3)
  {
    ROS_INFO("usage: FacePose_client X Y");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<uh_simple_scenario1::FacePose>("FacePose");
  uh_simple_scenario1::FacePose srv;
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);
  if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service FacePose");
    return 1;
  }

  return 0;
}

