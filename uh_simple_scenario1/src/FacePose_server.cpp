#include "ros/ros.h"
#include "uh_study/FacePose.h"

bool add(uh_simple_scenario1::FacePose::Request  &req,
         uh_simple_scenario1::FacePose::Response &res )
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "FacePose_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("FacePose", add);
  ROS_INFO("Ready to FacePose.");
  ros::spin();

  return 0;
}

