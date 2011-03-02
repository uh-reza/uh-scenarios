#include "ros/ros.h"
#include "uh_study/FacePoseSrv.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "face_pose_srv_client");
  if (argc != 3)
  {
    ROS_INFO("usage: face_pose_srv_client X Y");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<uh_study::FacePoseSrv>("face_pose_srv");
  uh_study::FacePoseSrv srv;
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);
  if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service face_pose_srv");
    return 1;
  }

  return 0;
}

