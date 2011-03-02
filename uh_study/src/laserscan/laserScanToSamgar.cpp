#include "ros/ros.h"
//#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
//#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"

class laserScanToSamgar{

public:

  ros::NodeHandle n_;
  //laser_geometry::LaserProjection projector_;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  //ros::Publisher scan_pub_;

  laserScanToSamgar(ros::NodeHandle n) : 
    n_(n),
    laser_sub_(n_, "base_scan", 10)
  {
    laser_sub_.registerCallback(boost::bind(&laserScanToSamgar::scanCallback, this, _1));
    //scan_pub_ = n_.advertise<sensor_msgs::PointCloud>("/my_cloud",1);

  }

  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {

	double angle = scan_in->angle_min;
	int i = 0;
	while( angle < scan_in->angle_max )
	{
		i++;
		angle = angle + scan_in->angle_increment;
	}
	printf("\n %.1lf %.1lf %.2lf %d", scan_in->angle_min, scan_in->angle_max, scan_in->angle_increment, i);

    //scan_pub_.publish(cloud);

  }
};

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "laserScanToSamgar");
  ros::NodeHandle n;
  laserScanToSamgar lstopc(n);
  
  ros::spin();
  
  return 0;
}

