#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include "LegsDetector.h"
#include "LegsDetectorUtility.h"

class LaserScanToPointCloud{

public:

  ros::NodeHandle n_;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
  ros::Publisher scan_pub_;

  LegsDetector *ld;

  LaserScanToPointCloud(ros::NodeHandle n) : 
    n_(n),
    laser_sub_(n_, "base_scan", 10),
    laser_notifier_(laser_sub_,listener_, "base_link", 10)
  {
    laser_notifier_.registerCallback(boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
    laser_notifier_.setTolerance(ros::Duration(0.01));
    scan_pub_ = n_.advertise<sensor_msgs::PointCloud>("/my_cloud",1);

    ld = new LegsDetector(512, 2, 3);
    ld->setDebug(true, 20);

  }

  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {

    sensor_msgs::PointCloud cloud;
    try
    {
        projector_.transformLaserScanToPointCloud(
          "base_link",*scan_in, cloud,listener_);
    }
    catch (tf::TransformException& e)
    {
        std::cout << e.what();
        return;
    }
    
    // Do something with cloud.
	laser_t ta;
	vector< laser_t > laserVector;

	laserVector.clear();
	printf("\n %lf %lf", RTOD(scan_in->angle_min), RTOD(scan_in->angle_max) );
	for (int i = 0; i < 512; i++ )
	{
		ta.angle = (scan_in->angle_increment * i) + scan_in->angle_min;
		if ( (ta.angle < scan_in->angle_min) || (ta.angle > scan_in->angle_max) ) continue;
		ta.range = scan_in->ranges[i];
		if ( (ta.range < scan_in->range_min) || (ta.range > scan_in->range_max) ) ta.range = scan_in->range_max;
		ta.x = 0; ta.y = 0;	ta.intensity = 4;
		laserVector.push_back(ta);
	}
	ld->update(laserVector);
	int legHowMany = ld->getHowMany();

    scan_pub_.publish(cloud);

  }
};

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "my_scan_to_cloud");
  ros::NodeHandle n;
  LaserScanToPointCloud lstopc(n);
  
  ros::spin();
  
  return 0;
}

