#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include "LegsDetector/LegsDetector.h"
#include "LegsDetector/LegsDetectorUtility.h"

class LegDetection{

public:

  ros::NodeHandle n_;
  laser_geometry::LaserProjection projector_;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  ros::Publisher scan_pub_;

  LegsDetector *ld;

  LegDetection(ros::NodeHandle n) : 
    n_(n),
    laser_sub_(n_, "base_scan", 10)
  {
    laser_sub_.registerCallback(boost::bind(&LegDetection::scanCallback, this, _1));
    //scan_pub_ = n_.advertise<sensor_msgs::PointCloud>("/my_cloud",1);

    ld = new LegsDetector(312, 2, 2);
    ld->setDebug(true, 20);

  }

  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {

	laser_t ta;
	vector< laser_t > laserVector;

	laserVector.clear();
	double angle = scan_in->angle_min;
	int i = 0;
	//for (int i = 0; i < 512; i++ )
	while( angle < scan_in->angle_max )
	{
		ta.angle = angle;
		//ta.angle = (scan_in->angle_increment * i) + scan_in->angle_min;
		//if ( (ta.angle < scan_in->angle_min) || (ta.angle > scan_in->angle_max) ) continue;
		ta.range = scan_in->ranges[i];
		if ( (ta.range < scan_in->range_min) || (ta.range > scan_in->range_max) ) ta.range = scan_in->range_max;
		ta.x = 0; ta.y = 0;	ta.intensity = 4;
		laserVector.push_back(ta);
		i++;
		angle = angle + scan_in->angle_increment;
	}
	printf("\n %.1lf %.1lf %.2lf %d", RTOD(scan_in->angle_min), RTOD(scan_in->angle_max), RTOD(scan_in->angle_increment), i);
	ld->update(laserVector);
	int legHowMany = ld->getHowMany();

    //scan_pub_.publish(cloud);

  }
};

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "lserscan");
  ros::NodeHandle n;
  LegDetection lstopc(n);
  
  ros::spin();
  
  return 0;
}

