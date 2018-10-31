#include <ros/ros.h>

#include <geometry_msgs/Point32.h>
#include <ohm_igvc_msgs/Waypoint.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>

#include <cmath>
#include <string>

#define DEG2RAD(x) ((3.14159265359 * x) / 180.00)

struct scan_transformer {
  	ros::NodeHandle nh;
  	ros::Publisher pub;
	ros::Publisher scanPub;
  	ros::Subscriber scanSub;
  	laser_geometry::LaserProjection projector_;
  	tf::TransformListener listener;
	std::string ref_frame_id;
	std::string base_frame_id;
	bool use_sim;

  	scan_transformer() {
      	ROS_INFO_STREAM("Converting /scan to XY point cloud");

      	// Create a ROS subscriber for the input laser scan
		ros::NodeHandle nh_private("~");

      	bool use_tf = false;
		nh_private.param("use_tf", use_tf, use_tf);
		nh_private.param(std::string("reference_frame"), ref_frame_id, std::string("world"));
		nh_private.param(std::string("base_frame"), base_frame_id, std::string("ohm_base_link"));
		nh_private.param(std::string("use_sim"), use_sim, false);
		
      	if(use_tf) scanSub = nh.subscribe(std::string("scan_in"), 1, &scan_transformer::scan_to_XY_tf, this);
      	else scanSub = nh.subscribe(std::string("scan_in"), 1, &scan_transformer::scan_to_XY, this);

      	// Create a ROS publisher for the output point cloud
      	pub = nh.advertise<sensor_msgs::PointCloud>(std::string("scan_to_xy_out"), 1);
		scanPub = nh.advertise<sensor_msgs::LaserScan>(std::string("corrected_scan"), 1);

      	ros::Duration(5.0).sleep();
  	};

  	void scan_to_XY (const sensor_msgs::LaserScan::ConstPtr& scan_in) {
		// Create a container for the data.
  	  	sensor_msgs::PointCloud cloud;
		cloud.header.stamp = ros::Time::now();
		cloud.header.frame_id = scan_in->header.frame_id;

		sensor_msgs::LaserScan scan = *scan_in;

		if(use_sim) {
			scan.angle_min = DEG2RAD(-90.0);
			scan.angle_max = DEG2RAD(90.0);
		}

  	  	//convert scan data into point cloud
    	projector_.projectLaser(scan, cloud);

    	//publish point cloud
    	pub.publish(cloud);  
		scanPub.publish(scan);

  	};

	void scan_to_XY_tf(const sensor_msgs::LaserScan::ConstPtr& scan_in) {
    	if(!listener.waitForTransform(ref_frame_id, scan_in->header.frame_id, scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment), ros::Duration(1.0))) {
    	  ROS_INFO("scan_to_XY_tf: waitForTransform failed");
    	  return;
   		}
		
		sensor_msgs::LaserScan scan = *scan_in;

		if(use_sim) {
			scan.angle_min = DEG2RAD(-90.0);
			scan.angle_max = DEG2RAD(90.0);
		}
  
   		sensor_msgs::PointCloud cloud;
    	projector_.transformLaserScanToPointCloud(ref_frame_id, scan, cloud, listener);

    	pub.publish(cloud);
		scanPub.publish(scan);
  	}; 
};

int main (int argc, char** argv) {
  // Initialize ROS
  ros::init (argc, argv, "scan_to_xy");

  scan_transformer transformer;

  // Spin
  ros::spin ();
}
