#include <ros/ros.h>
#include <math.h>
#include <cmath>
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
// for printing
#include <iostream>
// for RAND_MAX
#include <cstdlib>

class Bug2 {
private:
	ros::NodeHandle n;

	// Listen for odom & laser scan messages
    ros::Subscriber scan_sub;

public:
	Bug2() {
		n = ros::NodeHandle("~");

		std::string scan_topic = "/base_scan";

        scan_sub = n.subscribe(scan_topic, 1, &Bug2::scan_callback, this);

	}

	void scan_callback(const sensor_msgs::LaserScan & lc_msg) {
	}

	// Function for conversion of quaternion to roll pitch and yaw. 
	void MsgCallback(const geometry_msgs::Quaternion msg, geometry_msgs::Vector3& rpy)
	{
	    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
	    tf::Quaternion quat;
	    tf::quaternionMsgToTF(msg, quat);

	    // the tf::Quaternion has a method to acess roll pitch and yaw
	    double roll, pitch, yaw;
	    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

	    // the found angles are written in a geometry_msgs::Vector3
	    rpy.x = roll;
	    rpy.y = pitch;
	    rpy.z = yaw;
	}

};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "bug2");
    Bug2 bug2;
    ros::spin();
    return 0;
}