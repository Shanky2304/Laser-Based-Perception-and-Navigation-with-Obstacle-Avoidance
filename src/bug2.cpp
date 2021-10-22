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
#include "nav_msgs/Odometry.h"
// for printing
#include <iostream>
// for RAND_MAX
#include <cstdlib>

class Bug2 {
private:
	ros::NodeHandle n;

	// Listen for odom & laser scan messages
    ros::Subscriber scan_sub, pose_truth_sub, ransac_vis_sub;
    ros::Publisher drive_pub;

    double goal_x = 4.5;
    double goal_y = 9.0;

    // When 1 means GOAL_SEEK when 0 means WALL_FOLLOW
    bool mode = 1, obs_need_to_turn = 0;

    // Flag to check if there's an obstacle in sight
    bool no_obstacle = 1;

    double detected_slope;

    // Set to one when the detected line has been populated and the robot needs to orient itself.
    bool orient = 0;

    // Turn left when we need to
    bool turn_left = 0;

public:
	Bug2() {
		n = ros::NodeHandle("~");

		std::string scan_topic = "/base_scan",
		pose_truth_topic = "/base_pose_ground_truth",
		cmd_vel_topic = "/cmd_vel",
		ransac_vis_topic = "/ransac_vis";

        scan_sub = n.subscribe(scan_topic, 1, &Bug2::scan_callback, this);
        pose_truth_sub = n.subscribe(pose_truth_topic, 1, &Bug2::pose_truth_callback, this);
        ransac_vis_sub = n.subscribe(ransac_vis_topic, 1, &Bug2::ransac_vis_callback, this);

        drive_pub = n.advertise<geometry_msgs::Twist>(cmd_vel_topic, 1, false);

	}

	void scan_callback(const sensor_msgs::LaserScan & lc_msg) {

		std::vector<double> ranges(lc_msg.ranges.begin(), lc_msg.ranges.end());
		ROS_INFO_STREAM("Num of scans: "<<ranges.size());

		no_obstacle = 1;

		for (int i = 0; i< ranges.size(); i++) {
			if (i > 145 && i < 157 && ranges[i] < 1.0) {
				// ROS_INFO("Less than 1!!");
				no_obstacle = 0;
				mode = 0;
				obs_need_to_turn = 1;
				//break;
			} else if (ranges[i] < 1.0) {
				no_obstacle = 0;
				//break;
			} 
			//else if (!mode && r) {
			 //	turn_left = 1;
			//}
		}

		if(!mode && no_obstacle) {
			for (int i = 0; i < ranges.size(); i++) {
				if (i > 350 && ranges[i] < 3.0) {
					// Something is on the left so do nothing wall follow is good/
				} else {
					turn_left = 1;
				}
			}
		}

	}

	void ransac_vis_callback(const visualization_msgs::Marker& marker) {

		// if WALL FOLLOW then do something with detected line
		if (!mode && obs_need_to_turn && marker.type == visualization_msgs::Marker::LINE_LIST) {
			
			ROS_INFO("Detected a line with ransac!");
			std::vector<geometry_msgs::Point> points = marker.points;
			double detected_slope = atan ((points[0].y - points[1].y) / (points[0].x - points[1].x));
			orient = 1;
		}

	}

	void pose_truth_callback(const nav_msgs::Odometry & odom) {
		geometry_msgs::Quaternion quat =  odom.pose.pose.orientation;
		geometry_msgs::Point position = odom.pose.pose.position;

		// Get the robot's current orientation
		geometry_msgs::Vector3 rpy;
		QuaternionToRPY(quat, rpy);

		if (mode) {

			// Slope of line from the current position to the goal, our robot should also orient with this.
			double theta_of_slope = atan((goal_y - position.y) / (goal_x - position.x));

			ROS_INFO_STREAM("Current orientation in z: "<<rpy.z<<" & "<<"orientation of the line to the goal: "<<theta_of_slope);

			double rad_to_turn = theta_of_slope - rpy.z;
			geometry_msgs::Twist twist;

			if (abs(rad_to_turn) > 0.01) {

				twist.angular.z = rad_to_turn * 2;

				// publish rad_to_turn*2 angular vel in z
				ROS_INFO("Turning...");
				publish_cmd_vel(twist);
				// Let the robot turn we can safely ignore callbacks while it's turning.
				ros::Duration(1).sleep();
				twist.angular.z = 0.0;
				publish_cmd_vel(twist);
			}

			if (no_obstacle) {
				ROS_INFO("Driving...");
				twist.linear.x = 1.0;
			} else {
				ROS_INFO("Stopping...");
				twist.linear.x = 0.0;
			}

			publish_cmd_vel(twist);

		} else {

			geometry_msgs::Twist twist;

			// WALL FOLLOW
			if(orient) {

				ROS_INFO_STREAM("No obstacle: "<<no_obstacle);

				if (no_obstacle) {
					ROS_INFO("Driving...");
					twist.linear.x = 1.0;
				} else {
					ROS_INFO("Stopping...");
					twist.linear.x = 0.0;
				}

				publish_cmd_vel(twist);

				// We have stopped at an obstacle need to turn now.
				obs_need_to_turn = 1;

				double rad_to_turn = detected_slope - rpy.z - 1.1;

				ROS_INFO_STREAM("Current orientation in z: "<<rpy.z<<" & "<<"orientation of the line to the detected_slope: "<<detected_slope);

				ROS_INFO_STREAM("rad_to_turn: "<<rad_to_turn);
				if (abs(rad_to_turn) > 0.00001) {

					twist.angular.z = rad_to_turn * 2;

					// publish rad_to_turn*2 angular vel in z
					ROS_INFO("Turning...");
					publish_cmd_vel(twist);
					// Let the robot turn.
					ros::Duration(0.5).sleep();
					twist.angular.z = 0.0;
					publish_cmd_vel(twist);
				}
				orient = 0;
				obs_need_to_turn = 0;

			} 
			 else if (turn_left) {
				// None of the ranges are less than one and we're in wall follow
				// Turn left by 90
				geometry_msgs::Twist twist;
				twist.angular.z = 1.57;
				ROS_INFO("Turning...");
				publish_cmd_vel(twist);
				ros::Duration(1).sleep();
				twist.angular.z = 0.0;
				publish_cmd_vel(twist);
				turn_left = 0;

			} 
			else {

				twist.linear.x = 1.0;
				publish_cmd_vel(twist);
			}


		}



	}

	void publish_cmd_vel(geometry_msgs::Twist twist) {

		ROS_INFO_STREAM("Twist.linear.x = "<<twist.linear.x);
		ROS_INFO_STREAM("Twist.linear.x = "<<twist.angular.z);
		drive_pub.publish(twist);

	}

	// Function for conversion of quaternion to roll pitch and yaw. 
	void QuaternionToRPY(const geometry_msgs::Quaternion msg, geometry_msgs::Vector3& rpy)
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