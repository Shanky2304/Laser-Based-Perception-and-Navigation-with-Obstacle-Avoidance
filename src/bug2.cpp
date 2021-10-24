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

    double start_x = -8.0;
    double start_y = -2.0;

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

    // Is there an obstacle on the left?
    bool obs_on_left = 0;

    // Are we on the line use to retain state from last base_pose_truth.
    bool on_the_line = 0;

    struct Point {
        double x;
        double y;
    };

    Point start_point, end_point;

    double theta_of_slope;

    bool reached_goal = 0;

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

    	start_point.x = start_x;
    	start_point.y = start_y;

    	end_point.x = goal_x;
    	end_point.y = goal_y;

    	// Slope of line from the current position to the goal, our robot should also orient with this.
		theta_of_slope = atan((goal_y - start_point.y) / (goal_x - start_point.x));

	}

	void scan_callback(const sensor_msgs::LaserScan & lc_msg) {

		std::vector<double> ranges(lc_msg.ranges.begin(), lc_msg.ranges.end());

		no_obstacle = 1;

		for (int i = 0; i< ranges.size(); i++) {
			if (i > 147 && i < 180 && ranges[i] < 1.6) {
				// ROS_INFO("Less than 1!!");
				no_obstacle = 0;
				mode = 0;
				obs_need_to_turn = 1;
				//break;
			} else if (ranges[i] < 1.0) {
				// ROS_INFO_STREAM("Less than 1 on the side : "<<i);
				no_obstacle = 0;
				// mode = 0;
				//break;
			} 
			//else if (!mode && r) {
			 //	turn_left = 1;
			//}
		}

		if(!mode && no_obstacle) {
			for (int i = 0; i < ranges.size(); i++) {
				if (i > 350 && ranges[i] < 1.5) {
					obs_on_left = 1;
				} else {
					turn_left = 1;
				}
			}
		}

	}

	void ransac_vis_callback(const visualization_msgs::Marker& marker) {

		// if WALL FOLLOW then do something with detected line
		if (!mode && obs_need_to_turn && marker.type == visualization_msgs::Marker::LINE_LIST) {
			
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

		Point curr_point;
		curr_point.x = position.x;
		curr_point.y = position.y;

		if (abs (curr_point.x - goal_x) < 1.2 && abs (curr_point.y - goal_y) < 1.2) {
			ROS_INFO("Reached the goal!");
			geometry_msgs::Twist twist;
			if(abs (curr_point.x - goal_x) < 0.07 && abs (curr_point.y - goal_y) < 0.07) {
				twist.linear.x = 0.0;
				twist.angular.z = 0.0;
				publish_cmd_vel(twist);
			} else {
				twist.linear.x = 1.0;
				twist.angular.z = 1.0;
				publish_cmd_vel(twist);
				ros::Duration(1).sleep();
				twist.linear.x = 0.0;
				twist.angular.z = 0.0;
				publish_cmd_vel(twist);
				reached_goal = 1;
			}	
			return;
		}

		if (mode) {

			ROS_INFO_STREAM("Current orientation in z: "<<rpy.z<<" & "<<"orientation of the line to the goal: "<<theta_of_slope);

			double rad_to_turn = theta_of_slope - rpy.z;
			if(rpy.z < 0) {
				rad_to_turn = rpy.z - theta_of_slope;
			}
			geometry_msgs::Twist twist;

			if (abs(rad_to_turn) > 0.01) {

				twist.angular.z = rad_to_turn * 2;

				// publish rad_to_turn*2 angular vel in z
				publish_cmd_vel(twist);
				// Let the robot turn we can safely ignore callbacks while it's turning.
				ros::Duration(1).sleep();
				twist.angular.z = 0.0;
				publish_cmd_vel(twist);
			}

			if (no_obstacle) {
				twist.linear.x = 1.0;
			} else {
				twist.linear.x = 0.0;
			}

			publish_cmd_vel(twist);

		} else {
			// WALL FOLLOW
			std::pair<Point, double> curr_dist = calculate_dist(start_point, end_point, curr_point);
			ROS_INFO("Are we on the line now?");
			if (curr_dist.second < 0.1 && obs_on_left && !on_the_line) {
					ROS_INFO("On the line!");
					geometry_msgs::Twist twist;
					twist.linear.x = 0;

					double rad_to_turn = theta_of_slope - rpy.z;
					twist.angular.z = rad_to_turn;

					publish_cmd_vel(twist);

					ros::Duration(1).sleep();
					twist.angular.z = 0.0;
					publish_cmd_vel(twist);
					mode = 1;
				ROS_INFO("Already on the line!");
				on_the_line = 1;
				return;
			} else if (curr_dist.second > 0.2) {
				ROS_INFO("Got off the line!");
				on_the_line = 0;
			}

			geometry_msgs::Twist twist;

			ROS_INFO("Got here!");

			if (orient) {

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
					publish_cmd_vel(twist);
					// Let the robot turn.
					ros::Duration(1.0).sleep();
					twist.angular.z = 0.0;
					publish_cmd_vel(twist);
				}
				orient = 0;
				obs_need_to_turn = 0;

			} else if (turn_left) {
				// None of the ranges are less than one and we're in wall follow
				// Turn left by 90
				geometry_msgs::Twist twist;
				twist.angular.z = 1.57;
				publish_cmd_vel(twist);
				ros::Duration(1).sleep();
				twist.angular.z = 0.0;
				publish_cmd_vel(twist);
				turn_left = 0;

			} else {
				ROS_INFO("Nothing ahead following wall.");
				twist.linear.x = 1.0;
				publish_cmd_vel(twist);
			}


		}



	}

	void publish_cmd_vel(geometry_msgs::Twist twist) {

		ROS_INFO_STREAM("Twist.linear.x = "<<twist.linear.x);
		ROS_INFO_STREAM("Twist.angular.x = "<<twist.angular.z);
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

	std::pair<Point, double> calculate_dist(Point point_a, Point point_b, Point point_c) {
        double a, b, c;
        getLine (point_a, point_b, a, b, c);
        return std::pair<Point ,double>(point_c,abs (a * point_c.x + b * point_c.y + c) / sqrt (a * a + b * b));
    }
    

    void getLine(Point point_a, Point point_b, double &a, double &b, double &c)
    {
        // (x- p1X) / (p2X - p1X) = (y - p1Y) / (p2Y - p1Y)
        a = point_a.y - point_b.y;  
        b = point_b.x - point_a.x;
        c = point_a.x * point_b.y - point_b.x * point_a.y;
    }

};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "bug2");
    Bug2 bug2;
    ros::spin();
    return 0;
}