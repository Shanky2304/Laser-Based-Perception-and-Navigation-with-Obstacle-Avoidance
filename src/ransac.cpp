#include <ros/ros.h>

#include "ackermann_msgs/AckermannDriveStamped.h"

#include "ackermann_msgs/AckermannDrive.h"

#include "nav_msgs/Odometry.h"

#include "sensor_msgs/LaserScan.h"

// for printing
#include <iostream>

// for RAND_MAX
#include <cstdlib>

class Ransac {
private:
    ros::NodeHandle n;

    double max_speed, max_steering_angle;

    double safe_distance_threshold = 1;

    // Listen for odom & laser scan messages
    ros::Subscriber scan_sub;

    // Publish drive data
    ros::Publisher drive_pub;

    // previous desired steering angle
    double prev_angle=0.0;

    bool first_run = true;

    int64_t turn = 0;

public:
    Ransac() {
        n = ros::NodeHandle("~");
    }
};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "ransac");
    Ransac ransac;
    ros::spin();
    return 0;
}