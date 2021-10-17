#include <ros/ros.h>
#include <math.h>
#include <cmath>
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
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
    ros::Publisher drive_pub, marker_pub;

    // previous desired steering angle
    double prev_angle=0.0;

    bool first_run = true;

    double inlier_threshold = 0.3;

    struct Point {
        int x;
        int y;
    };

public:
    Ransac() {
        n = ros::NodeHandle("~");
        std::string scan_topic="/base_scan";

        scan_sub = n.subscribe(scan_topic, 1, &Ransac::scan_callback, this);

        marker_pub = n.advertise<visualization_msgs::Marker>("/ransac_vis", 10);
    }

    void scan_callback(const sensor_msgs::LaserScan & lc_msg) {
        double angle_increment = lc_msg.angle_increment;
        double angle_min = lc_msg.angle_min;
        std::vector<double> ranges(lc_msg.ranges.begin(), lc_msg.ranges.end());
        Point p;
        std::vector<Point> points;
        double theta = 0.0;
        visualization_msgs::Marker points_marker, line_strip_marker, line_list_marker;
        geometry_msgs::Point pub_points;
        points_marker.type = visualization_msgs::Marker::POINTS;
        line_strip_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_list_marker.type = visualization_msgs::Marker::LINE_LIST;
        points_marker.header.frame_id = "/base_link";

        // Find the cartesian coordinates of points from the scan data.
        for (auto i : ranges) {
            theta = (angle_increment * i) + angle_min;

            p.x = ranges[i] * cos (theta);
            p.y = ranges[i] * sin (theta);

            points.push_back (p);
        }

        std::vector<geometry_msgs::Point> viz_points = processToSend(points);
        points_marker.points = viz_points;
        marker_pub.publish(points_marker);

        // Run ransac on the points we got above to detect lines.
        // std::vector<Point> line = ransac_cal(points);

        /* Publish the selected lines to rviz, if we found more than one line we should check if
         * they intersect, if they do we should publish the non-intersecting end points and the intersection point
         * to form a corner.
        */




    }

    std::vector<geometry_msgs::Point> processToSend (std::vector<Point> pointStructs) {
        std::vector <geometry_msgs::Point> points;
        for (auto i: pointStructs) {
            geometry_msgs::Point p;
            p.x = i.x;
            p.y = i.y;
            p.z = 0;
            points.push_back(p);
        }
        return points;
    }
//
//    std::vector <std::pair<Point, Point>> ransac_cal(std::vector<Point> points) {
//
//        int points_size = points.size();
//        int iterations = (int) (points_size * 0.1);
//        std::pair <Point, Point> best_candidate_pair;
//        std::vector <std::pair<Point, Point>> best_candidates;
//        double best_candidate_inliers_count = 0;
//
//        // Unless we have discarded 95% of the points keep going
//        while (points.size() > (int) (0.05 * points_size)) {
//            // In one iteration of this loop have detected one line
//            while (iterations--) {
//                // Random number between zero and one
//                double random1 = ((double) rand() / RAND_MAX);
//                random1 *= points.size();
//                double random2 = ((double) rand() / RAND_MAX);
//                random2 *= points.size();
//
//                std::vector <Point> inliers, outliers;
//
//                Point point_a = points[(int) random1];
//                // points.erase(points.begin() + (int) random1);
//                Point point_b = points[(int) random2];
//                // points.erase(points.begin() + (int) random1);
//
//                // First - A point, Second - Distance of "First" point from the line formed by point_a & point_b
//                std::vector<std::pair<Point ,double>> distances;
//
//                for (int i = 0; i < points.size(); i++) {
//                    if (i == (int) random1 || i == (int) random2)
//                        continue;
//
//                    // Calculate distance of points from the line formed by point_a & point_b
//                    distances.push_back(calculate_dist(point_a, point_b, points[i]));
//                }
//
//                /*
//                 * Now from the distance vector populate inliers and outliers but comparing with the threshold,
//                 * if the number of outliers is max amongst the ones detected so far this is the best candidate.
//                 * */
//                int inliers_count = 0;
//                for (auto i: distances) {
//                    if (i.second < inlier_threshold) {
//                        inliers_count++;
//                        // inliers.push_back(i);
//                    } else {
//                        outliers.push_back(i.first);
//                    }
//                }
//
//                if (inliers_count > best_candidate_inliers_count) {
//                    // Discard the inliers
//                    points = intersection(points, outliers);
//                    best_candidate_pair.first = point_a;
//                    best_candidate_pair.second = point_b;
//                    best_candidate_inliers_count = inliers.size();
//                }
//            }
//        }
//    }
//
//    // Calculate distance of point_c from the line being formed by point_a & point_b
//    // Adapted from the formula & https://stackoverflow.com/a/12132746
//    std::pair<Point ,double> calculate_dist(Point point_a, Point point_b, Point point_c) {
//        double a, b, c;
//        getLine (point_a, point_b, a, b, c);
//        return std::pair<Point ,double>(point_c,abs (a * point_c.x + b * point_c.y + c) / sqrt (a * a + b * b));
//    }
//
//    void getLine(Point point_a, Point point_b, double &a, double &b, double &c)
//    {
//        // (x- p1X) / (p2X - p1X) = (y - p1Y) / (p2Y - p1Y)
//        a = point_a.y - point_b.y; // Note: this was incorrectly "y2 - y1" in the original answer
//        b = point_b.x - point_a.x;
//        c = point_a.x * point_b.y - point_b.x * point_a.y;
//    }
//
//    bool isSamePoint(Point a, Point b) {
//        if (a.x == b.x && a.y == b.y)
//            return true;
//        return false;
//    }
//
//    std::vector <Point> intersection(std::vector <Point> &v1,
//                                           std::vector <Point> &v2) {
//        std::vector <Point> v3;
//
//        std::sort(v1.begin(), v1.end());
//        std::sort(v2.begin(), v2.end());
//
//        std::set_intersection(v1.begin(), v1.end(),
//                              v2.begin(), v2.end(),
//                              std::back_inserter(v3));
//        return v3;
//    }

};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "ransac");
    Ransac ransac;
    ros::spin();
    return 0;
}