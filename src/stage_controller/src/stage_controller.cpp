//  so stupid
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <vector>
#include <algorithm>
#include <numeric>
using namespace std;
// ROS Publishers
ros::Publisher pub;
ros::Publisher pub_pos;

// ROS Messages
sensor_msgs::LaserScan laser_msg;
nav_msgs::Odometry odometry_msg;
geometry_msgs::Twist velocity;

// State variables
std::string state = "start";
std::string next_state = "start";
double roll, pitch, yaw;
double target_angle = 0.0;
bool TURNING = false;
int cont = 0;
double kp = 0.9;

// Target positions
std::vector<std::vector<double>> targets = {
    {8.0, 13.0},  // red
    {17.0, 4.4},  // blue
    {16.2, 12.0}, // green
    {2.0, 2.0}    // yellow
};

int target = 0;
double target_x = targets[target][0];
double target_y = targets[target][1];

// Minimum distance to detect the target
double min_distance = 0.5;

// Function declarations
void show_info();
void go_to_start_pos();
double get_rotation();
void move_forward();
void stop();
double get_turn_target_angle(double turn_angle);
void turn(double target_degree);
void turn_right();
void turn_left();
void start();
void aim();
void odometry_callback(const nav_msgs::Odometry::ConstPtr& data);
void laser_callback(const sensor_msgs::LaserScan::ConstPtr& data);
double get_robot2target_angle();
double get_robot2target_rel_angle();
int get_robot2target_sensor_index();
void fsm();

int main(int argc, char** argv) {
    ros::init(argc, argv, "stage_controller_node");
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber sub_odom = nh.subscribe("/base_pose_ground_truth", 10, odometry_callback);
    ros::Subscriber sub_laser = nh.subscribe("/base_scan", 10, laser_callback);

    // Publishers
    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    pub_pos = nh.advertise<nav_msgs::Odometry>("/base_pose_ground_truth", 10);

    ros::Rate rate(10); // 10 Hz

    while (ros::ok()) {
        double x = odometry_msg.pose.pose.position.x;
        double y = odometry_msg.pose.pose.position.y;

        // Distance to target
        double distance = std::sqrt(std::pow(x - target_x, 2) + std::pow(y - target_y, 2));

        // Wait for sensor to start
        if (!laser_msg.ranges.empty()) {
            // Checks if reached the target
            if (distance > min_distance) {
                fsm();
                show_info();
            } else {
                stop();
                ROS_INFO("Target reached!!");
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

void show_info() {
    double x = odometry_msg.pose.pose.position.x;
    double y = odometry_msg.pose.pose.position.y;
    double distance = std::sqrt(std::pow(x - target_x, 2) + std::pow(y - target_y, 2));

    system("clear");

    ROS_INFO("%s", state.c_str());
    ROS_INFO("T: (%.2f, %.2f)", target_x, target_y);
    ROS_INFO("P: (%.2f, %.2f)", x, y);
    ROS_INFO("D: %.2f", distance);
    ROS_INFO("robot angle: %f", get_rotation() * 180.0 / M_PI);
    ROS_INFO("robot2target_angle: %f", get_robot2target_angle() * 180.0 / M_PI);
    ROS_INFO("robot2target_rel_angle: %f", get_robot2target_rel_angle());
    ROS_INFO("sensor[%d] = %f ", get_robot2target_sensor_index(), laser_msg.ranges[get_robot2target_sensor_index()]);

    ros::Duration(0.2).sleep();
}

void go_to_start_pos() {
    nav_msgs::Odometry start_point;

    start_point.pose.pose.position.x = 8.0;
    start_point.pose.pose.position.y = 8.0;
    start_point.pose.pose.position.z = 0.0;

    tf::Quaternion quaternion = tf::createQuaternionFromRPY(0.0, 0.0, 0.0);
    start_point.pose.pose.orientation.x = quaternion.x();
    start_point.pose.pose.orientation.y = quaternion.y();
    start_point.pose.pose.orientation.z = quaternion.z();
    start_point.pose.pose.orientation.w = quaternion.w();

    pub_pos.publish(start_point);
}

double get_rotation() {
    tf::Quaternion quaternion(
        odometry_msg.pose.pose.orientation.x,
        odometry_msg.pose.pose.orientation.y,
        odometry_msg.pose.pose.orientation.z,
        odometry_msg.pose.pose.orientation.w
    );
    tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

    return yaw;
}

void move_forward() {
    velocity.linear.x = 0.5;
    velocity.angular.z = 0.0;
    pub.publish(velocity);
}

void stop() {
    velocity.linear.x = 0.0;
    velocity.angular.z = 0.0;
    pub.publish(velocity);
}

double get_turn_target_angle(double turn_angle) {
    double robot_angle = get_rotation() * 180.0 / M_PI;
    double target_angle = robot_angle + turn_angle;

    if (target_angle < -180.0) {
        return target_angle + 360.0;
    } else if (target_angle > 180.0) {
        return target_angle - 360.0;
    } else {
        return target_angle;
    }
}

void turn(double target_degree) {
    double robot_angle = get_rotation();
    double target_rad = target_degree * M_PI / 180.0;
    velocity.angular.z = kp * (target_rad - robot_angle);
    pub.publish(velocity);
}

void turn_right() {
    if (!TURNING) {
        target_angle = get_turn_target_angle(-30);
        TURNING = true;
    }

    turn(target_angle);

    if (std::abs(velocity.angular.z) < 0.0000001) {
        stop();
        TURNING = false;
        next_state = "move";
    }
}

void turn_left() {
    if (!TURNING) {
        target_angle = get_turn_target_angle(30);
        TURNING = true;
    }

    turn(target_angle);

    if (std::abs(velocity.angular.z) < 0.0000001) {
        stop();
        TURNING = false;
        next_state = "move";
    }
}

void start() {
    if (cont == 10) {
        next_state = "aim";
    }
    cont++;
}

void aim() {
    turn(get_robot2target_angle() * 180.0 / M_PI);
    if (std::abs(velocity.angular.z) < 0.0000001) {
        next_state = "move";
    }
}

void odometry_callback(const nav_msgs::Odometry::ConstPtr& data) {
    odometry_msg = *data;
}

void laser_callback(const sensor_msgs::LaserScan::ConstPtr& data) {
    laser_msg = *data;
}

double get_robot2target_angle() {
    double x = odometry_msg.pose.pose.position.x;
    double y = odometry_msg.pose.pose.position.y;
    double target_angle = std::atan2((target_y - y), (target_x - x));

    return target_angle;
}

double get_robot2target_rel_angle() {
    double robot_angle = get_rotation();
    double robot2target_angle = get_robot2target_angle();

    double rel_angle = (robot2target_angle - robot_angle) * 180.0 / M_PI;

    if (rel_angle < -230.0) {
        rel_angle = rel_angle + 360.0;
    } else if (rel_angle > 230.0) {
        rel_angle = rel_angle - 360.0;
    }
    return rel_angle;
}

int get_robot2target_sensor_index() {
    double rel_angle = get_robot2target_rel_angle();

    if (rel_angle <= -135.0) {
        return 0;
    } else if (rel_angle >= 135.0) {
        return 1080;
    } else {
        return 540 + static_cast<int>(4 * rel_angle);
    }
}

void fsm() {
    if (state == "start") {
        start();
    } else if (state == "aim") {
        aim();
    } else if (state == "move") {
        move_forward();

        double r_sensor_ave = std::accumulate(laser_msg.ranges.begin() + 180, laser_msg.ranges.begin() + 500, 0.0) / 320;
        double r_sensor_min = *std::min_element(laser_msg.ranges.begin() + 180, laser_msg.ranges.begin() + 500);

        double f_sensor_ave = std::accumulate(laser_msg.ranges.begin() + 500, laser_msg.ranges.begin() + 580, 0.0) / 80;
        double f_sensor_min = *std::min_element(laser_msg.ranges.begin() + 500, laser_msg.ranges.begin() + 580);

        double l_sensor_ave = std::accumulate(laser_msg.ranges.begin() + 580, laser_msg.ranges.begin() + 900, 0.0) / 320;
        double l_sensor_min = *std::min_element(laser_msg.ranges.begin() + 580, laser_msg.ranges.begin() + 900);

        double x = odometry_msg.pose.pose.position.x;
        double y = odometry_msg.pose.pose.position.y;

        if (*std::min_element(laser_msg.ranges.begin() + 420, laser_msg.ranges.begin() + 660) <= 0.5) {
            stop();
            ROS_INFO("AVOID WALL!!!");
            if (l_sensor_min > r_sensor_min) {
                next_state = "turn_left";
            } else {
                next_state = "turn_right";
            }
        } else {
            next_state = "move";
        }
    } else if (state == "turn_right") {
        turn_right();
    } else if (state == "turn_left") {
        turn_left();
    }

    state = next_state;
}