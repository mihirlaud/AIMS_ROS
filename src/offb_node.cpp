/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <array>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ManualControl.h>

double x_kP = 2;
double x_kD = 5;

double y_kP = 1;
double y_kD = 5;

double z_kP = 1;
double z_kD = 5;

double x_prev_error = 0;
double y_prev_error = 0;
double z_prev_error = 0;

std::array<double, 3> setpoint;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    // ROS_INFO("%s", (*msg).mode);
    current_state = *msg;
}

int t = 0;

geometry_msgs::PoseStamped current_pose;
void pose_cb(const geometry_msgs::PoseStamped msg) {
    if(msg.header.stamp.sec - t > 1) {
        t = msg.header.stamp.sec;
        ROS_INFO("x: %.3lf y: %.3lf z: %.3lf", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    }
    current_pose = msg;
}

double limitCommand(double command, double max) {
    if(command > max) {
        return max;
    } else if(command < -max) {
        return -max;
    }

    return command;
}

std::array<double, 3> pidControl() {
    geometry_msgs::Point pos = current_pose.pose.position;

    double x_error = setpoint[0] - pos.x;
    double y_error = setpoint[1] - pos.y;
    double z_error = setpoint[2] - pos.z;

    double x_command = limitCommand(x_kP * x_error + x_kD * (x_error - x_prev_error), 3.0);
    double y_command = limitCommand(y_kP * y_error + y_kD * (y_error - y_prev_error), 3.0);
    double z_command = limitCommand(z_kP * z_error + z_kD * (z_error - z_prev_error), 3.0);

    x_prev_error = x_error;
    y_prev_error = y_error;
    z_prev_error = z_error;

    return {x_command, y_command, z_command};
}

double distanceToTarget(std::array<double, 3> target) {
    geometry_msgs::Point pos = current_pose.pose.position;

    return sqrt(pow(target[0] - pos.x, 2) + pow(target[1] - pos.y, 2) + pow(target[2] - pos.z, 2));
}

std::vector<std::array<double, 3>> path;
int idx = 0;

int main(int argc, char **argv)
{

    std::ifstream ifs;
    ifs.open("src/offb/points.txt", std::ifstream::in);

    std::array<double, 3> point;
    int i = 0;
    std::string str;
    char c = ifs.get();

    while (ifs.good()) {
        if(c == ' ' || c == '\n') {
            point[i] = std::stod(str);
            i++;
            if(i > 2) {
                path.push_back(point);
                i = 0;
            }
            str.clear();
        } else {
            str.push_back(c);
        }

        c = ifs.get();
    }

    setpoint = path.at(idx);

    ifs.close();

    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pose_cb);

    ros::Publisher manual_control_pub = nh.advertise<mavros_msgs::ManualControl>
            ("mavros/manual_control/send", 10);
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = mavros_msgs::State::MODE_PX4_OFFBOARD;

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    bool takeoff = false;

    mavros_msgs::ManualControl control;
    control.x = 0;
    control.y = 0;
    control.z = 0;
    control.r = 0;

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0;

    while(ros::ok()){
        if( current_state.mode != mavros_msgs::State::MODE_PX4_OFFBOARD && (ros::Time::now() - last_request > ros::Duration(5.0))){
			if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
				ROS_INFO("Offboard enabled");
			}
			last_request = ros::Time::now();
        }

        if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( arming_client.call(arm_cmd) &&
                arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }

        std::array<double, 3> command = pidControl();

        if(current_pose.pose.position.z > 0.75) {
            cmd_vel.linear.x = command[0];
            cmd_vel.linear.y = command[1];
        } else {
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
        }

        cmd_vel.linear.z = command[2];

        manual_control_pub.publish(control);

        cmd_vel_pub.publish(cmd_vel);

        if(distanceToTarget(setpoint) < 0.05) {
            idx++;
            ROS_INFO("Waypoint %i reached...", idx);
            if(idx < path.size()) {
                setpoint = path.at(idx);
            } else {
                ROS_INFO("End of path reached!");
                break;
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
