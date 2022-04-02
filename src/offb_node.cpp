/**
 * @file offb_node.cpp
 * @brief Offboard control node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */


// Include std libraries
#include <array>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>

// Include ROS
#include <ros/ros.h>

// Include geometry messages
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

// Include MAVROS communication messages
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ManualControl.h>

// Define PD constants for controller
// Each direction gets a separate set of constants
double x_kP = 1;
double x_kD = 5;

double y_kP = 1;
double y_kD = 5;

double z_kP = 2;
double z_kD = 5;

// Create variables for previous error to use with derivative control
double x_prev_error = 0;
double y_prev_error = 0;
double z_prev_error = 0;

// Define setpoint as double array of length 3
// Setpoint is the target point
std::array<double, 3> setpoint;

// Update drone state every time it changes
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

// t variable is used to control rate of position message output
int t = 0;

// Update drone pose continuously and output position 
// to log once every 2 seconds
geometry_msgs::PoseStamped current_pose;
void pose_cb(const geometry_msgs::PoseStamped msg) {
    if(msg.header.stamp.sec - t > 0.2) {
        t = msg.header.stamp.sec;
        ROS_INFO("x: %.3lf y: %.3lf z: %.3lf", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    }
    current_pose = msg;
}

// Limit value of velocity command using a given maximum value
double limitCommand(double command, double max) {
    if(command > max) {
        return max;
    } else if(command < -max) {
        return -max;
    }

    return command;
}

// PID control, returns velocity commands in all 3 directions using
// current setpoint and current position
std::array<double, 3> pidControl() {
    // Get position as Point from current pose
    geometry_msgs::Point pos = current_pose.pose.position;

    // Calculate error in position
    double x_error = setpoint[0] - pos.x;
    double y_error = setpoint[1] - pos.y;
    double z_error = setpoint[2] - pos.z;

    // Calculate individual commands using PD constants, error, and derivative
    // Limit command to 3.0 m/s
    double x_command = limitCommand(x_kP * x_error + x_kD * (x_error - x_prev_error), 1.0);
    double y_command = limitCommand(y_kP * y_error + y_kD * (y_error - y_prev_error), 1.0);
    double z_command = limitCommand(z_kP * z_error + z_kD * (z_error - z_prev_error), 1.0);

    // Update previous error values for next loop
    x_prev_error = x_error;
    y_prev_error = y_error;
    z_prev_error = z_error;

    // Return command values in double array
    return {x_command, y_command, z_command};
}

// Calculate absolute distance to target point from current
// pose. Used to determine when to move to next waypoint
double distanceToTarget(std::array<double, 3> target) {
    geometry_msgs::Point pos = current_pose.pose.position;

    return sqrt(pow(target[0] - pos.x, 2) + pow(target[1] - pos.y, 2) + pow(target[2] - pos.z, 2));
}

// Create path variable to store a list of setpoints in
// order. idx variable points to current setpoint
std::vector<std::array<double, 3>> path;
int idx = 0;

// Main function
int main(int argc, char **argv)
{

    // Open points.txt in read-only to extract pre-determined
    // path of waypoints
    std::ifstream ifs;
    ifs.open("src/offb/points.txt", std::ifstream::in);

    // Instantiate variable for parsing loop
    std::array<double, 3> point;
    int i = 0;
    std::string str;
    char c = ifs.get();

    // Read each character in text file and parse
    while (ifs.good()) {
        // If c is a space or newline, the number is done,
        // so it will be added to the array
        if(c == ' ' || c == '\n') {
            point[i] = std::stod(str);
            i++;
            // If there are three numbers in the array, push
            // the point onto the vector and reset i
            if(i > 2) {
                path.push_back(point);
                i = 0;
            }
            // Clear string for next number
            str.clear();
        } else {
            // Add c to the string if it is a number
            str.push_back(c);
        }

        // Get next character in file
        c = ifs.get();
    }

    // Set first setpoint
    setpoint = path.at(idx);

    // Close file stream
    ifs.close();

    // Create node handle
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    // Subscribe to State topic and Pose topic to update drone state
    // and position/velocity
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mocap_pos", 10, pose_cb);

    // Create publishers to manual control and velocity command
    ros::Publisher manual_control_pub = nh.advertise<mavros_msgs::ManualControl>
            ("mavros/manual_control/send", 10);
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    
    // Create service clients for arming drone and setting mode to offboard
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

    // Create request for switching to offboard flight mode
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = mavros_msgs::State::MODE_PX4_OFFBOARD;

    // Create request to arm drone motors
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    // Store current time for use later with limiting requests to
    // once every 5 seconds
    ros::Time last_request = ros::Time::now();

    // Manual control struct. Drone does not use manual control,
    // but publishing to manual control topic is still necessary
    // for autonomous flight for unknown reasons
    mavros_msgs::ManualControl control;
    control.x = 0;
    control.y = 0;
    control.z = 0;
    control.r = 0;

    // Command velocity struct. Includes both linear and angular 
    // velocity. This is what the drone repsonds to. Currently, 
    // only linear velocity is used
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0;

    // Main logic loop
    while(ros::ok()){
        // If the drone is not in offboard flight mode, switch to offboard
        // Request once every 5 seconds
        if( current_state.mode != mavros_msgs::State::MODE_PX4_OFFBOARD && (ros::Time::now() - last_request > ros::Duration(5.0))){
			if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
				// Log once offboard enabled
                ROS_INFO("Offboard enabled");
			}
            // Update request time
			last_request = ros::Time::now();
        }

        // If drone is not armed, request arming motors
        // Request once every 5 seconds
        if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( arming_client.call(arm_cmd) &&
                arm_cmd.response.success){
                    // Log once armed
                    ROS_INFO("Vehicle armed");
                }
                // Update request time
                last_request = ros::Time::now();
            }

        // Get velocity commands
        std::array<double, 3> command = pidControl();

        // If altitude is less than 0.75 m, do not move horizontally
        // This prevents drone from hitting the ground when taking off
        if(current_pose.pose.position.z > 0.75) {
            // Set x and y velocity commands
            cmd_vel.linear.x = command[0];
            cmd_vel.linear.y = command[1];
        } else {
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
        }

        // Set z velocity command
        cmd_vel.linear.z = command[2];

        // Publish manual control and velocity command
        manual_control_pub.publish(control);
        cmd_vel_pub.publish(cmd_vel);

        // If drone is less than 2 inches away from target,
        // move to next waypoint in the path
        if(distanceToTarget(setpoint) < 0.05) {
            // Increment pointer
            idx++;

            // Log which waypoint was reached
            ROS_INFO("Waypoint %i reached...", idx);

            // If there are no more waypoints, exit control loop
            if(idx < path.size()) {
                // Set current setpoint to next waypoint
                setpoint = path.at(idx);
            } else {
                // Log that end of path was reached, then exit loop
                ROS_INFO("End of path reached!");
                break;
            }
        }

        // Spin up node then briefly sleep to conserve 
        // computational resources
        ros::spinOnce();
        rate.sleep();
    }

    // Return 0 if exiting normally
    return 0;
}
