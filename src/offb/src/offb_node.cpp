/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ManualControl.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    // ROS_INFO("%s", (*msg).mode);
    current_state = *msg;
}

geometry_msgs::PoseStamped current_pose;
void pose_cb(const geometry_msgs::PoseStamped msg) {
    ROS_INFO("x: %lf\ty: %lf\tz: %lf", msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    current_pose = msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    
    ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pose_cb);

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
    cmd_vel.linear.z = 1;
    cmd_vel.angular.x = 0;
    cmd_vel.angular.y = 0;
    cmd_vel.angular.z = 0;

    while(ros::ok()){
            if( current_state.mode != mavros_msgs::State::MODE_PX4_OFFBOARD &&
				(ros::Time::now() - last_request > ros::Duration(5.0))){
				if( set_mode_client.call(offb_set_mode) &&
						offb_set_mode.response.mode_sent){
						ROS_INFO("Offboard enabled");
				}
				last_request = ros::Time::now();
            }

            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        
        double kP = 10;

        double setpoint = 5;

        double error = setpoint - current_pose.pose.position.z;

        double command = kP * error;

        cmd_vel.linear.z = command;

        manual_control_pub.publish(control);

        cmd_vel_pub.publish(cmd_vel);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
