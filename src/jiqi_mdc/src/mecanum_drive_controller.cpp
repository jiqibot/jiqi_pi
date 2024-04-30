#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

#include "../include/mdc_controller.hpp"
#include "jiqi_mdc/jiqi_data.h"

class MDCNode
{
    public:
        MDCNode() : mdc_controller(), linear_x_velocity(0.0), linear_y_velocity(0.0), angular_z_velocity(0.0)
        {
            motor_pps.fl = 0;
            motor_pps.fr = 0;
            motor_pps.rl = 0;
            motor_pps.rr = 0;
            stop_msg = false;
            slow_msg = false;
        }

        void main()
        {
            // ROS node initialisation
            ros::NodeHandle nh;

            // Initialise publishers
            motor_pps_pub = nh.advertise<jiqi_mdc::jiqi_data>("motor_pps_data", 1);
            twist_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &MDCNode::twistCallback, this);
            
            // Initialise subscribers
            stop_obj_sub = nh.subscribe<std_msgs::Bool>("front_object_close_stop", 1, &MDCNode::stopCallback, this);
            slow_obj_sub = nh.subscribe<std_msgs::Bool>("front_object_close_slow", 1, &MDCNode::slowCallback, this);

            // Retrieve parameters
            nh.param<int>("/mecanum_drive_controller/pulses_per_meter", pulses_per_meter, 536);
            nh.param<double>("/mecanum_drive_controller/wheel_separation_width", wheel_separation_width, 0.58);
            nh.param<double>("/mecanum_drive_controller/wheel_separation_length", wheel_separation_length, 0.65);
            nh.param<int>("/mecanum_drive_controller/max_motor_speed", max_motor_speed, 256);
            nh.param<double>("/mecanum_drive_controller/rate", rate, 10.0);

            // Initialise MDCController object with parameters
            mdc_controller.setWheelSeparationWidth(wheel_separation_width);
            mdc_controller.setWheelSeparationLength(wheel_separation_length);
            mdc_controller.setPulsesPerMeter(pulses_per_meter);
            mdc_controller.setMaxMotorSpeed(max_motor_speed);

            ros::Rate loop_rate(rate);

            while (ros::ok())
            {
                personCheck();
                ros::spinOnce();
                loop_rate.sleep();
            }
        }

        
    private:
        // Checks if a person is within 1m or 2m and stops or slows robot respectively
        void personCheck()
        {
            if (stop_msg)
            {
                motor_pps.fl = 0;
                motor_pps.fr = 0;
                motor_pps.rl = 0;
                motor_pps.rr = 0;
                motor_pps_pub.publish(motor_pps);
                ROS_INFO_STREAM("Stopped");
            }

            else if (slow_msg)
            {
                max_motor_speed = 128;
                publish();
                ROS_INFO_STREAM("Slowed");
            }

            else
            {
                max_motor_speed = 256;
                publish();
                ROS_INFO_STREAM("Full Speed");
            }

            
        }

        void publish()
        {
            // Calculate motor speeds
            auto speeds = mdc_controller.motorSpeed(linear_x_velocity, linear_y_velocity, angular_z_velocity);

            // Set motor_pps message member variables
            motor_pps.fl = speeds.FL;
            motor_pps.fr = speeds.FR;
            motor_pps.rl = speeds.RL;
            motor_pps.rr = speeds.RR;

            // Publish motor_pps message
            motor_pps_pub.publish(motor_pps);
        }

        void twistCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
        {
            // Set variables equal to respective cmd_vel data
            linear_x_velocity = cmd_vel->linear.x;
            linear_y_velocity = cmd_vel->linear.y;
            angular_z_velocity = cmd_vel->angular.z;
        }

        void stopCallback(const std_msgs::Bool::ConstPtr& msg)
        {
            // Set variable according to front_object_close_stop data
            stop_msg = msg->data;
        }

        void slowCallback(const std_msgs::Bool::ConstPtr& msg)
        {
            // Set variable according to front_object_close_slow data
            slow_msg = msg->data;
        }

        // Member variables
        MDCController mdc_controller;
        double linear_x_velocity, linear_y_velocity, angular_z_velocity;
        double wheel_separation_width, wheel_separation_length;
        int max_motor_speed, pulses_per_meter;
        double rate;
        bool stop_msg, slow_msg;
        jiqi_mdc::jiqi_data motor_pps;

        // ROS publishers and subscribers
        ros::Publisher motor_pps_pub;
        ros::Subscriber twist_sub;
        ros::Subscriber slow_obj_sub;
        ros::Subscriber stop_obj_sub;
};

int main(int argc, char **argv)
{
    // ROS node initialisation
    ros::init(argc, argv, "mecanum_drive_controller");
    MDCNode node;
    node.main();

    // Indicates successful execution of program
    return 0;
}
