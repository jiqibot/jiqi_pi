#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

#include "../include/mdc_controller.hpp"
#include "jiqi_mdc/jiqi_data.h"

class MDCNode
{
    public:
        MDCNode() : controller(), linear_x_velocity(0.0), linear_y_velocity(0.0), angular_velocity(0.0)
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
            ros::NodeHandle nh;
            motor_pps_pub = nh.advertise<jiqi_mdc::jiqi_data>("motor_pps_data", 1);
            twist_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &MDCNode::twistCallback, this);
            
            stop_obj_sub = nh.subscribe<std_msgs::Bool>("front_object_close_stop", 1, &MDCNode::stopCallback, this);
            slow_obj_sub = nh.subscribe<std_msgs::Bool>("front_object_close_slow", 1, &MDCNode::slowCallback, this);

            nh.param("pulses_per_meter", pulses_per_meter, 536);
            nh.param("wheel_separation_width", wheel_separation_width, 0.58);
            nh.param("wheel_separation_length", wheel_separation_length, 0.65);
            nh.param("max_motor_speed", max_motor_speed, 256);
            nh.param("rate", rate, 10.0);
            nh.param("timeout", timeout, 0.2);

            controller.setWheelSeparationWidth(wheel_separation_width);
            controller.setWheelSeparationLength(wheel_separation_length);
            controller.setPulsesPerMeter(pulses_per_meter);
            controller.setMaxMotorSpeed(max_motor_speed);

            ros::Rate loop_rate(rate);

            while (ros::ok())
            {
                personCheck();
                ros::spinOnce();
                loop_rate.sleep();
            }
        }

        
    private:
        // METHODS

        void personCheck()
        {
            if (stop_msg)
            {
                motor_pps.fl = 0;
                motor_pps.fr = 0;
                motor_pps.rl = 0;
                motor_pps.rr = 0;
                motor_pps_pub.publish(motor_pps);
		std::cout << "Before ROS_INFO" << std::endl;
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
            auto speeds = controller.motorSpeed(linear_x_velocity, linear_y_velocity, angular_velocity);

            ROS_INFO_STREAM(linear_x_velocity);
            ROS_INFO_STREAM("speeds.FL = " << speeds.FL);

            motor_pps.fl = speeds.FL;
            motor_pps.fr = speeds.FR;
            motor_pps.rl = speeds.RL;
            motor_pps.rr = speeds.RR;

            motor_pps_pub.publish(motor_pps);
        }

        void twistCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
        {
            linear_x_velocity = cmd_vel->linear.x;
            linear_y_velocity = cmd_vel->linear.y;
            angular_velocity = cmd_vel->angular.z;
        }

        void stopCallback(const std_msgs::Bool::ConstPtr& msg)
        {
            stop_msg = msg->data;
        }

        void slowCallback(const std_msgs::Bool::ConstPtr& msg)
        {
            slow_msg = msg->data;
        }

        // MEMBER VARIABLES
        MDCController controller;
        double linear_x_velocity, linear_y_velocity, angular_velocity;
        double wheel_separation_width, wheel_separation_length;
        int max_motor_speed, pulses_per_meter;
        double rate, timeout;
        bool stop_msg, slow_msg;
        jiqi_mdc::jiqi_data motor_pps;

        // ALL ROS STUFF: SUBS, PUBS AND SERVICES
        ros::Publisher motor_pps_pub;
        ros::Subscriber twist_sub;
        ros::Subscriber slow_obj_sub;
        ros::Subscriber stop_obj_sub;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mecanum_drive_controller");
    MDCNode node;
    node.main();
    ROS_INFO("mecanum_drive_controller has started");

    return 0;
}
