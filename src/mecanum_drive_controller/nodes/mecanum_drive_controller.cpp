#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16MultiArray.h>

#include "../include/mdc_controller.hpp"

class MDCNode
{
    public:
        MDCNode() : controller(), linear_x_velocity(0.0), linear_y_velocity(0.0), angular_velocity(0.0)
        {
            motor_pps.data.resize(4);
        }

        void main()
        {
            ros::NodeHandle nh;
            motor_pps_pub = nh.advertise<std_msgs::Int16MultiArray>("motor_pps_data", 1);
            twist_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &MDCNode::twistCallback, this);

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
            last_twist_time = ros::Time::now();

            while (ros::ok())
            {
                publish();
                ros::spinOnce();
                loop_rate.sleep();
            }

            shutdown();
        }

        void shutdown()
        {
            std_msgs::Int16MultiArray stop_msg;
                stop_msg.data = {0, 0, 0, 0};
                motor_pps_pub.publish(stop_msg);
        }

    private:
        // METHODS
        void publish()
        {
            if ((ros::Time::now() - last_twist_time).toSec() < timeout) {
                auto speeds = controller.motorSpeed(linear_x_velocity, linear_y_velocity, angular_velocity);

                motor_pps.data[0] = speeds.FL;
                motor_pps.data[1] = speeds.FR;
                motor_pps.data[2] = speeds.RL;
                motor_pps.data[3] = speeds.RR;

                motor_pps_pub.publish(motor_pps);
            } else {
                shutdown();
            }
        }

        void twistCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel)
        {
            linear_x_velocity = cmd_vel->linear.x;
            linear_y_velocity = cmd_vel->linear.y;
            angular_velocity = cmd_vel->angular.z;
            last_twist_time = ros::Time::now();
        }

        // MEMBER VARIABLES
        MDCController controller;
        double linear_x_velocity, linear_y_velocity, angular_velocity;
        double wheel_separation_width, wheel_separation_length;
        int max_motor_speed, pulses_per_meter;
        double rate, timeout;
        std_msgs::Int16MultiArray motor_pps;

        // ALL ROS STUFF: SUBS, PUBS AND SERVICES
        ros::Time last_twist_time;
        ros::Publisher motor_pps_pub;
        ros::Subscriber twist_sub;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mecanum_drive_controller");
    MDCNode node;
    node.main();
    ROS_INFO("mecanum_drive_controller has started");

    return 0;
}