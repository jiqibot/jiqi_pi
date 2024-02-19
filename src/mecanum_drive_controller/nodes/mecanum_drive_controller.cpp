#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16MultiArray.h>

#include "../include/mdc_controller.hpp"

class MDCNode
{
    public:
        MDCNode() : controller(), linear_x_velocity(0.0), linear_y_velocity(0.0), angular_velocity(0.0)
        {
            wheels_to_send.data.resize(4);
        }

        void main()
        {
            ros::NodeHandle nh;
            wheel_pub = nh.advertise<std_msgs::Int16MultiArray>("wheels_desired_rate", 1);
            twist_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &MDCNode::twistCallback, this);

            nh.getParam("ticks_per_meter", ticks_per_meter);
            nh.getParam("wheel_separation", wheel_separation_width);
            nh.getParam("wheel_separation_length", wheel_separation_length);
            nh.getParam("max_motor_speed", max_motor_speed);
            nh.param("rate", rate, 10.0);
            nh.param("timeout", timeout, 0.2);

            controller.setWheelSeparationWidth(wheel_separation_width);
            controller.setWheelSeparationLength(wheel_separation_length);
            controller.setTicksPerMeter(ticks_per_meter);
            controller.setMaxMotorSpeed(max_motor_speed);

            ros::Rate loop_rate(rate);
            last_twist_time = ros::Time::now();

            while (ros::ok())
            {
                publish();
                ros::spinOnce();
                loop_rate.sleep();
            }
        }


    private:
        // METHODS
        void publish()
        {
            if ((ros::Time::now() - last_twist_time).toSec() < timeout) {
                auto speeds = controller.motorSpeed(linear_x_velocity, linear_y_velocity, angular_velocity);

                wheels_to_send.data[0] = speeds.FL;
                wheels_to_send.data[1] = speeds.FR;
                wheels_to_send.data[2] = speeds.RL;
                wheels_to_send.data[3] = speeds.RR;

                wheel_pub.publish(wheels_to_send);
            } else {
                std_msgs::Int16MultiArray stop_msg;
                stop_msg.data = {0, 0, 0, 0};
                wheel_pub.publish(stop_msg);
            }
        }

        void twistCallback(const geometry_msgs::Twist &twist)
        {
            linear_x_velocity = twist.linear.x;
            linear_y_velocity = twist.linear.y;
            angular_velocity = twist.angular.z;
            last_twist_time = ros::Time::now();
        }

        // MEMBER VARIABLES
        MDCController controller;
        double linear_x_velocity, linear_y_velocity, angular_velocity;
        double ticks_per_meter, wheel_separation_width, wheel_separation_length;
        int max_motor_speed;
        double rate, timeout;
        std_msgs::Int16MultiArray wheels_to_send;

        // ALL ROS STUFF: SUBS, PUBS AND SERVICES
        ros::Time last_twist_time;
        ros::Publisher wheel_pub;
        ros::Subscriber twist_sub;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mecanum_drive_controller");
    MDCNode node;
    node.main();
    ROS_INFO("mecanum_drive_controller has started");

    ros::waitForShutdown();
    // INSERT SHUTDOWN FUNCTIONS HERE
}