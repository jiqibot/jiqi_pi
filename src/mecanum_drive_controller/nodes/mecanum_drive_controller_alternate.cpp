#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16MultiArray.h>

#include "../include/mdc_controller.hpp"

class MDCNode
{
    public:
        MDCNode(ros::NodeHandle *nh)
        {
        }

        MDCController controller;

    private:
        // MEMBER VARIABLES
        MDCController controller_;
        double linear_x_velocity_;
        double linear_y_velocity_;
        double angular_velocity_;
        std_msgs::Int16MultiArray wheels_to_send_;

        // ALL SUBS, PUBS AND SERVICES
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mecanum_drive_controller");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(0);
    spinner.start();

    MDCNode node(&nh);
    ROS_INFO("mecanum_drive_controller has started");

    ros::waitForShutdown();
    // INSERT SHUTDOWN FUNCTIONS HERE
}