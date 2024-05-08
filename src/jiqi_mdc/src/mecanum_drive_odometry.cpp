#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int16MultiArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <math.h>

#include "../include/mdc_odometry.hpp"
#include "../include/mdc_pose.hpp"
#include "jiqi_mdc/jiqi_data.h"

//
class MDCOdometryNode
{
    public:
        MDCOdometryNode() : mdc_odometry() {}

        void main()
        {
            // ROS node initialisation
            ros::NodeHandle nh;

            // Initialise publishers
            mdc_odom_pub = nh.advertise<nav_msgs::Odometry>("/odometry/encoder", 10);

            // Initialise subscribers
            pulses_sub = nh.subscribe<jiqi_mdc::jiqi_data>("/encoder_pulse_data", 10, &MDCOdometryNode::pulsesCallback, this);
            initial_pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 10, &MDCOdometryNode::initialPoseCallback, this);

            // Retrieve parameters
            nh.param<int>("/mecanum_drive_odometry/pulses_per_meter", pulses_per_meter, 536);
            nh.param<double>("/mecanum_drive_odometry/wheel_separation_width", wheel_separation_width, 0.58);
            nh.param<double>("/mecanum_drive_odometry/wheel_separation_length", wheel_separation_length, 0.65);
            nh.param<double>("/mecanum_drive_odometry/rate", rate, 20.0);
            nh.param<std::string>("/mecanum_drive_odometry/base_frame_id", base_frame_id, "base_link");
            nh.param<std::string>("/mecanum_drive_odometry/odom_frame_id", odom_frame_id, "odom");
            
            // Initialise MDCOdometry object with parameters
            mdc_odometry.setWheelSeparationWidth(wheel_separation_width);
            mdc_odometry.setWheelSeparationLength(wheel_separation_length);
            mdc_odometry.setPulsesPerMeter(pulses_per_meter);
            mdc_odometry.setTime(ros::Time::now().toSec());

            ros::Rate loop_rate(rate);

            while (ros::ok())
            {
                publish();
                ros::spinOnce();
                loop_rate.sleep();
            }
        }

    private:
        void publish()
        {
            // Set now equal to current time
            ros::Time now = ros::Time::now();

            // Update pose
            mdc_odometry.updatePose(now.toSec());
            MDCPose mdc_pose = mdc_odometry.getPose();

            // Convert orientation to quaternion
            tf2::Quaternion q;
            q.setRPY(0, 0, mdc_pose.z_ang_pos);

            // Set odometry message member variables
            nav_msgs::Odometry odom;
            odom.header.stamp = now;
            odom.header.frame_id = odom_frame_id;
            odom.child_frame_id = base_frame_id;
            odom.pose.pose.position.x = mdc_pose.x_lin_pos;
            odom.pose.pose.position.y = mdc_pose.y_lin_pos;
            odom.pose.pose.position.z = 0;
            odom.pose.pose.orientation.x = q.x();
            odom.pose.pose.orientation.y = q.y();
            odom.pose.pose.orientation.z = q.z();
            odom.pose.pose.orientation.w = q.w();
            odom.twist.twist.linear.x = mdc_pose.x_lin_vel;
            odom.twist.twist.linear.y = mdc_pose.y_lin_vel;
            odom.twist.twist.angular.z = mdc_pose.z_ang_vel;
            
            // Set covariance matrices
            for (int i = 0; i < 36; i++)
            {
                odom.pose.covariance[i] = ODOM_POSE_COVARIANCE[i];
                odom.twist.covariance[i] = ODOM_TWIST_COVARIANCE[i];
            }

            // Publish odometry message
            mdc_odom_pub.publish(odom);
        }

        void pulsesCallback(const jiqi_mdc::jiqi_data::ConstPtr& msg)
        {
            // Updates encoder count according to new encoder_pulse_data message
            mdc_odometry.updateEncoderCount(msg->fl, msg->fr, msg->rl, msg->rr);
        }

        // UNSURE OF WHERE THIS TOPIC COMES FROM?????
        void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
        {
            // Convert quaternion to RPY
            tf2::Quaternion q;
            q.setX(msg->pose.pose.orientation.x);
            q.setY(msg->pose.pose.orientation.y);
            q.setZ(msg->pose.pose.orientation.z);
            q.setW(msg->pose.pose.orientation.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

            // Set initial pose
            MDCPose mdc_pose;
            mdc_pose.x_lin_pos = msg->pose.pose.position.x;
            mdc_pose.y_lin_pos = msg->pose.pose.position.y;
            mdc_pose.z_ang_pos = yaw;
            mdc_odometry.setPose(mdc_pose);
        }
        
        // Member variables
        MDCOdometry mdc_odometry;
        double wheel_separation_width, wheel_separation_length;
        int pulses_per_meter;
        double rate;
        std::string base_frame_id, odom_frame_id;

        // Define covariance matrices
        const double ODOM_POSE_COVARIANCE[36] { 1, 0, 0, 0,     0,     0, \
                                                0, 1, 0, 0,     0,     0, \
                                                0, 0, 1, 0,     0,     0, \
                                                0, 0, 0, 99999, 0,     0, \
                                                0, 0, 0, 0,     99999, 0, \
                                                0, 0, 0, 0,     0,     1  };

        const double ODOM_TWIST_COVARIANCE[36] { 1, 0, 0, 0,     0,     0, \
                                                0, 1, 0, 0,     0,     0, \
                                                0, 0, 1, 0,     0,     0, \
                                                0, 0, 0, 99999, 0,     0, \
                                                0, 0, 0, 0,     99999, 0, \
                                                0, 0, 0, 0,     0,     1  };

        // ROS publishers and subscribers
        ros::Publisher mdc_odom_pub;
        ros::Subscriber pulses_sub;
        ros::Subscriber initial_pose_sub;
};

int main(int argc, char **argv)
{
    // ROS node initialisation
    ros::init(argc, argv, "mecanum_drive_odometry");
    MDCOdometryNode node;
    node.main();

    // Indicates successful execution of program
    return 0;
}