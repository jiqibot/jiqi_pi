#ifndef MDC_ODOMETRY_HPP
#define MDC_ODOMETRY_HPP

#include <cmath>
#include "mdc_encoder.hpp"
#include "mdc_pose.hpp"

class MDCOdometry
{
    public:
        // Constructor ...
        MDCOdometry() : delta_time(0), last_time(0), pulses_per_meter(0), wheel_separation_width(0), wheel_separation_length(0), FL_travel(0), FR_travel(0), RL_travel(0), RR_travel(0), delta_x_travel(0), delta_y_travel(0), delta_z_rotate(0) {} // IMU last_yaw_deg(0), new_yaw_deg(0) {}

        // Initialising four instances of the MDCEncoder class for each motor/wheel
        MDCEncoder FL_encoder, FR_encoder, RL_encoder, RR_encoder;
        // Initialising an instance of the MDCPose class
        MDCPose pose;

        // IMU
        // void setYawAngleDeg(double yaw_ang)
        // {
        //     new_yaw_deg = yaw_ang;
        // }

        void setWheelSeparatioWidth(double separation)
        {
            wheel_separation_width = separation;
        }

        void setWheelSeparationLength(double separation)
        {
            wheel_separation_length = separation;
        }

        void setPulsesPerMeter(int tpm)
        {
            pulses_per_meter = tpm;
        }

        void setEncoderRange(int low, int high)
        {
            FL_encoder.setRange(low, high);
            FR_encoder.setRange(low, high);
            RL_encoder.setRange(low, high);
            RR_encoder.setRange(low, high);
        }

        void setTime(double new_time)
        {
            last_time = new_time;
        }

        void updateEncoderCount(int FL, int FR, int RL, int RR)
        {
            FL_encoder.updateCount(FL);
            FR_encoder.updateCount(FR);
            RL_encoder.updateCount(RL);
            RR_encoder.updateCount(RR);
        }

        void updatePose(int new_time)
        {
            FL_travel = FL_encoder.getDelta() / pulses_per_meter;
            FR_travel = FR_encoder.getDelta() / pulses_per_meter;
            RL_travel = RL_encoder.getDelta() / pulses_per_meter;
            RR_travel = RR_encoder.getDelta() / pulses_per_meter;
            
            delta_time = new_time - last_time;

            delta_x_travel = (FL_travel + FR_travel + RL_travel + RR_travel);
            delta_y_travel = (-FL_travel + FR_travel + RL_travel - RR_travel);
            delta_z_rotate = (-FL_travel + FR_travel - RL_travel + RR_travel) / (2 * (wheel_separation_width + wheel_separation_length));
            // IMU - convert to radians
            // delta_z_rotate = (new_yaw_deg - last_yaw_deg) * (M_PI / 180);

            pose.x_lin_pos += delta_x_travel * cos(pose.z_ang_pos) - delta_y_travel * sin(pose.z_ang_pos);
            pose.y_lin_pos += delta_y_travel * cos(pose.z_ang_pos) + delta_x_travel * sin(pose.z_ang_pos);
            pose.z_ang_pos = fmod((pose.z_ang_pos + delta_z_rotate), (2 * M_PI));

            pose.x_lin_vel = (delta_time > 0) ? delta_x_travel / delta_time : 0;
            pose.y_lin_vel = (delta_time > 0) ? delta_y_travel / delta_time : 0;
            pose.z_ang_vel = (delta_time > 0) ? delta_z_rotate / delta_time : 0;

            last_time = new_time;
            // IMU
            // last_yaw_deg = new_yaw_deg;
        }

        // MDCPose specifies the return type of the method
        // const dictates that pose is read-only
        MDCPose getPose() const
        {
            return pose;
        }

        // MDCPose& specifies that new_pose refers to a MDCPose object
        // const dictates that new_pose is read-only
        void setPose(const MDCPose& new_pose)
        {
            pose = new_pose;
        }

    private:
        // 
        double delta_time, last_time;
        // IMU
        // double last_yaw_deg, new_yaw_deg;
        //
        int pulses_per_meter;
        double wheel_separation_width, wheel_separation_length;
        //
        double FL_travel, FR_travel, RL_travel, RR_travel;
        double delta_x_travel, delta_y_travel, delta_z_rotate;
};

#endif