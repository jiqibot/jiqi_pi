#ifndef MDC_ODOMETRY_HPP
#define MDC_ODOMETRY_HPP

#include <cmath>
#include "mdc_encoder.hpp"
#include "mdc_pose.hpp"

class MDCOdometry
{
    public:
        // Constructor initialising member variables
        MDCOdometry() : delta_time(0), last_time(0), pulses_per_meter(0), wheel_separation_width(0), wheel_separation_length(0), FL_travel(0), FR_travel(0), RL_travel(0), RR_travel(0), delta_x_travel(0), delta_y_travel(0), delta_z_rotate(0) {}

        // Initialising four instances of the MDCEncoder class for each motor/wheel
        MDCEncoder FL_encoder, FR_encoder, RL_encoder, RR_encoder;
        
        // Initialising an instance of the MDCPose class
        MDCPose mdc_pose;

        void setWheelSeparationWidth(double separation)
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
            // Updates encoder data and converts from pulses to distance traveled (per wheel)
            FL_travel = FL_encoder.getDelta() / pulses_per_meter;
            FR_travel = FR_encoder.getDelta() / pulses_per_meter;
            RL_travel = RL_encoder.getDelta() / pulses_per_meter;
            RR_travel = RR_encoder.getDelta() / pulses_per_meter;
            
            // Calculates time expended between readings
            delta_time = new_time - last_time;

            // Calculates distance traveled / rotated
            delta_x_travel = (FL_travel + FR_travel + RL_travel + RR_travel);
            delta_y_travel = (-FL_travel + FR_travel + RL_travel - RR_travel);
            delta_z_rotate = (-FL_travel + FR_travel - RL_travel + RR_travel) / (2 * (wheel_separation_width + wheel_separation_length));

            // Calculates position data
            mdc_pose.x_lin_pos += delta_x_travel * cos(mdc_pose.z_ang_pos) - delta_y_travel * sin(mdc_pose.z_ang_pos);
            mdc_pose.y_lin_pos += delta_y_travel * cos(mdc_pose.z_ang_pos) + delta_x_travel * sin(mdc_pose.z_ang_pos);
            // Ensures the resultant angle remains within 0 to 2 pi (will return the remainder if not, wrapping back round)
            mdc_pose.z_ang_pos = fmod((mdc_pose.z_ang_pos + delta_z_rotate), (2 * M_PI));

            // Calculates velocity data (as long as delta_time > 0, to prevent non-defined values)
            mdc_pose.x_lin_vel = (delta_time > 0) ? delta_x_travel / delta_time : 0;
            mdc_pose.y_lin_vel = (delta_time > 0) ? delta_y_travel / delta_time : 0;
            mdc_pose.z_ang_vel = (delta_time > 0) ? delta_z_rotate / delta_time : 0;

            // Set last_time for future usage
            last_time = new_time;

        }

        // MDCPose specifies the return type of the method (const dictates that pose is read-only)
        MDCPose getPose() const
        {
            return mdc_pose;
        }

        // MDCPose& specifies that new_pose refers to a MDCPose object
        // const dictates that new_pose is read-only
        void setPose(const MDCPose& new_pose)
        {
            mdc_pose = new_pose;
        }

    private:
        // Member variables
        double delta_time, last_time;
        int pulses_per_meter;
        double wheel_separation_width, wheel_separation_length;
        double FL_travel, FR_travel, RL_travel, RR_travel; // Individual wheel travel
        double delta_x_travel, delta_y_travel, delta_z_rotate; // Overall robot travel
};

#endif