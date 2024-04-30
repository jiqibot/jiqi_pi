#ifndef MDC_POSE_HPP
#define MDC_POSE_HPP

class MDCPose
{
    public:
        // Constructor to initialise member variables
        MDCPose() : x_lin_pos(0), y_lin_pos(0), z_ang_pos(0), x_lin_vel(0), y_lin_vel(0), z_ang_vel(0) {}

        // Member variables
        double x_lin_pos, y_lin_pos, z_ang_pos;
        double x_lin_vel, y_lin_vel, z_ang_vel;
};

#endif