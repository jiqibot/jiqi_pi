#ifndef MDC_POSE_HPP
#define MDC_POSE_HPP

// Included to allow use of std::string and std::stringstream functions
#include <string>
#include <sstream>

class MDCPose
{
    public:
        // Define member variables for robot position
        double x_lin_pos, y_lin_pos, z_ang_pos;
        // Define member variables for robot velocity
        double x_lin_vel, y_lin_vel, z_ang_vel;
        // // Constructor to initialise member variables
        MDCPose() : x_lin_pos(0), y_lin_pos(0), z_ang_pos(0), x_lin_vel(0), y_lin_vel(0), z_ang_vel(0) {}

        // Method to convert pose data values into a singular string object 
        // Usage of 'const method' indicates that this function doesn't modify any Pose object's state
        std::string toString() const
        {
            // Declares variable pose_stream of type std::stringstream
            // Allows appending of various components using the stream insertion operator (<<)
            std::stringstream pose_stream;
            // Inserting robot position values into pose_stream
            pose_stream << "{'x_lin_pos': " << x_lin_pos << ", 'y_lin_pos': " << y_lin_pos << ", 'z_ang_pos': " << z_ang_pos;
            // Inserting robot velocity values into pose_stream
            pose_stream << "; 'x_lin_vel': " << x_lin_vel << ", 'y_lin_vel': " << y_lin_vel << ", 'z_ang_vel': " << z_ang_vel;
            // Return resultant string containing pose data
            return pose_stream.str();
        }
};

#endif