#ifndef MDC_CONTROLLER_HPP
#define MDC_CONTROLLER_HPP

// Included to allow use of std::max function
#include <algorithm>

// Define class to hold motor control commands
class MDCMotorCommand
{
    public:
        // Member variables representing motor speeds
        int FL, FR, RL, RR;
        // Constructor to initialise member variables
        MDCMotorCommand() : FL(0), FR(0), RL(0), RR(0) {}
};

// Define class to determine motor speeds
class MDCController
{
    public:
        // Constructor initialising member variables SET AS DESIRED
        MDCController() : max_motor_speed(0), ticks_per_meter(0), wheel_separation_width(0), wheel_separation_length(0) {}

        // Method to calculate motor speeds - returns values within a MotorCommand object 
        MDCMotorCommand motorSpeed(int x_lin, int y_lin, int z_ang)
        {
            // Initialising MotorCommand instance/object motor_speed to store motor speeds
            MDCMotorCommand motor_speed;

            // Calculating motor speed for each wheel_separation_length
            motor_speed.FL = ticks_per_meter * (x_lin - y_lin - (wheel_separation_width + wheel_separation_length) * z_ang);
            motor_speed.FR = ticks_per_meter * (x_lin + y_lin + (wheel_separation_width + wheel_separation_length) * z_ang);
            motor_speed.RL = ticks_per_meter * (x_lin + y_lin - (wheel_separation_width + wheel_separation_length) * z_ang);
            motor_speed.RR = ticks_per_meter * (x_lin - y_lin + (wheel_separation_width + wheel_separation_length) * z_ang);

            // Proportionally adjust motor speed if exceeding set maximum
            // Finds highest motor speed and compares to set maximum
            if (std::max({motor_speed.FL, motor_speed.FR, motor_speed.RL, motor_speed.RR}) > max_motor_speed)
            {
                // Calculate factor required to scale motor speeds below set maximum
                // Maximum is casted to type double to allow for a result of type double (double / int = double) 
                double factor = static_cast<double>(max_motor_speed) / std::max({motor_speed.FL, motor_speed.FR, motor_speed.RL, motor_speed.RR});
                // Multiply motor speeds by scale factor 
                motor_speed.FL *= factor;
                motor_speed.FR *= factor;
                motor_speed.RL *= factor;
                motor_speed.RR *= factor;
            }

            // Converts/casts motor speed from double to expected integer type (because in prior scaling: int * double = double)
            motor_speed.FL = static_cast<int>(motor_speed.FL);
            motor_speed.FR = static_cast<int>(motor_speed.FR);
            motor_speed.RL = static_cast<int>(motor_speed.RL);
            motor_speed.RR = static_cast<int>(motor_speed.RR);

            // Return calculated motor speeds
            return motor_speed;
        }

        // Method to set robot parameters
        void setMaxMotorSpeed(int mms)
        {
            max_motor_speed = mms;
        }

        void setTicksPerMeter(int tpm)
        {
            ticks_per_meter = tpm;
        }

        void setWheelSeparationWidth(int wsw)
        {
            wheel_separation_width = wsw;
        }

        void setWheelSeparationLength(int wsl)
        {
            wheel_separation_length = wsl;
        }

    private:
        // Define robot parameters member variables
        int max_motor_speed; // Motor speed in ticks per second
        int ticks_per_meter; // Encoder ticks per meter traveled
        int wheel_separation_width; // Wheel separation width in meters
        int wheel_separation_length; // Wheel separation length in meters
};

#endif