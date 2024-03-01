#ifndef MDC_CONTROLLER_HPP
#define MDC_CONTROLLER_HPP

// Included to allow use of std::max function
#include <algorithm>

// Define class to hold motor control commands
class MDCMotorCommand
{
    public:
        // Member variables representing motor speeds
        double FL, FR, RL, RR;
        // Constructor to initialise member variables
        MDCMotorCommand() : FL(0), FR(0), RL(0), RR(0) {}
};

// Define class to determine motor speeds
class MDCController
{
    public:
        // Constructor initialising member variables SET AS DESIRED
        MDCController() : max_motor_speed(0), pulses_per_meter(0), wheel_separation_width(0), wheel_separation_length(0) {}

        // Method to calculate motor speeds - returns values within a MotorCommand object 
        MDCMotorCommand motorSpeed(double x_lin, double y_lin, double z_ang)
        {
            // Initialising MotorCommand instance/object motor_speed to store motor speeds
            MDCMotorCommand motor_speed;

            // Calculating motor speed for each wheel_separation_length
            motor_speed.FL = pulses_per_meter * (x_lin - y_lin - (wheel_separation_width + wheel_separation_length) * z_ang);
            motor_speed.FR = pulses_per_meter * (x_lin + y_lin + (wheel_separation_width + wheel_separation_length) * z_ang);
            motor_speed.RL = pulses_per_meter * (x_lin + y_lin - (wheel_separation_width + wheel_separation_length) * z_ang);
            motor_speed.RR = pulses_per_meter * (x_lin - y_lin + (wheel_separation_width + wheel_separation_length) * z_ang);

            // Proportionally adjust motor speed if exceeding set maximum
            double max_abs_speed = std::max({std::abs(motor_speed.FL), std::abs(motor_speed.FR), std::abs(motor_speed.RL), std::abs(motor_speed.RR)});
            // Finds highest motor speed and compares to set maximum
            if (max_abs_speed > max_motor_speed)
            {
                // Calculate factor required to scale motor speeds below set maximum
                // Maximum is casted to type double to allow for a result of type double (double / int = double) 
                double factor = static_cast<double>(max_motor_speed) / max_abs_speed;
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

        void setPulsesPerMeter(int tpm)
        {
            pulses_per_meter = tpm;
        }

        void setWheelSeparationWidth(double wsw)
        {
            wheel_separation_width = wsw;
        }

        void setWheelSeparationLength(double wsl)
        {
            wheel_separation_length = wsl;
        }

    private:
        // Define robot parameters member variables
        int max_motor_speed; // Motor speed in pulses per second
        int pulses_per_meter; // Encoder pulses per meter traveled
        double wheel_separation_width; // Wheel separation width in meters
        double wheel_separation_length; // Wheel separation length in meters
};

#endif