// Included with "" instead of <> to indicate user-defined
#include "../include/mdc_controller.hpp"
#include "../include/mdc_pose.hpp"
#include "../include/mdc_encoder.hpp"

// Use of mdc_controller.hpp
int controller()
{
    // Creating an instance of the MDCController class
    MDCController controller;

    // Set motor parameters
    controller.setMaxMotorSpeed(0);
    controller.setTicksPerMeter(0);
    controller.setWheelSeparationWidth(0);
    controller.setWheelSeparationLength(0);

    // Calculate motor speeds for motion (WILL BE cmd_vel TOPIC)
    MDCMotorCommand velocities = controller.motorSpeed(0, 0, 0);

    // Indicate successful program execution
    return 0;
}

// Use of mdc_pose.hpp
int pose()
{
    // Creating an instance of the MDCPose class
    MDCPose pose;

    // Set pose data values (WILL BE RETRIEVED FROM ENCODERS/IMU/OTHER)
    pose.x_lin_pos = 0;
    pose.y_lin_pos = 0;
    pose.z_ang_pos = 0;
    pose.x_lin_vel = 0;
    pose.y_lin_vel = 0;
    pose.z_ang_vel = 0;

    // Convert pose object to string representation
    std::string pose_string = pose.toString();

    // Indicate successful program execution
    return 0;
}

// Use of mdc_encoder.hpp
int encoder()
{
    // Creating an instance of the MDCEncoder class
    MDCEncoder encoder;

    // Set encoder parameters
    encoder.setRange(-0, 0); // Set +/- max encoder ticks
    encoder.initCount(0); // WILL BE RECEIVED FROM ENCODERS
    
    // WILL CHANGE DEPENDENT ON DRIVEN DIRECTION
    encoder.setReversed(false);

    // Update encoder count RECEIVED VIA rosserial TOPIC
    encoder.updateCount(0);

    // Get delta value and encoder range (and thresholds)
    int delta_value = encoder.getDelta();
    std::map<std::string, int> range_data = encoder.getRange();

    // Indicate successful program execution
    return 0;
}