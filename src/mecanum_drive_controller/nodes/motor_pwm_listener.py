#!/usr/bin/env python3

import rospy
# This library is used as we are receiving Float32MultiArray messages - used as velocities are in the range 0-255
from mecanum_drive_controller.msg import pwm_data

# Callback functions are called everytime a message is received - prints wheel speeds to screen
def callback_pwm(motor_pwm):
    print("FL: %s" %motor_pwm.fl, end="")
    print(", FR: %s" %motor_pwm.fr, end="")
    print(", RL: %s" %motor_pwm.rl, end="")
    print(", RR: %s" %motor_pwm.rr)

# Initialise subscriber node - setting 'anonymous=True' to ensure node has unique name - ensured via adding random
    # numbers to end of node name
rospy.init_node('motor_pwm_listener', anonymous=True)

# Subscribe to the appropriate topics - names must match with Arduino code, specify type of message to be received and
    # callback function to be used
rospy.Subscriber("motor_pwm_data", pwm_data, callback_pwm)

# Spin the code
rospy.spin()