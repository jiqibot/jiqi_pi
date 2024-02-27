#!/usr/bin/env python3

import rospy
# This library is used as we are receiving Int32 messages - used as velocities are in the range 0-255
from std_msgs.msg import Int16MultiArray

# Callback functions are called everytime a message is received - prints wheel speeds to screen
def callback_pwm(motor_pwm):
    print("FL: %s" %motor_pwm.data[0], end="")
    print(", FR: %s" %motor_pwm.data[1], end="")
    print(", RL: %s" %motor_pwm.data[2], end="")
    print(", RR: %s" %motor_pwm.data[3])

# Initialise subscriber node - setting 'anonymous=True' to ensure node has unique name - ensured via adding random
    # numbers to end of node name
rospy.init_node('motor_pwm_listener', anonymous=True)

# Subscribe to the appropriate topics - names must match with Arduino code, specify type of message to be received and
    # callback function to be used
rospy.Subscriber("motor_pwm_data", Int16MultiArray, callback_pwm)

# Spin the code
rospy.spin()