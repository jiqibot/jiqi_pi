#!/usr/bin/env python3

import rospy
# This library is used as we are receiving Int16MultiArray messages - used as velocities are in the range 0-255
from mecanum_drive_controller.msg import motor_data

# Callback functions are called everytime a message is received - prints wheel speeds to screen
def callback_pps(motor_pps):
    print("FL: %s" %motor_pps.fl, end="")
    print(", FR: %s" %motor_pps.fr, end="")
    print(", RL: %s" %motor_pps.rl, end="")
    print(", RR: %s" %motor_pps.rr)

# Initialise subscriber node - setting 'anonymous=True' to ensure node has unique name - ensured via adding random
    # numbers to end of node name
rospy.init_node('motor_pps_listener', anonymous=True)

# Subscribe to the appropriate topics - names must match with Arduino code, specify type of message to be received and
    # callback function to be used
rospy.Subscriber("motor_pps_data", motor_data, callback_pps)

# Spin the code
rospy.spin()
