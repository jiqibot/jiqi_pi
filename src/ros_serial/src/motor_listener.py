#!/usr/bin/env python3

import rospy
# This library is used as we are receiving Int32 messages - used as velocities are in the range 0-255
from std_msgs.msg import Int32

# Callback functions are called everytime a message is received - prints motor PWM values to the screen
def callback_r(right_msg):
    print("Right motors PWM: %s" %right_msg.data)

def callback_l(left_msg):
    print("Left motors PWM: %s" %left_msg.data)

# Initialise subscriber node - setting 'anonymous=True' to ensure node has unique name - ensured via adding random
    # numbers to end of node name
rospy.init_node('motor_listener', anonymous=True)

# Subscribe to the appropriate topics - names must match with Arduino code, specify type of message to be received and
    # callback function to be used
rospy.Subscriber("right_motor_pwm", Int32, callback_r)
rospy.Subscriber("left_motor_pwm", Int32, callback_l)

# Spin the code
rospy.spin()
