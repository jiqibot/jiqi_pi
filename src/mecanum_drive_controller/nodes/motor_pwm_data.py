#!/usr/bin/env python3

import rospy
# This library is used as we are receiving Int32 messages - used as velocities are in the range 0-255
from std_msgs.msg import Int16

# Callback functions are called everytime a message is received - prints wheel speeds to screen
def callback_fl(motor_pwm):
    print("FL: %s" %motor_pwm.data)

# Initialise subscriber node - setting 'anonymous=True' to ensure node has unique name - ensured via adding random
    # numbers to end of node name
rospy.init_node('interpret_test', anonymous=True)

# Subscribe to the appropriate topics - names must match with Arduino code, specify type of message to be received and
    # callback function to be used
rospy.Subscriber("fl_motor_pwm_data", Int16, callback_fl)

# Spin the code
rospy.spin()
