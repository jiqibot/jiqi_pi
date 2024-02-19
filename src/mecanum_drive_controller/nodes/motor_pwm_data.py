#!/usr/bin/env python3

import rospy
# This library is used as we are receiving Int32 messages - used as velocities are in the range 0-255
from std_msgs.msg import Int16MultiArray

# Callback functions are called everytime a message is received - prints wheel speeds to screen
def interpret(wheel_speed):
    print("FL: ", wheel_speed.data[0], "   FR: ", wheel_speed.data[1])
    print("RL: ", wheel_speed.data[2], "   RR: ", wheel_speed.data[3])

# Initialise subscriber node - setting 'anonymous=True' to ensure node has unique name - ensured via adding random
    # numbers to end of node name
rospy.init_node('interpret_test', anonymous=True)

# Subscribe to the appropriate topics - names must match with Arduino code, specify type of message to be received and
    # callback function to be used
rospy.Subscriber("motor_pwm_data", Int16MultiArray, interpret)

# Spin the code
rospy.spin()
