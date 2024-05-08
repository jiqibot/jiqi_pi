#!/usr/bin/env python3

import rospy
from random import randint

from jiqi_mdc.msg import jiqi_data

value_fl = int(0)
value_fr = int(0)
value_rl = int(0)
value_rr = int(0)

def dummyEncoder():
    pub = rospy.Publisher("encoder_pulse_data", jiqi_data, queue_size=10)
    rospy.init_node('dummy_encoder', anonymous=True)
    rate = rospy.Rate(5)

    dummy = jiqi_data()
    while not rospy.is_shutdown():
        # Randomly increment encoder value in given range
        global value_fl
        value_fl += randint(-10, 10) 
        global value_fr
        value_fr += randint(-10, 10) 
        global value_rl
        value_rl += randint(-10, 10)
        global value_rr
        value_rr += randint(-10, 10)
        # Set incremented value equal to respective message element
        dummy.fl = value_fl
        dummy.fr = value_fr
        dummy.rl = value_rl
        dummy.rr = value_rr       
        # Publish data
        pub.publish(dummy)
        rate.sleep

if __name__ =='__main__':
    try:
        dummyEncoder()
    except rospy.ROSInterruptException:
        pass