#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry

# This node combines the PoseStamped and Twist messages from the hector_mapping package and any topic that is publishing a Twist message to create an Odometry message. The GPS requires a Odometry message to function. The message will be published on a topic

def poseCallback(msg: PoseStamped):
    rospy.loginfo()
    
def twistCallback(msg: Twist):
    rospy.loginfo("")
    
def publishOdom(msg: Odometry):
    rospy.loginfo("")
    
    twistData = Twist()
    poseData = PoseStamped()
    odomData = Odometry()

if __name__ == "__main__":
    
    rospy.init_node("odomPublisher")
    poseSub = rospy.Subscriber("/slam_out_pose", PoseStamped, callback=poseCallback) # Subscribes to the hector slam topic which is sending PoseStamped messages
    
    twistSub = rospy.Subscriber("/cmd_vel", Twist, callback=twistCallback) # Subscribes to any topic which is sending Twist messages messages
    
    odomPub = rospy.Publisher("odom_pub", Odometry, queue_size=10)
    
    rospy.loginfo("Odom pubsliher running")
    
    