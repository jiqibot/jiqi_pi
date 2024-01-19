#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix
import yaml # this is needed to create and update yaml files

#### these imports and code are strictly for reading the keys inputted to the keyboard
#### based on code from teleop_twist_keyboard github
import sys
from select import select
if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty
    
def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)
        
def getKey(settings, timeout):
    global keyInput
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rospy.loginfo(key)
    if key == "":
        return
    else:
        keyInput = key
        return
#### these imports and code are strictly for reading the keys inputted to the keyboard

index = 1
loop = 0
printRate = 3 # determines the printRate of the gps values in seconds
pathToYaml = "/home/jiqi/jiqi_pi/src/gps_waypointer/config/dummy.yaml"
writeToYaml = "/home/jiqi/jiqi_pi/src/gps_waypointer/config/dummyWrite.yaml"
gpsPoints = []
keyInput = "" # will hold the most recent keyboard input

def gpsCallout(gpsMsg: NavSatFix):
    global keyInput
    if keyInput == "r":
        # rospy.loginfo(gpsMsg) # logs the message being received from /fix (nmea_serial_driver)
        gpsPoint = { # had to recreate the format of the NatSatFix msg as a python dict object so that the names in the yaml files aligned with the NatSatFix
            "header": {
                "seq": len(gpsPoints) + 1,
                "stamp": {
                    "secs": gpsMsg.header.stamp.secs,
                    "nsecs": gpsMsg.header.stamp.nsecs
                },
                "frame_id": gpsMsg.header.frame_id,
            },
            "status": {
                "status": gpsMsg.status.status,
                "service": gpsMsg.status.service
            },
            "latitude": gpsMsg.latitude,
            "longitude": gpsMsg.longitude,
            "altitude": gpsMsg.altitude,
            "position_covariance": gpsMsg.position_covariance,
            "position_covariance_type": gpsMsg.position_covariance_type 
        }
        gpsPoints.append(gpsPoint)
        rospy.loginfo(gpsPoint)
        keyInput = ""
    

        

settings = saveTerminalSettings()

if __name__ == "__main__":
    rospy.init_node("gpsRouteSaver") # initialise node with name gpsRouteSaver
    gpsSub = rospy.Subscriber("/fix", NavSatFix, callback=gpsCallout)
    
    rospy.loginfo("Gps Route Saver Node is running!")
    
    rate = rospy.Rate(20)
    
    # with open(pathToYaml, "r") as file:
    #     data = yaml.load(file) # returns the yaml file as a dict (same as object in javaScript)
    
    
    while not rospy.is_shutdown():
        if loop == printRate:
            loop = 0
        loop += 1
        getKey(settings,0.5)
        if keyInput == "c":
            rospy.signal_shutdown("Shutdown by pressing c")
        rate.sleep()
        
    if rospy.is_shutdown(): # when the node ended it will execute in this if statement before it fully dies
        # you need to HOLD ctrl + c to kill this node
        with open(writeToYaml, "w") as file:
            yaml.dump(gpsPoints, file, default_flow_style=False)
        rospy.loginfo("Node killed")
        # rospy.loginfo(index)
        