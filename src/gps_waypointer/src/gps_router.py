#!/usr/bin/env python3
import rospy
import yaml
from sensor_msgs.msg import NavSatFix

yamlData = []
gpsPoints = []
currentLocation = NavSatFix()
index = 1
avgLongRange = 0.0001 # this is the range longitude vals fluctuate at single point using our gps
avgLatRange = 0.00006 # this is the range latitude vals fluctuate at single point using our gps
rangeLongDivider = 3.4 # determines the range around the goal gps waypoint that counts as the robot reaching the waypoint
rangeLatDivider = 2.6 # determines the range around the goal gps waypoint that counts as the robot reaching the waypoint

pathToYaml = "/home/jiqi/jiqi_pi/src/gps_waypointer/config/dummyWrite.yaml" # path to yaml file where gps coords are stored for route

def trackGps(gpsMsg: NavSatFix): # this is callback that stores the current location of the robot. The current location will be compared with wapoint gps point
    global currentLocation
    currentLocation = gpsMsg
    # rospy.loginfo(currentLocation)

def longGpsTolerance(pose, goal):
    global avgLongRange
    if pose >= (goal - (avgLongRange / rangeLongDivider)) and pose <= (goal + (avgLongRange / rangeLongDivider)):
        # return {
        #     "longHigh": goal + (avgLongRange / rangeDivider),
        #     "long": pose,
        #     "longLow": goal - (avgLongRange / rangeDivider),
        # }
        return f"Longitude Distance Away: {(goal - pose):.10f} Tolerance (+-): {(avgLongRange / rangeLongDivider):.10f}"
        return "reached"
    else:
        # return f"Longitude Distance Away: {(pose):.10f}"
        return "false"

def latGpsTolerance(pose, goal):    
    global avgLatRange
    if pose >= (goal - (avgLatRange / rangeLatDivider)) and pose <= (goal + (avgLatRange / rangeLatDivider)):
        # return {
        #     "LatHigh": goal + (avgLatRange / rangeDivider),
        #     "Lat": pose,
        #     "LatLow": goal - (avgLatRange / rangeDivider),
        # }
        return f"Latitude Distance Away: {(goal - pose):.10f} Tolerance (+-): {(avgLatRange / rangeLatDivider):.10f}"
        return "reached"
    else:
        # return f"Latitude Distance Away: {(pose):.10f}"
        return "false"

with open(pathToYaml, "r") as file: # opens the yaml file and stores the data in dict object
    yamlData = yaml.load(file)
    
for point in yamlData: # converts the array of dict gps data into a ROS NatSatFix message
    navPoint = NavSatFix()
    navPoint.header.seq = point["seq"]
    navPoint.header.stamp.secs = point["secs"]
    navPoint.header.stamp.nsecs = point["nsecs"]
    navPoint.header.frame_id = point["frame_id"]
    navPoint.status.status = point["status"]
    navPoint.status.service = point["service"]
    navPoint.altitude = point["altitude"]
    navPoint.latitude = point["latitude"]
    navPoint.longitude = point["longitude"]
    navPoint.position_covariance = point["position_covariance"]
    navPoint.position_covariance_type = point["position_covariance_type"]
    gpsPoints.append(navPoint)
    
if __name__ == "__main__":
    rospy.init_node("gps_waypoint_router")
    rospy.Subscriber("/fix", NavSatFix, callback=trackGps) # subs to the nmea_serial_driver node to receive the current gps data
    rospy.loginfo("Gps waypoint router node is running!")
    # rospy.loginfo(avgLongRange / 20)
    # rospy.loginfo(avgLatRange / 20)
    while not index >= len(gpsPoints):
        for point in gpsPoints:
            longState = "false"
            latState = "false"
            while longState == "false" or latState == "false":
            #while currentLocation.longitude != point.longitude and currentLocation.latitude != point.latitude:
                longState = longGpsTolerance(currentLocation.longitude, point.longitude)
                latState = latGpsTolerance(currentLocation.latitude, point.latitude)
                # rospy.loginfo(point.longitude)
                # rospy.loginfo(currentLocation.longitude)
                # rospy.loginfo(point.latitude)
                # rospy.loginfo(currentLocation.latitude)
                rospy.loginfo(longState)
                # rospy.loginfo(f"{(avgLongRange / 2):.10f}")
                rospy.loginfo(latState)
                # rospy.loginfo(f"{(avgLatRange / 2):.10f}")
                pass
            rospy.loginfo(f"GPS Waypoint Reached! {index} of {len(gpsPoints)}")
            index += 1
            rospy.loginfo(longState)
            rospy.loginfo(latState)
            
    if rospy.is_shutdown():
        rospy.loginfo("Node killed")