#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import NavSatFix

curLong = 0
curLat = 0
arrLat = []
arrLong = []
latSum = 0
longSum = 0

def gpsCallout(gpsMsg: NavSatFix):
    global curLong
    global curLat
    curLong = gpsMsg.longitude
    curLat = gpsMsg.latitude
    arrLat.append(curLat)
    arrLong.append(curLong)
    
if __name__ == "__main__":
    rospy.init_node("gps_tolerance")
    rospy.Subscriber("/fix", NavSatFix, callback=gpsCallout)
    
    rospy.spin()
    if rospy.is_shutdown():
        arrLat.sort()
        arrLong.sort()
        for num in arrLat:
            latSum = latSum + num
        avgLat = latSum / len(arrLat)
        for num in arrLong:
            longSum = longSum + num
        avgLong = longSum / len(arrLong)
        rangeLong = arrLong[len(arrLong) - 1] - arrLong[0]
        rangeLat = arrLat[len(arrLat) - 1] - arrLat[0]
        rospy.loginfo(f"lat avg: {avgLat}, lat range: {rangeLat:.10f}, lat high: {arrLat[len(arrLat) - 1]}, lat low: {arrLat[0]}")
        rospy.loginfo(f"long avg: {avgLong}, long range: {rangeLong:.10f}, long high: {arrLong[len(arrLong) - 1]}, long low: {arrLong[0]}")
        rospy.loginfo(f"Num. of Samples: {len(arrLat)}")
        rospy.loginfo("Node killed")