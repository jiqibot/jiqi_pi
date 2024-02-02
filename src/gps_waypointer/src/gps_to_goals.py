#!/env/usr/python3

import rospy
import yaml # this is needed to create and update yaml files
import math 

pathToYaml = "/home/jiqi/jiqi_pi/src/gps_waypointer/config/dummyWrite.yaml"

with open(pathToYaml, "r") as file: # opens the yaml file and stores the data in dict object
    yamlData = yaml.load(file)
    
def getGpsDistance(lat1, long1, lat2, long2):
    

if __name__ == "__main__":
    node = rospy.init_node
    
    
