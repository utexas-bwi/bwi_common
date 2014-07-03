#!/usr/bin/env python

from map_mux.srv import *
import rospy
import time
import rosparam
import rospkg
import os
from std_msgs.msg import String
from std_msgs.msg import Int16
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from nav_msgs.msg import MapMetaData
from geometry_msgs.msg import PoseWithCovarianceStamped
#from map_mux.srv import ChangeMap
map1 = None
map2 = None
map3 = None
change_map = None
flag = None
# Subscribe to
    # TOPIC : 3 maps
    # SERVICE : Change map
    #
# Send out
    # TOPIC: /map
    # SERVICE : /static_map
    # /inital_pose
    # /clear_cosmaps

# assume maps are loaded to topics map1, map2, map3

initalPositionFloor2 = PoseWithCovarianceStamped()
initalPositionFloor2.header.seq = 0
#initalPositionFloor2.header.stamp.sec = 0
#initalPositionFloor2.header.stamp.nsec = 0
initalPositionFloor2.header.frame_id = "map"
initalPositionFloor2.pose.pose.position.x = 24.46
initalPositionFloor2.pose.pose.position.y = 29.62
initalPositionFloor2.pose.pose.position.z = 0
initalPositionFloor2.pose.pose.orientation.x = 0
initalPositionFloor2.pose.pose.orientation.y = 0
initalPositionFloor2.pose.pose.orientation.z = 0.76631578005
initalPositionFloor2.pose.pose.orientation.w = 0.642464104247
initalPositionFloor2.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]

initalPositionFloor3 = PoseWithCovarianceStamped()
initalPositionFloor3.header.seq = 0
initalPositionFloor3.header.frame_id = "map"
initalPositionFloor3.pose.pose.position.x = 32.85366
initalPositionFloor3.pose.pose.position.y = 7.262574
initalPositionFloor3.pose.pose.position.z = 0
initalPositionFloor3.pose.pose.orientation.x = 0
initalPositionFloor3.pose.pose.orientation.y = 0
#initalPositionFloor3.pose.pose.orientation.z = 0.704652683065
#initalPositionFloor3.pose.pose.orientation.w = 0.709552391476
initalPositionFloor3.pose.pose.orientation.z = 0.6154
initalPositionFloor3.pose.pose.orientation.w = 0.78819
initalPositionFloor3.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]


def map_mux():

    rospy.init_node("map_mux", anonymous=True)
    # Topic Subscriber
    rospy.Subscriber("map1", OccupancyGrid, addMap1)
    rospy.Subscriber("map2", OccupancyGrid, addMap2)
    rospy.Subscriber("map3", OccupancyGrid, addMap3)
    # Topic Publisher
    topic = rospy.resolve_name("map")
    pub = rospy.Publisher("map", OccupancyGrid)
    topic = rospy.resolve_name("map_metadata")
    pub_metadata = rospy.Publisher("map_metadata", MapMetaData)
    initialPosePub = rospy.Publisher("initialpose",  PoseWithCovarianceStamped)
    global change_map
    global flag
    # Service Servers
    s = rospy.Service('change_map', ChangeMap, changeMapfunc)
    s = rospy.Service('static_map', GetMap, staticMapfunc)
    # Service Clients
    service_floor_switch = rospy.ServiceProxy("floor_switch", ChangeMap)
    rp = rospkg.RosPack()
    try:
        path = rp.get_path("map_mux") + "/src"
    except rospkg.ResourceNotFound:
        print "package not found"
    print path
    print "waiting for floor_switch to come up"
    #rospy.wait_for_service("floor_switch")


    r = rospy.Rate(.1)
    old_change_map = 0

    while not rospy.is_shutdown():
        if( map1 == None  and map2 == None and map3 == None):
            print "you need to provide 3 maps on topics map1 map2 map3 from a launch file."
        #if (change_map != old_change_map):
        if (1):
            if (change_map == 1 and map1 != None):
                rospy.loginfo("changing to 1")
                pub.publish(map1)
                pub_metadata.publish(map1.info)
            if (change_map == 2 and map2 != None  and flag ==1 ):
                rospy.loginfo("changing to 2")
                pub.publish(map2)
                pub_metadata.publish(map2.info)
                #print str(type(initalPositionFloor2))
                try:
                    initialPosePub.publish(initalPositionFloor2)
                    rosparam.set_param("/segbot_logical_navigator/door_file",
                            rp.get_path("bwi_kr") + "/config/multi_map/atrium_doors.yaml")
                    rosparam.set_param("/segbot_logical_navigator/location_file",
                            rp.get_path("bwi_kr") + "/config/multi_map/atrium_locations.yaml")
                    rosparam.set_param("/segbot_logical_navigator/object_file",
                            rp.get_path("bwi_kr") + "/config/multi_map/atrium_objects.yaml")
                    rosparam.set_param("/segbot_logical_navigator/map_file",
                            rp.get_path("map_mux") + "/maps/atrium_with_elevators.yaml")

                    resp_floor_switch = service_floor_switch(change_map)
                except rospy.ServiceException:
                    print "floor switch service call failed"
                print "done with floor 2"
                flag = 0
            if (change_map == 3 and map3 != None and flag ==1 ):
                rospy.loginfo("changing to 3")
                pub.publish(map3)
                pub_metadata.publish(map3.info)
                try:
                    initialPosePub.publish(initalPositionFloor3)
                    rosparam.set_param("/segbot_logical_navigator/door_file",
                            rp.get_path("bwi_kr") + "/config/multi_map/3ne_doors.yaml")
                    rosparam.set_param("/segbot_logical_navigator/location_file",
                            rp.get_path("bwi_kr") + "/config/multi_map/3ne_locations.yaml")
                    rosparam.set_param("/segbot_logical_navigator/object_file",
                            rp.get_path("bwi_kr") + "/config/multi_map/3ne_objects.yaml")
                    rosparam.set_param("/segbot_logical_navigator/map_file",
                            rp.get_path("map_mux") + "/maps/map_whole2_with_elevators.yaml")

                    resp_floor_switch = service_floor_switch(change_map)
                except rospy.ServiceException:
                    print "floor switch service call failed"
                print "done with floor 3"
                flag = 0
        old_change_map = change_map
        #rospy.sleep(1)
        #rospy.spin once()
        #r.sleep()
        #rospy.spin()


def addMap1(data):
    #rospy.loginfo( type(data.data))
    global map1
    map1 = data
def addMap2(data):
    global map2
    map2 = data
def addMap3(data):
    global map3
    map3 = data
def changeMapfunc( data):
    print "the service made it here"
    global change_map
    global flag
    flag = 1
    change_map = data.data
    while(1):
        if (flag == 0): break
    return 1
def staticMapfunc( data):
    global change_map
    return change_map

if __name__ == '__main__':
    try:
        map_mux()
    except rospy.ROSInterruptException: pass
    rospy.spin()

#def handle_add_two_ints(req):
    #print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
    #return AddTwoIntsResponse(req.a + req.b)

#def add_two_ints_server():
    #rospy.init_node('add_two_ints_server')
    #s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    #print "Ready to add two ints."
    #rospy.spin()

#if __name__ == "__main__":
    #add_two_ints_server()
