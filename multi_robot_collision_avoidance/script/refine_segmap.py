#!/usr/bin/env python3.7

import os, sys
import time
import numpy as np
from PIL import Image
import yaml

import rospy, rospkg

from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped, Quaternion

tolerance = 0.1
hallway_idx = [6, 18, 21, 26, 29] # 9: sofa area, 25: printer area

def is_valid_point_client(x,y, tolerance):
    rospy.wait_for_service("/roberto/move_base/NavfnROS/make_plan")
    try:
        is_valid_point = rospy.ServiceProxy("/roberto/move_base/NavfnROS/make_plan", GetPlan)
#        is_valid_point = rospy.ServiceProxy("/roberto/move_base/make_plan", GetPlan)

        start = PoseStamped()
        goal = PoseStamped()
        start.header.frame_id = goal.header.frame_id = "roberto/level_mux_map"
        start.header.stamp = goal.header.stamp = rospy.Time.now()

        start.pose.position.x = 0
        start.pose.position.y = 10
        start.pose.orientation = Quaternion(0,0,0,1)
        
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation = Quaternion(0,0,0,1)

        resp = is_valid_point(start, goal, tolerance);

        return len(resp.plan.poses)
    except rospy.ServiceException as e:
        print("service call failed: {}".format(e))

def pix2pose_generator(segment, configPath):
    with open( configPath, 'r' ) as f:
        config = yaml.load(f)
    h,w = segment.shape
    ox, oy, _ = config['origin']
    res = config['resolution']

    def pix2pose(px,py):
        x = px * res + ox
        y = (h - py) * res + oy
        return x, y
    return pix2pose

if __name__ == '__main__':
    rospy.init_node('exclude_invalid_points', anonymous=True)
    # find map
    pkgPath = rospkg.RosPack().get_path('utexas_gdc')
    mapPath = os.path.join(pkgPath, 'maps','simulation','3ne')
    segPath = os.path.join(mapPath, 'locations.pgm')
    configPath = os.path.join(mapPath, '3ne.yaml')
    # load data
    segment = np.array( Image.open( segPath ) )#.copy()

    pix2pose = pix2pose_generator( segment, configPath ) ###

    idx = np.argwhere(np.isin(segment, hallway_idx))
    print(segment.shape)
    st = time.time()
    for i, [py,px] in enumerate(idx):
        if(i%1000 == 0):
            print("{}/{}: {:.2f}s".format(i, idx.shape[0], time.time() - st))
            st = time.time()
        x,y = pix2pose(px,py)
#        print(x,y)
        length = is_valid_point_client(x,y,tolerance) ###
        if(length == 0):
            segment[py,px] = 255.

    Image.fromarray(segment).convert('RGB').save("new_location_tolerance_{:.1f}.pgm".format(tolerance))
    #plt.imsave("new_segmentation.png", segment, cmap='gray')
