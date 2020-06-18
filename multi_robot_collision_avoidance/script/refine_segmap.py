#!/usr/bin/env python3.7

import os, sys
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
import yaml

import rospy, rospkg

from nav_msgs.srv import GetPlan
from geometry_msgs.msg import PoseStamped

window = 1
hallway_idx = [6,18,21,26,29]

def is_valid_point_client(x,y):
    rospy.wait_for_service("/roberto/move_base/NavfnROS/make_plan")
    try:
        is_valid_point = rospy.ServiceProxy("/roberto/move_base/NavfnROS/make_plan", GetPlan)
        start = PoseStamped()
        goal = PoseStamped()
        start.header.frame_id = goal.header.frame_id = "roberto/level_mux_map"
        start.header.stamp = goal.header.stamp = rospy.Time.now()
        start.pose.position.x = 0
        start.pose.position.y = 10
        goal.pose.position.x = x
        goal.pose.position.y = y

        resp = is_valid_point(start, goal, 0.5);
        # DO SOMETING
        return len(resp.plan.poses)
    except rospy.ServiceException as e:
        print("service call failed: {}".format(e))

pkg_path = rospkg.RosPack().get_path('utexas_gdc')
config_path = os.path.join(pkg_path, 'maps', 'simulation', '3ne')
config = None
with open( os.path.join(config_path, '3ne.yaml'), 'r') as f:
    config = yaml.load(f)
def pix2pose(px,py):
    x = px * 0.05 + config['origin'][0]
    y = (610 - py) * 0.05 + config['origin'][1]
    return x, y

if __name__ == '__main__':
    rospy.init_node('exclude_invalid_points', anonymous=True)

    rospack = rospkg.RosPack()
    pkgPath = rospack.get_path('utexas_gdc')
    mapPath = os.path.join(pkgPath, 'maps','simulation','3ne')
    segPath = os.path.join(mapPath, 'locations.pgm')
    segment = np.array( Image.open(segPath) ).copy()

    idx = np.argwhere(np.isin(segment, hallway_idx))
    print(segment.shape)
    for i, [py,px] in enumerate(idx):
        if(i%10000 == 0):
            print("{}/{}".format(i, idx.shape[0]))
        x,y = pix2pose(px,py)
        length = is_valid_point_client(x,y)
        if(length == 0):
            mask = np.isin(segment[py-window:py+window+1, px-window:px+window+1], hallway_idx)
            segment[py-window:py+window+1,px-window:px+window+1][mask] = 255.

    plt.imsave("new_segmentation.png", segment, cmap='gray')
