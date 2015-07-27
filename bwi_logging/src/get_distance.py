#!/usr/bin/env python

import rospy
import rosbag
import time
import datetime
import os
import sys
from time import gmtime, strftime

# if rospy.has_param('bag_file'):
#   bag_file = rospy.get_param('bag_file', '')
#   print('bag_file: ' + bag_file)
# else:
#   exit('please set the bag_file first: "rosparam set bag_file /path/to/your/bag_file"')

if len(sys.argv) != 2:
    print('please run it this way: ./get_distance.py /path/to/the/log_files')
    sys.exit()

path = str(sys.argv[1])
filenames = os.listdir(path)
filenames.sort()
print(filenames)
output_filename = 'zzz.txt'

for filename in filenames:

    if filename.find('_') != 0 or filename.find('.bag') < 0:
        continue

    print('working on: ' + path+filename)
    bag = rosbag.Bag(path+filename)
    
    time_start = -1
    time_consumed = 0
    
    distance_traveled = 0.0
    x = None
    y = None
    
    time_current = -1
    for topic, msg, t in bag.read_messages(topics=['odom']):

      time_current = int(msg.header.stamp.secs)
    
      x_current = float(msg.pose.pose.position.x)
      y_current = float(msg.pose.pose.position.y)
    
      if time_start < 0:
        x = x_current
        y = y_current
        time_start = time_current
        continue
      
      distance_traveled += ((x_current - x)**2 + (y_current - y)**2)**0.5
    
      x = x_current
      y = y_current
    
    time_consumed = time_current - time_start
    print('time_consumed: ' + str(datetime.timedelta(seconds=time_consumed)))
    print('distance_traveled: ' + str(distance_traveled))
    
    bag.close()

    f = open(path+output_filename, 'a')
    f.write(strftime("%Y-%m-%d %H:%M:%S", gmtime()) + \
            '  filename: ' + filename + \
            '  time: ' + str(time_consumed) + \
            '  distance: ' + str(distance_traveled) + '\n')

    f.close()

# end of for

