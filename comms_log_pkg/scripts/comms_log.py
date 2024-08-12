#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from datetime import datetime
import subprocess
import os
import pytz

class CommsMonitorNode:
    def __init__(self):
        rospy.init_node('comms_monitor_node')

        self.subscriber_pose = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.callback_pose_info)
        self.pub_wifi_status = rospy.Publisher('iswifion', String, queue_size=10)

        self.if_wifi_On = False
        self.is_first_occurance = True
        self.curr_pose = None

    def callback_pose_info(self, msg):
        self.curr_pose = msg.pose.pose

    def monitor_comms_activity(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            
            # avoid printing on the terminal
            with open(os.devnull, 'w') as devnull:
                iwConfigData = subprocess.check_output(['iwconfig'], stderr=devnull)

            # this if condition is hard coded please change it to make it more robust
            if b'utexas' not in iwConfigData:
                self.if_wifi_On = False

                if self.is_first_occurance:
                    now = datetime.now(pytz.timezone('America/Chicago'))
                    dt_string = now.strftime("%d/%m/%Y %H:%M:%S")

                    # log the connection lost date and time
                    f = open("src/bwi_common/comms_log_pkg/dataLog/comms_drop_daytime_data.txt", "a")
                    f.write(dt_string)
                    if self.curr_pose != None:
                        f.write('\t X: ')
                        f.write(str(self.curr_pose.position.x))
                        f.write('\t Y: ')
                        f.write(str(self.curr_pose.position.y))
                        f.write('\t W: ')
                        f.write(str(self.curr_pose.orientation.w))

                    f.write('\n')
                    f.close()
                    self.is_first_occurance = False            
            else:
                self.if_wifi_On = True
                self.is_first_occurance = True
                
            self.pub_wifi_status.publish(str(self.if_wifi_On))

            rate.sleep()
    
if __name__ == "__main__":
    try:
        node = CommsMonitorNode()
        node.monitor_comms_activity()
    except rospy.ROSInterruptException:
        pass
