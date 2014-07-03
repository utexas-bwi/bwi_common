#!/usr/bin/env python

import roslib; roslib.load_manifest('rospy_tutorials')

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16
from map_mux.srv import *

def talker():
    rospy.init_node('talker', anonymous=True)
    print 12
    rospy.wait_for_service('change_map')
    print 13

    #pub = rospy.Publisher('change_map', Int16)
    r = rospy.Rate(.2) # 10hz
    i = 1
    while not rospy.is_shutdown():
        if i == 1:
            i = 2
        elif i == 2:
            i = 3
        elif i == 3:
            i = 1
        try:
            change_map = rospy.ServiceProxy('change_map', ChangeMap)
#        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
            ii = change_map(i)
            print i
            print "..."+str(ii)
        except rospy.ServiceException , e:
            print "failure"

        #rospy.loginfo(i)
        #pub.publish(i)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass




##!/usr/bin/env python
#import roslib; roslib.load_manifest('beginner_tutorials')
#
#import sys
#
#import rospy
#from beginner_tutorials.srv import *
#
#def add_two_ints_client(x, y):
#    rospy.wait_for_service('add_two_ints')
#    try:
#        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
#        resp1 = add_two_ints(x, y)
#        return resp1.sum
#    except rospy.ServiceException, e:
#        print "Service call failed: %s"%e
#
#def usage():
#    return "%s [x y]"%sys.argv[0]
#
#if __name__ == "__main__":
#    if len(sys.argv) == 3:
#        x = int(sys.argv[1])
#        y = int(sys.argv[2])
#    else:
#        print usage()
#        sys.exit(1)
#    print "Requesting %s+%s"%(x, y)
#    print "%s + %s = %s"%(x, y, add_two_ints_client(x, y))
