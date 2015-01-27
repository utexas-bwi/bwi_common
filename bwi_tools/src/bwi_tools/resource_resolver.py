#!/usr/bin/env python

import rospkg
import rospy

def resolveResource(filename):
    if filename.startswith('package://'):
        rest = filename[10:]
        package_name = rest[:rest.find('/')]
        package_relative_file = rest[rest.find('/')+1:]
        if package_name == rest:
            rospy.logfatal("Unable to parse resource %s" %filename)
            return None
        rp = rospkg.RosPack()
        try:
            package_path = rp.get_path(package_name)
        except:
            rospy.logfatal("Requested resource: " + filename + ". Unable to find package " + package_name)
            return None
        filename = package_path + '/' + package_relative_file
    return filename

