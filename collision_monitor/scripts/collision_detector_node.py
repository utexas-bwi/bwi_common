#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
from std_msgs.msg import Bool
import os
import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
from std_msgs.msg import String
from datetime import datetime
import subprocess
import pytz

# Radius of both the robots
ROBOT_RADIUS = 0.3

class CollisionDetector:
    def __init__(self):
        rospy.init_node('collision_detector_node', anonymous=True)

        self.robot1_pose = None
        self.robot2_pose = None
        
        self.robot1_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.robot1_callback)
        self.robot2_sub = rospy.Subscriber('/comms/amcl_pose', PoseWithCovarianceStamped, self.robot2_callback)

        self.count = 0
        
        # self.collision_happened = False
        self.pub_collision_status = rospy.Publisher('v2collision', String, queue_size=10)

        self.first_collision = True
        
        rospy.spin()

    def robot1_callback(self, msg):
        self.robot1_pose = msg.pose.pose
        self.check_collision()

    def robot2_callback(self, msg):
        self.robot2_pose = msg.pose.pose
        self.check_collision()

    def check_collision(self):
        if self.robot1_pose and self.robot2_pose:
            x1, y1 = self.robot1_pose.position.x, self.robot1_pose.position.y
            x2, y2 = self.robot2_pose.position.x, self.robot2_pose.position.y
            
            distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

            if distance < 2 * ROBOT_RADIUS:
                self.count += 1
                print("\n collision count: ")
                print(self.count)

                now = datetime.now(pytz.timezone('America/Chicago'))
                dt_string = now.strftime("%d/%m/%Y %H:%M:%S")

                # log the connection lost date and time
                f = open("src/bwi_common/collision_monitor/logs/collision_daytime_data.txt", "a")
                f.write(dt_string)
                f.write('\n')
                f.close()                

                self.pub_collision_status.publish(str(True))
                if self.first_collision:
                    self.send_email()
                    self.first_collision = False
            else:
                self.pub_collision_status.publish(str(False))

    def send_email(self):
        sender_email = 'nikunjparmar828@gmail.com'
        sender_password = 'xhdu ypdp dbxv leey'
        receiver_email = 'nikunjparmar828@gmail.com'
        # Create the email
        msg = MIMEMultipart()
        msg['From'] = sender_email
        msg['To'] = receiver_email
        msg['Subject'] = 'BENDER Collision!!'

        # Email body
        body = 'The v2s are collided'
        msg.attach(MIMEText(body, 'plain'))
        # Connect to the server
        server = smtplib.SMTP('smtp.gmail.com', 587)
        server.starttls()

        try:
            # Login to the email account
            server.login(sender_email, sender_password)
            # Send the email
            server.sendmail(sender_email, receiver_email, msg.as_string())
            print("Email sent successfully!")
        except Exception as e:
            # print(f"Failed to send email: {e}")
            print("failed!")
        finally:
            server.quit()


if __name__ == '__main__':
    try:
        CollisionDetector()
    except rospy.ROSInterruptException:
        pass