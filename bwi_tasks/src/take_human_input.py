#!/usr/bin/python

import time
import rospy

import roslib; roslib.load_manifest('bwi_tasks')

import actionlib
from bwi_kr_execution.msg import *
import segbot_gui.srv

from multiprocessing import Process, Value

human_waiting = False
next_room = None

def get_human_waiting():
    global human_waiting
    return human_waiting

def set_human_waiting(value):
    global human_waiting
    human_waiting = value

# option 
# 1: click me if you need help
# 2: please let me know your goal place
def gui_thread(human_waiting):
  
    rospy.init_node('human_input_gui_thread')

    print("gui_thread started")

    rospy.wait_for_service('question_dialog')
    
    while not rospy.is_shutdown():

        if human_waiting.value == True:
            rospy.sleep(2)
            continue

        try:
            handle = rospy.ServiceProxy('question_dialog', segbot_gui.srv.QuestionDialog)
            res = handle(1, "Please click the button, if you need my help.", ["Button"], 0)
            human_waiting.value = True
            res = handle(0, "Follow me please, I am moving fast.", ["Button"], 0)

        except rospy.ServiceException, e:
            print ("Service call failed")
    
    return True

def platform_thread(human_waiting):

    rospy.init_node('human_input_platform_thread')
    rospy.wait_for_service('question_dialog')

    print("platform_thread started")

    rooms = ["414a", "414b", "416", "418", "420"]
    doors = ["d3_414a1", "d3_414b1", "d3_416", "d3_418", "d3_420"]
    
    client = actionlib.SimpleActionClient('/action_executor/execute_plan',\
            ExecutePlanAction)
    client.wait_for_server()


    while not rospy.is_shutdown():
        print("human_waiting: " + str(human_waiting))

        if human_waiting.value == True:
            print("Human is waiting. Let me see where the goal is.")

            try:
                handle = rospy.ServiceProxy('question_dialog', segbot_gui.srv.QuestionDialog)
                res = handle(1, "Where's your goal? \n\nPlease select the room.", rooms, 60)
            except rospy.ServiceException, e:
                print ("Service call failed: %s"%e)

            if res.index == None:

                try:
                    handle = rospy.ServiceProxy('question_dialog', segbot_gui.srv.QuestionDialog)
                    handle(0, "It seems you do not need me anymore. \nI am leaving.", doors, 0)
                except rospy.ServiceException, e:
                    print ("Service call failed: %s"%e)

                loc = doors[0]

            else: 

                try:
                    handle = rospy.ServiceProxy('question_dialog', segbot_gui.srv.QuestionDialog)
                    handle(0, "Follow me please. We are arriving soon. ", doors, 0)
                except rospy.ServiceException, e:
                    print ("Service call failed: %s"%e)

                loc = doors[res.index]

            goal = ExecutePlanGoal()
            rule = AspRule()
            fluent = AspFluent()
            
            fluent.name = "not beside"
            fluent.variables = [loc]
            rule.body = [fluent]
            goal.aspGoal = [rule]
            
            print("sending goal: " + loc)
            client.send_goal(goal)
    
            client.wait_for_result()

            try:
                handle = rospy.ServiceProxy('question_dialog', segbot_gui.srv.QuestionDialog)
                res = handle(0, "You have arrived. I am leaving. \n\nThank you!", doors, 0)
                rospy.sleep(10)

            except rospy.ServiceException, e:
                print ("Service call failed: %s"%e)

            human_waiting.value = False

        else:
            print("No one needs my help. Let me take a random walk.")

            loc = doors[int(time.time()) % len(rooms)]
            
            goal = ExecutePlanGoal()
            rule = AspRule()
            fluent = AspFluent()
            
            fluent.name = "not beside"
            fluent.variables = [loc]
            rule.body = [fluent]
            goal.aspGoal = [rule]
            
            print("sending goal: " + loc)
            client.send_goal(goal)
    
            client.wait_for_result()


    return 1

if __name__ == '__main__':

  try:

    human_waiting = Value('b', False)

    p1 = Process(target = gui_thread, args = (human_waiting, ))
    p2 = Process(target = platform_thread, args = (human_waiting, ))

    p1.start()
    p2.start()
    p1.join()
    p2.join()

  except:

    rospy.loginfo("Error: unable to start thread")


