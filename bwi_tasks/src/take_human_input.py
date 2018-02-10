#!/usr/bin/python

import time
import rospy

import roslib; roslib.load_manifest('bwi_tasks')

import actionlib
import actionlib_msgs.msg
from plan_execution.msg import *
import bwi_msgs.srv
import bwi_rlg.srv
import os.path
import string

from multiprocessing import Process, Value, Array

from PIL import Image
import subprocess

# human_waiting = False
# curr_goal = []
# next_room = None

person_door = {'peter'      : 'd3_508', 
               'dana'       : 'd3_510',
               'ray'        : 'd3_512',
               'raymond'    : 'd3_512',
               'stacy'      : 'd3_502',
               'kazunori'   : 'd3_402',
               'matteo'     : 'd3_418',
               'jivko'      : 'd3_432',
               'shiqi'      : 'd3_420',
               'piyush'     : 'd3_416',
               'daniel'     : 'd3_436'}

room_list = ['l3_502', 'l3_504', 'l3_508', 'l3_510', 'l3_512', 'l3_516',
             'l3_436', 'l3_402', 'l3_404', 'l3_416', 'l3_418', 'l3_420',
             'l3_422', 'l3_430', 'l3_432', 'l3_414a', 'l3_414b']

door_list = ['d3_502', 'd3_504', 'd3_508', 'd3_510', 'd3_512', 'd3_516',
             'd3_436', 'd3_402', 'd3_404', 'd3_416', 'd3_418', 'd3_420',
             'd3_422', 'd3_430', 'd3_432', 'd3_414a1', 'd3_414a2', 'd3_414b1',
             'd3_414b2']

resting_time = 5
cnt = 0
last_loc = ''

def task_guiding(doorname, client, dialog_handle):

    roomname = doorname[doorname.find('_')+1 : ]
    
    if roomname.find('414') >= 0:
        roomname = roomname[:-1]

    roomname = '3.' + roomname

    dialog_handle(0, "I am going to " + roomname + '. ', [], 0)

    goal = ExecutePlanGoal()
    rule = AspRule()
    fluent = AspFluent()
    
    fluent.name = "not facing"
    fluent.variables = [doorname]
    rule.body = [fluent]
    goal.aspGoal = [rule]
    
    rospy.loginfo("Sending goal (doorname): " + doorname)
    client.send_goal(goal)
    client.wait_for_result()

def task_delivery(person, item, client, dialog_handle):

    global person_door

    upper_person = person[0].upper() + person[1:]
    dialog_handle(0, "I am going to pick up " + item + " for " + \
                  upper_person + '. ', [], 0)

    goal = ExecutePlanGoal()
    rule = AspRule()
    fluent = AspFluent()
    
    # going to the shop first - there is no shop - going to the kitchen: 520
    fluent.name = "not facing"
    fluent.variables = ["d3_520"]
    rule.body = [fluent]
    goal.aspGoal = [rule]
    
    rospy.loginfo("Sending goal (doorname): " + fluent.variables[0])
    client.send_goal(goal)

    client.wait_for_result()

    res = dialog_handle(1, "May I have " + item + " please?", \
                 ["Sorry, we do not have that", "Loaded"], 60)

    hasLoaded = (res.index == 1)
    
    if res.index == 1:
        res = dialog_handle(0, "I am taking " + item + " to " + upper_person + \
                        ". ", [""], 0)
    else:
        res = dialog_handle(0, "I am going to tell " + upper_person + \
                            " that " + item + " is not available. ", [""], 0)

    fluent.name = "not facing"
    fluent.variables = [person_door[person]]
    rule.body = [fluent]
    goal.aspGoal = [rule]
    
    # loaded item, now going to the person's place

    rospy.loginfo("Sending goal (doorname): " + person_door[person])
    client.send_goal(goal)
    client.wait_for_result()

    if hasLoaded == True:
        res = dialog_handle(1, "Someone requested this " + item + \
                            " to be delivered to you. Please take it. ", \
                            ["Unloaded"], 60)
    else:
        res = dialog_handle(1, "Someone requested " + item + \
                            " be delivered to you, but the store did not give it to me. ", \
                            ["I see."], 60)

def process_request(query, client, dialog_handle):

    global room_list
    global door_list
    global person_door

    rospy.loginfo("query: " + query)

    if (query.find("at(") >= 0): # this is a guiding task! 

        room = query[query.find('l') : query.find(',')]

        if room in room_list:

            query = query.replace("at(l", "d")
            query = query[:query.find(",")]

            # in case there are multiple doors to a room, select the first one
            if query.find("d3_414") > 0:
                query += "1"

            task_guiding(query, client, dialog_handle)

        else:

            rospy.logwarn("This does not seem to be a room name: " + room)

    elif (query.find("query(") >= 0): # this is a question-asking task! 

        rospy.logwarn("Query from semantic parser: " + query)
        # query = query.replace("query(l", "d")
        # query = query[:query.find(":")]

        # if query.find("d3_414") > 0:
        #     query += "1"

        # task_guiding(query, client, dialog_handle)

        return False

    elif (query.find("served(") >= 0): # this is a delivery task! 

        # served(shiqi,coffee,n)

        person = query[query.find('(')+1 : query.find(',')]

        if person in person_door: 

            # remove the person name -> coffee,n)
            query = query[query.find(',')+1 : ]
            item = query[: query.find(',')]

            task_delivery(person, item, client, dialog_handle)

        else:

            rospy.logwarn("This does not seem to be a person name: " + person)
    
    return True
# option 
# 1: click me if you need help
# 2: please let me know your goal place
def gui_thread(human_waiting, curr_goal):
  
    rospy.init_node('human_input_gui_thread')

    rospy.loginfo("gui_thread started")

    rospy.wait_for_service('question_dialog')
    
    handle = rospy.ServiceProxy('question_dialog', \
                                bwi_msgs.srv.QuestionDialog)

    while not rospy.is_shutdown():

        if human_waiting.value == True:
            rospy.sleep(2)
            continue

        res = handle(1, "", ["Button"], 2)

        while (res.index < 0):

            if len(curr_goal.value) > 6:
                g = "3." + curr_goal.value[3:-1]
            else:
                g = "3." + curr_goal.value[3:]

            res = handle(1, "Please click the button, if you need my help." + \
                         "\n\nI am moving to room " + g, ["Button"], 2)

        human_waiting.value = True
        res = handle(0, "I have to go to room " + g + " first." + \
                        "\n\nWe can chat when I get there.",\
                     ["Button"], 0)

    return True

def platform_thread(human_waiting, curr_goal):

    global cnt
    global resting_time
    global last_loc

    rospy.init_node('human_input_platform_thread')

    rospy.loginfo("platform_thread started")

    rospy.wait_for_service('question_dialog')

    rooms = ["414a", "414b", "414b", "418", "420"]
    doors = ["d3_414a2", "d3_414b1", "d3_414b2", "d3_418", "d3_420"]
    
    client = actionlib.SimpleActionClient('/action_executor/execute_plan',\
                                          ExecutePlanAction)
    client.wait_for_server()

    dialog_handle = rospy.ServiceProxy('question_dialog', \
                                       bwi_msgs.srv.QuestionDialog)

    parser_handle = rospy.ServiceProxy('semantic_parser', \
                                       bwi_rlg.srv.SemanticParser)

    path_rlg = rospy.get_param("/semantic_parser_server/path_to_bwi_rlg")
    filepath = path_rlg + "/agent/dialog/list_of_bad_data.txt"

    while not rospy.is_shutdown():

        if human_waiting.value == True:

            # a human may give goals to the robot one after another
            while True:

                # commumication......
                # robot speaks first

                res_qd = dialog_handle(0, \
                         'At this time, I can only help with navigation and item delivery to named people. Please try not to change your mind about what you want while we chat. ', \
                         [""], 5)
                rospy.sleep(6)

                # img = subprocess.Popen(["eog", \
                #                       path_rlg + '/images/unnamed.jpg'])

                res_sp = parser_handle(0, "STARTING-KEYWORD")

                while len(res_sp.query) == 0 and res_qd.index != -2:
                    
                    # take human feedback
                    res_qd = dialog_handle(2, res_sp.output_text, doors, 60)

                    # robot speaks back
                    res_sp = parser_handle(0, res_qd.text)

                    # the robot prints: I am thinking...
                    dialog_handle(0, "I am thinking...", doors, 2)
                    rospy.sleep(2)

                # now the robot has found the query from semantic parser

                hasSucceeded = process_request(res_sp.query, \
                                               client, dialog_handle)
                
                # img.kill()
                # identify good (and bad) data
                res = dialog_handle(1, "Did I accomplish the goal you were trying to convey?", \
                                    ["Yes", "No"], 60)

                # if no response, assuming it's a wrong one
                if res.index == 0:

                    # incrementally learning
                    dialog_handle(0, "I am learning...", doors, 10)
                    subprocess.call("python " + path_rlg +\
                                    "/agent/dialog/main.py " +\
                                     path_rlg + "/agent/dialog/ " +\
                                     "retrain -exclude_test_goals", shell=True)

                else:

                    if os.path.exists(filepath):
                        f = open(filepath, 'a')
                    else:
                        f = open(filepath, 'w+')

                    res_sp = parser_handle(3, res_qd.text)
                    f.write(res_sp.output_text + '---' + res_sp.query + '\n')
                    f.close()

                # anything else for the same user? 
                res = dialog_handle(1, "Do you need anything else?", \
                                    ["Yes", "No"], 15)

                if res.index < 0 or res.index == 1:

                    human_waiting.value = False
                    break

        else:

            rospy.loginfo("No one needs me. I will do a random walk.")
            rospy.sleep(1)

            if (cnt % resting_time) == 0 or last_loc == '': 

                loc = doors[int(time.time()) % len(rooms)]
                last_loc = loc

            else:

                loc = last_loc

            cnt += 1

            curr_goal.value = loc
            goal = ExecutePlanGoal()
            rule = AspRule()
            fluent = AspFluent()
            
            fluent.name = "not beside"
            fluent.variables = [loc]
            rule.body = [fluent]
            goal.aspGoal = [rule]
            
            rospy.loginfo("sending goal: " + loc)
            client.send_goal(goal)
            
            while client.wait_for_result(
                    timeout = rospy.Duration.from_sec(1.0)) == False:
                if human_waiting.value:
                    break

            client.cancel_goal()

            move_base_pub = rospy.Publisher('move_base/cancel',\
                                            actionlib_msgs.msg.GoalID)

            # for unknown reasons, we have to publish the message at least twice
            # to make it work
            for i in range(5):
                move_base_pub.publish(rospy.Time.now(), '')
                rospy.sleep(0.2)

    return 1

if __name__ == '__main__':

    human_waiting = Value('b', False)
    curr_goal = Array('c', "This is a very very very long string for nothing.")

    p1 = Process(target = gui_thread, args = (human_waiting, curr_goal))
    p2 = Process(target = platform_thread, args = (human_waiting, curr_goal))

    p1.start()
    p2.start()

    p1.join()
    p2.join()

