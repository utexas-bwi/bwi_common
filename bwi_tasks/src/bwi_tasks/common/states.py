import rospy
from plan_execution.msg import ExecutePlanAction
from smach import State
from smach_ros import SimpleActionState

from plan_execution.helpers import *

topics = {'plan_execution': "/plan_executor/execute_plan"}


class Wait(State):
    def __init__(self, amount):
        State.__init__(self, outcomes=["succeeded"])
        self.amount = amount

    def execute(self, ud):
        rospy.sleep(self.amount)
        return "succeeded"


class NoOp(Wait):
    def __init__(self):
        Wait.__init__(self, 0)


class ExecuteGoal(SimpleActionState):
    def __init__(self):
        SimpleActionState.__init__(self, topics["plan_execution"],
                                   ExecutePlanAction,
                                   goal_cb=self.goal_cb,
                                   result_cb=self.result_cb,
                                   input_keys=['goal'],
                                   output_keys=['result'])

    def goal_cb(self, userdata, goal):
        goal.aspGoal = userdata.goal.aspGoal

    def result_cb(self, userdata, state, result):
        print (state, result)
        userdata['result'] = result

    """
    def _goal_feedback_cb(self, feedback):
        rospy.logerr("GOT FEEDBACK :" + str(feedback))
        ACTION_START_TYPE = 2
        speech = None
        try:
            speech = villa_audio.tts.TextToSpeech(timeout=5.0)
        except RuntimeError:
            return

        if feedback.event_type == ACTION_START_TYPE:
            if feedback.plan[0].name == "navigate_to":
                speech.say("I'm starting to move.", wait=False)
            if feedback.plan[0].name == "perceive_surface":
                speech.say("I'm starting to scan.", wait=False)
    """