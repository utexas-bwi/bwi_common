import rospy
import os
from smach import State
from smach_ros import SimpleActionState

class PlaySound(State):
    def __init__(self, audio_file):
        State.__init__(self, outcomes=["succeeded", "aborted"])
        self.audio_file = audio_file

    def execute(self, userdata):
        print self.audio_file
        os.system("cvlc --play-and-exit " + self.audio_file)

        return "succeeded"
