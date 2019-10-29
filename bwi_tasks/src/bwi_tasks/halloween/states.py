import rospy
import os
import random
from smach import State
from smach_ros import SimpleActionState

class PlaySound(State):
    def __init__(self, audio_dir, file_names):
        State.__init__(self, outcomes=["succeeded", "aborted"])
        self.audio_dir = audio_dir
	self.file_names = file_names

    def execute(self, userdata):
	audio_file = self.audio_dir + random.choice(self.file_names)
        print audio_file
        os.system("cvlc --play-and-exit " + audio_file)

        return "succeeded"
