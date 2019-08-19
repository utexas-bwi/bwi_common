from actionlib import SimpleActionServer
from actasp_ros.msg import ExecuteSmachStateMachineAction
from smach_ros import action_server_wrapper, ActionServerWrapper


class SmachActionExecutor(ActionServerWrapper):
    def __init__(self,
                 server_name,
                 wrapped_container,
                 succeeded_outcomes=["succeeded"],
                 aborted_outcomes=["aborted"],
                 preempted_outcomes=["preempted"],
                 goal_key='action_goal',
                 feedback_key='action_feedback',
                 result_key='action_result',
                 goal_slots_map={},
                 feedback_slots_map={},
                 result_slots_map={},
                 expand_goal_slots=False,
                 pack_result_slots=False
                 ):
        ActionServerWrapper.__init__(self, server_name, ExecuteSmachStateMachineAction, wrapped_container,
                                     succeeded_outcomes, aborted_outcomes, preempted_outcomes, goal_key, feedback_key,
                                     result_key, goal_slots_map, feedback_slots_map, result_slots_map,
                                     expand_goal_slots, pack_result_slots)
