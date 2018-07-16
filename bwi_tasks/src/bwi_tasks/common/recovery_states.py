from bwi_tasks.common import control_flow
from smach import State

class DecomposeRecovery(control_flow.TransitionBasedOnUserdata):
    def __init__(self, forward_to):
        control_flow.TransitionBasedOnUserdata.__init__(self, "result", forward_to)

    def extract_from_userdata(self, userdata):
        # Change transition based on the name of the action that failed
        if not userdata.result:
            return "aborted"
        return userdata.result.final_action.name


recovery_machine_sig = {'outcomes': ['succeeded', 'aborted'], 'input_keys': ["goal", "result"],
                        'output_keys': ["new_goal", "msg_for_operator"]}


def set_recovery_sm_defaults():
    control_flow.inject_userdata_auto("_SET_DEFAULT_NEW_GOAL", "new_goal", None)
    control_flow.inject_userdata_auto("_SET_DEFAULT_MSG_FOR_OPERATOR", "msg_for_operator", None)


class HandleNavigationFailure(State):
    def __init__(self):
        State.__init__(self, **recovery_machine_sig)

    def execute(self, userdata):
        userdata["msg_for_operator"] = "I was blocked while I was on my way"
        return "succeeded"