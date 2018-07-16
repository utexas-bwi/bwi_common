import copy

from plan_execution.msg import ExecutePlanGoal
from smach import StateMachine, State

from bwi_tasks.common import states, recovery_states
from bwi_tasks.common import control_flow

task_sm_args = {"outcomes": ["succeeded", "preempted", "aborted"],
                "input_keys": [],
                "output_keys": ["msg_for_operator"]}
shared_recover_from_failure_sm = None
shared_execute_sm = None


def generate_goal_based_task_sm(goal_formulator, input_keys):
    assert isinstance(goal_formulator, FormulateGoal)

    sm = StateMachine(**gen_task_sig(input_keys))
    with sm:
        StateMachine.add_auto("FORMULATE_GOAL", goal_formulator, ['succeeded'])
        StateMachine.add("EXECUTE_GOAL", get_execute_sm())
    return sm


class FormulateGoal(State):
    def __init__(self, input_keys):
        State.__init__(self, outcomes=["succeeded", 'aborted'], input_keys=input_keys, output_keys=["goal"])

    def execute(self, ud):
        goal = ExecutePlanGoal()
        self.formulate_goal(ud, goal)
        ud.goal = goal
        return "succeeded"

    def formulate_goal(self, userdata, goal):
        raise NotImplementedError()

def set_task_sm_defaults():
    control_flow.inject_userdata_auto("_SET_DEFAULT_NEW_GOAL", "new_goal", None)
    control_flow.inject_userdata_auto("_SET_DEFAULT_MSG_FOR_OPERATOR", "msg_for_operator", None)


def gen_task_sig(input_keys):
    template = copy.deepcopy(task_sm_args)
    template["input_keys"] += input_keys
    return template


def get_shared_recover_from_failure_sm():
    global shared_recover_from_failure_sm
    if not shared_recover_from_failure_sm:
        raise RuntimeError("Must instantiate recovery state machine before using shared instance")
    return shared_recover_from_failure_sm


def get_recover_from_failure_sm(ltmc):
    # This state machine is large, so we will only instantiate once and point references to one copy
    global shared_recover_from_failure_sm
    if shared_recover_from_failure_sm:
        return shared_recover_from_failure_sm

    sm = StateMachine(outcomes=['succeeded', 'aborted'], input_keys=["result", "goal"],
                      output_keys=["new_goal", "msg_for_operator"])
    with sm:
        recover_transitions = {'navigate_to': "HANDLE_NAVIGATE_TO_FAILURE",
                               'aborted': 'SET_DEFAULT_MESSAGE'}
        StateMachine.add("DECOMPOSE_RECOVERY", recovery_states.DecomposeRecovery(recover_transitions.keys()),
                         transitions=recover_transitions)

        StateMachine.add('HANDLE_NAVIGATE_TO_FAILURE', recovery_states.HandleNavigationFailure())
        control_flow.inject_userdata("SET_DEFAULT_MESSAGE", "aborted", "msg_for_operator",
                                     "I could not complete the task because I encountered an unknown error. I am sorry.")
    shared_recover_from_failure_sm = sm
    return sm




def get_execute_sm():
    # This state machine is large, so we will only instantiate once and point references to one copy
    global shared_execute_sm
    if shared_execute_sm:
        return shared_execute_sm
    shared_execute_sm = StateMachine(outcomes=["succeeded", "aborted"], input_keys=["goal"],
                                     output_keys=["msg_for_operator"])
    with shared_execute_sm:
        control_flow.inject_userdata_auto("_SET_DEFAULT_MSG_FOR_OPERATOR", "msg_for_operator", "")
        repeat_state = control_flow.RepeatN(2)
        StateMachine.add_auto("RESET_REPEAT", control_flow.ResetRepeat(repeat_state), ["succeeded"])
        StateMachine.add("EXECUTE_GOAL", states.ExecuteGoal(),
                         transitions={"preempted": "RECOVERY_REPEAT_GATE",
                                      "aborted": "RECOVERY_REPEAT_GATE"})
        StateMachine.add("EXECUTE_RECOVERY_GOAL", states.ExecuteGoal(),
                         transitions={"preempted": "RECOVERY_REPEAT_GATE",
                                      "aborted": "RECOVERY_REPEAT_GATE"})
        StateMachine.add("RECOVERY_REPEAT_GATE", repeat_state,
                         transitions={"repeat": "RECOVER", "done": "aborted"})
        StateMachine.add("RECOVER", get_shared_recover_from_failure_sm(),
                         transitions={'succeeded': "EXECUTE_RECOVERY_GOAL"})
    return shared_execute_sm



