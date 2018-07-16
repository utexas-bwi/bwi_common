import copy

import rospy
import smach
from smach import State, StateMachine

from bwi_tasks.common import states


def break_dict_out(name, to, input_key, output_keys):
    class BreakDataOut(State):
        def __init__(self, ):
            State.__init__(self, outcomes=['succeeded'], input_keys=[input_key], output_keys=output_keys)
            self.output_keys = output_keys
            self.input_key = input_key

        def execute(self, userdata):
            for key in self.output_keys:
                userdata[key] = userdata[self.input_key][key]
            return 'succeeded'

    StateMachine.add(name, BreakDataOut(), transitions={'succeeded': to})


def break_dict_out_auto(name, input_key, output_keys):
    class BreakDataOut(State):
        def __init__(self, ):
            State.__init__(self, outcomes=['succeeded'], input_keys=[input_key], output_keys=output_keys)
            self.output_keys = output_keys
            self.input_key = input_key

        def execute(self, userdata):
            for key in self.output_keys:
                userdata[key] = userdata[self.input_key][key]
            return 'succeeded'

    StateMachine.add_auto(name, BreakDataOut(), ["succeeded"])


def inject_userdata(name, to, output_key, value):
    class InjectUserdata(State):
        def __init__(self, ):
            State.__init__(self, outcomes=['succeeded'], output_keys=[output_key])
            self.output_key = output_key

        def execute(self, userdata):
            userdata[output_key] = value
            return 'succeeded'

    StateMachine.add(name, InjectUserdata(), transitions={'succeeded': to})


def inject_userdata_auto(name, output_key, value):
    class InjectUserdata(State):
        def __init__(self, ):
            State.__init__(self, outcomes=['succeeded'], output_keys=[output_key])
            self.output_key = output_key

        def execute(self, userdata):
            userdata[output_key] = value
            return 'succeeded'

    StateMachine.add_auto(name, InjectUserdata(), ["succeeded"])


def input_to_output(input_key, output_key):
    class InputToOutput(smach.State):
        def __init__(self):
            smach.State.__init__(self, outcomes=['succeeded'], input_keys=[input_key], output_keys=[output_key])

        def execute(self, userdata):
            userdata[output_key] = userdata[input_key]
            return 'succeeded'

    return InputToOutput()


def chain_states(*args):
    class ChainedStates(smach.State):
        def __init__(self):
            input_keys = []
            output_keys = []
            for state in args:
                input_keys += state._input_keys
                output_keys += state._output_keys

            smach.State.__init__(self, outcomes=['succeeded'], input_keys=input_keys, output_keys=output_keys)
            self.states = args

        def execute(self, userdata):
            for state in self.states:
                state.execute(userdata)
            return 'succeeded'

    return ChainedStates()


class Sleep(smach.State):
    def __init__(self, seconds):
        self.seconds = seconds
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.sleep(self.seconds)
        return 'succeeded'


def retry_n(state, n):
    class RetryN(State):
        def __init__(self):
            State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=list(state._input_keys), output_keys=list(state._output_keys))
            self.repeat_state = state
            self.tries = 0

        def execute(self, userdata):
            status = 'failed'
            while (status == 'failed'
                    and self.tries < n):
                status = self.repeat_state.execute(userdata)
                self.tries += 1

            return status

    return RetryN()


class RepeatN(State):
    def __init__(self, n):
        State.__init__(self, outcomes=['repeat', 'done'], output_keys=["index"])
        self.counter = 0
        self.num_repetitions = n

    def execute(self, userdata):
        self.counter = self.counter + 1
        userdata["index"] = self.counter
        if self.counter >= self.num_repetitions:
            self.counter = 0
            userdata["index"] = self.counter
            return 'done'
        else:
            return 'repeat'


class ResetRepeat(State):
    def __init__(self, repeat_state):
        State.__init__(self, outcomes=["succeeded"])
        self.repeat_state = repeat_state
        assert isinstance(repeat_state, RepeatN)

    def execute(self, ud):
        self.repeat_state.counter = 0
        return "succeeded"


class TransitionBasedOnUserdata(State):
    def __init__(self, decide_based_on, forward_to):
        targets_copy = copy.deepcopy(forward_to)
        forwarding_targets = set(targets_copy)
        forwarding_targets.add("aborted")
        State.__init__(self, outcomes=list(forwarding_targets), input_keys=[decide_based_on])
        self.forward_to = targets_copy

    def execute(self, userdata):
        target = self.extract_from_userdata(userdata)
        # If we have the target registered, head towards it.
        if target in self.forward_to:
            return target
        return 'aborted'

    def extract_from_userdata(self, userdata):
        raise NotImplementedError


def remap_auto(name, from_key, to_key):
    class Remap(State):
        def __init__(self):
            State.__init__(self, outcomes=["succeeded"], input_keys=[from_key], output_keys=[to_key])

        def execute(self, ud):
            ud[to_key] = ud[from_key]
            return "succeeded"

    smach.StateMachine.add_auto(name, Remap(), ["succeeded"])


def remap(name, from_key, to_key, transitions={}):
    class Remap(State):
        def __init__(self):
            State.__init__(self, outcomes=["succeeded"], input_keys=[from_key], output_keys=[to_key])

        def execute(self, ud):
            ud[to_key] = ud[from_key]
            return "succeeded"

    smach.StateMachine.add(name, Remap(), transitions=transitions)


def select_nth_auto(name, from_key, i, to_key):
    class SelectNth(State):
        def __init__(self):
            State.__init__(self, outcomes=["succeeded"], input_keys=[from_key], output_keys=[to_key])

        def execute(self, ud):
            ud[to_key] = ud[from_key][i]
            return "succeeded"

    StateMachine.add_auto(name, SelectNth(), ["succeeded"])


def select_ith_auto(name, from_key, to_key):
    class SelectIth(State):
        def __init__(self):
            State.__init__(self, outcomes=["succeeded"], input_keys=[from_key, "index"], output_keys=[to_key])

        def execute(self, ud):
            ud[to_key] = ud[from_key][ud["index"]]
            return "succeeded"

    StateMachine.add_auto(name, SelectIth(), ["succeeded"])


def transition_to(name, target_state):
    StateMachine.add(name, states.NoOp(), transitions={"succeeded": target_state})


def dequeue(name, from_key, to_key, transitions={}):
    class Dequeue(State):
        def __init__(self):
            State.__init__(self, outcomes=["succeeded", "aborted"], input_keys=[from_key],
                           output_keys=[from_key, to_key])

        def execute(self, ud):
            queue = ud[from_key]
            if len(queue) > 0:
                ud[to_key] = queue[0]
                ud[from_key] = ud[from_key][1:]
                return "succeeded"
            return "aborted"

    StateMachine.add(name, Dequeue(), transitions=transitions)


def enqueue(name, from_key, to_key, transitions={}):
    class Enqueue(State):
        def __init__(self):
            State.__init__(self, outcomes=["succeeded", "aborted"], input_keys=[from_key, to_key], output_keys=[to_key])

        def execute(self, ud):
            ud[to_key] = " ".join([ud[to_key], ud[from_key]])
            return "succeeded"

    StateMachine.add(name, Enqueue(), transitions=transitions)
