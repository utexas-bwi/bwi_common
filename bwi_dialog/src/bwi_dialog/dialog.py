from knowledge_representation import xml_kbase
from smach import State, StateMachine

from villa_top_level.common import util, states
from bwi_tasks import control_flow
from villa_hsrb.speech import Say
from bwi_speech.speech import TranscribeSpeech


class CheckIfAffirmative(State):
    def __init__(self):
        State.__init__(self, outcomes=['affirmative', 'negative', 'indeterminate'], input_keys=["transcript"])

    def execute(self, userdata):
        transcript = userdata.transcript
        print("confimation: {}".format(transcript))
        if any(word in transcript for word in ["no", "nope", "never", "not correct", "incorrect", "wrong"]):
            return 'negative'
        elif any(word in transcript for word in ["yes", "yeah", "yup", "correct", "right"]):
            return 'affirmative'
        return 'indeterminate'


def obtain_confirmation():
    obtain_confirmation_wrist_sm = StateMachine(outcomes=['succeeded', 'aborted'])
    with obtain_confirmation_wrist_sm:
        if util.in_sim():
            control_flow.transition_to("BYPASS", "succeeded")
        else:
            StateMachine.add_auto('ASK_FOR_TAP', Say("Can you shake my wrist to confirm?"), ["succeeded"])
            StateMachine.add("AWAIT_TAP", states.WaitForStart(),{"signalled": "succeeded", "not_signalled": "aborted"})
    obtain_confirmation_sm = StateMachine(outcomes=['confirmed', 'denied', 'aborted'])
    with obtain_confirmation_sm:
        if util.in_sim():
            control_flow.transition_to("BYPASS", "confirmed")
        else:
            StateMachine.add('TRANSCRIBE_SPEECH', TranscribeSpeech(5),
                             transitions={"succeeded": "CHECK_IF_AFFIRMATIVE", "aborted": "OBTAIN_CONFIRMATION_WRIST" })
	    StateMachine.add("OBTAIN_CONFIRMATION_WRIST", obtain_confirmation_wrist_sm, transitions={"succeeded": "confirmed"})
            StateMachine.add('CHECK_IF_AFFIRMATIVE', CheckIfAffirmative(),
                             transitions={"affirmative": "confirmed", "negative": "denied",
                                          "indeterminate": 'denied'})
    return obtain_confirmation_sm


class GetNameFromSpeech(State):
    def __init__(self, ltmc):
        State.__init__(self, outcomes=['succeeded', 'aborted'], input_keys=["transcript"],
                       output_keys=["information_retrieved"])
        self.ltmc = ltmc
        self.xml_kbase = xml_kbase.get_default_xml_kbase()

    def execute(self, userdata):
        candidates = self.xml_kbase.name_parser.all_names()

        transcript = userdata.transcript.strip().lower()
        print("user response: {}".format(transcript))
        words = transcript.split(" ")
        word_set = set(words)
        candidate_set = set([candidate.lower() for candidate in candidates])
        candidate_set.intersection_update(word_set)
        if len(candidate_set) > 0:
            userdata.information_retrieved = list(candidate_set)[0]
            return "succeeded"
        return "aborted"
