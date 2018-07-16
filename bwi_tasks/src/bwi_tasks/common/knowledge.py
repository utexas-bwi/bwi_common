from smach import State


class ClearTemporaryKnowledge(State):
    def __init__(self, ltmc):
        State.__init__(self, outcomes=['succeeded'])
        self.ltmc = ltmc

    def execute(self, userdata):
        self.ltmc.remove_concept_references("operator")
        self.ltmc.remove_children_of_entity(self.ltmc.get_concept("hypothetical"))
        self.ltmc.remove_concept_references("scanned")
        self.ltmc.remove_children_of_entity(self.ltmc.get_concept("sensed"))

        return 'succeeded'


class SetHolding(State):
    def __init__(self, ltmc):
        State.__init__(self, outcomes=['succeeded'], input_keys=["object_id"])
        self.ltmc = ltmc

    def execute(self, userdata):
        self.ltmc.add_entity_attribute(1, "is_holding", userdata.object_id)
        self.ltmc.remove_entity_attribute(userdata.object_id, "is_placed")

        return 'succeeded'


class CheckHandEmpty(State):
    def __init__(self, ltmc):
        State.__init__(self, outcomes=['succeeded', 'aborted'])
        self.ltmc = ltmc

    def execute(self, userdata):
        entities = self.ltmc.get_entities_with_attribute_of_value("is_a", self.ltmc.get_concept("empty_handed"))
        if entities:
            return 'succeeded'

        return 'aborted'


