from smach import State
from knowledge_representation import PyAttributeList
import random

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


class GetRandomDoor(State):
    def __init__(self, ltmc):
        State.__init__(self, output_keys=["door"], outcomes=['succeeded', 'aborted'])
        self.ltmc = ltmc

    def execute(self, userdata):
        door = self.ltmc.get_concept("door")
        entities = door.get_instances()
        if entities:
            name_attrs = random.choice(entities).get_attributes("name")
            if not name_attrs:
                return 'failed'
            userdata["door"] = name_attrs[0].get_string_value()
            return 'succeeded'

        return 'aborted'

class GetRandomVisitableLocation(State):
    def __init__(self, ltmc):
        State.__init__(self, output_keys=["location"], outcomes=['succeeded', 'aborted'])
        self.ltmc = ltmc

    def execute(self, userdata):
        door = self.ltmc.get_concept("visitable")
        entities = door.get_instances()
        if entities:
            name_attrs = random.choice(entities).get_attributes("name")
            if not name_attrs:
                return 'failed'
            userdata["location"] = name_attrs[0].get_string_value()
            print (name_attrs[0].get_string_value())
            return 'succeeded'

        return 'aborted'

class GetRandomBwiLocation(State):
    def __init__(self, ltmc):
        State.__init__(self, output_keys=["location"], outcomes=['succeeded', 'aborted'])
        self.ltmc = ltmc

    def execute(self, userdata):
        lab_concept = self.ltmc.get_concept("lab")
        bwi_id = lab_concept.get_instance_named("bwi").entity_id

        def instances_of(concept):
            return """SELECT entity_id FROM instance_of
                WHERE concept_name = '{}'
            """.format(concept)

        bwi_people = """SELECT attribute_value FROM entity_attributes_id
            WHERE attribute_name = 'has'
            AND entity_id = {}
            AND attribute_value IN ({})
        """.format(bwi_id, instances_of("person"))

        locations_owned_by_bwi_people = """SELECT attribute_value FROM entity_attributes_id
            WHERE attribute_name = 'has'
            AND entity_id IN ({})
            AND attribute_value IN ({})
        """.format(bwi_people, instances_of("location"))

        rooms_owned_by_bwi_people = """SELECT attribute_value FROM entity_attributes_id
            WHERE attribute_name = 'has'
            AND entity_id IN ({})
            AND attribute_value IN ({})
        """.format(bwi_people, instances_of("room"))

        rooms_owned_by_bwi = """SELECT attribute_value FROM entity_attributes_id
            WHERE attribute_name = 'has'
            AND entity_id IN ({})
            AND attribute_value IN ({})
        """.format(bwi_id, instances_of("room"))

        # Gets things that bwi rooms have
        doors_owned_by_bwi_rooms = """SELECT attribute_value FROM entity_attributes_id
              WHERE attribute_name = 'has'
              AND entity_id IN (({}) UNION ({}))
              AND attribute_value IN ({})
        """.format(rooms_owned_by_bwi, rooms_owned_by_bwi_people, instances_of("door"))

        # Gets names of doors that BWI has
        query = """SELECT * FROM entity_attributes_str
              WHERE attribute_name = 'name' 
              AND entity_id IN (({}) UNION ({}))
        """.format(locations_owned_by_bwi_people, doors_owned_by_bwi_rooms)
        attributes = PyAttributeList()

        self.ltmc.select_query_string(query, attributes)

        locations = [attribute.get_string_value() for attribute in attributes]

        # These doors are inside the lab spaces. We don't want the robot trying to reach them.
        dont_go = ["d3_414a3", "d3_414b3"]
        for door in dont_go:
            if door in locations:
                locations.remove(door)

        if locations:
            name_attrs = random.choice(locations)
            if not name_attrs:
                return 'failed'

            print(name_attrs)
            userdata["location"] = name_attrs
            return 'succeeded'

        return 'aborted'
