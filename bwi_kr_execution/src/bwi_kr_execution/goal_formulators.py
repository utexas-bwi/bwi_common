from plan_execution.helpers import to_asp_rule_body, to_asp_rule_head
from smach_ros import SimpleActionState
from bwi_tasks.common.task_machine import FormulateGoal



class GoTo(FormulateGoal):
    def __init__(self, default_location=None):
        FormulateGoal.__init__(self, ['location'])

    def formulate_goal(self, userdata, goal):
        if self.default_location is None:
            to_go = userdata.location
        else:
            to_go = self.default_location
        goal_rule = to_asp_rule_body("not is_near", [1, to_go])
        goal.aspGoal = [goal_rule]


class GoToLocationName(FormulateGoal):
    def __init__(self, default_location=None):
        FormulateGoal.__init__(self, ['location'])
        self.default_location = default_location

    def formulate_goal(self, userdata, goal):
        if self.default_location is None:
            to_go = userdata.location
        else:
            to_go = self.default_location

        goal_rule = to_asp_rule_body("not is_near_name", [1, to_go])
        goal.aspGoal = [goal_rule]


class PickUpObject(FormulateGoal):
    def __init__(self, ltmc):
        FormulateGoal.__init__(self, ['location', 'object'])
        self.ltmc = ltmc

    def formulate_goal(self, userdata, goal):
        specific_object_con = self.ltmc.get_concept(userdata.object)
        object_id = self.ltmc.add_entity()
        self.ltmc.add_entity_attribute(object_id, "is_a", specific_object_con)
        self.ltmc.add_entity_attribute(object_id, "is_a", self.ltmc.get_concept("hypothetical"))

        location_rule = plan_exec_helpers.to_asp_rule_head("can_be_placed_concept", [object_id, userdata.location])
        goal_rule = plan_exec_helpers.to_asp_rule_body("not is_holding_concept", [1, userdata.object])
        goal.aspGoal = [location_rule, goal_rule]


class LocateObject(FormulateGoal):
    def __init__(self, ltmc):
        FormulateGoal.__init__(self, ['location', 'object'])
        self.ltmc = ltmc

    def formulate_goal(self, userdata, goal):
        specific_object_con = self.ltmc.get_concept(userdata.object)
        object_id = self.ltmc.add_entity()
        self.ltmc.add_entity_attribute(object_id, "is_a", specific_object_con)
        self.ltmc.add_entity_attribute(object_id, "is_a", self.ltmc.get_concept("hypothetical"))

        location_rule = plan_exec_helpers.to_asp_rule_head("can_be_placed_concept", [object_id, userdata.location])
        goal_rule = plan_exec_helpers.to_asp_rule_body("not is_in_concept", [userdata.object, userdata.location])
        goal.aspGoal = [location_rule, goal_rule]


class ScanPlacement(FormulateGoal):
    def __init__(self):
        FormulateGoal.__init__(self, ['object', 'location'])

    def formulate_goal(self, userdata, goal):
        goal_rule = plan_exec_helpers.to_asp_rule_body("not scanned_concept", [userdata.location])
        goal.aspGoal = [goal_rule]


class FindPerson(FormulateGoal):
    def __init__(self, ltmc):
        FormulateGoal.__init__(self, ['location', 'name'])
        self.ltmc = ltmc

    def formulate_goal(self, userdata, goal):
        person_con = self.ltmc.get_concept("person")
        person = self.ltmc.add_entity()
        self.ltmc.add_entity_attribute(person, "is_a", person_con)
        self.ltmc.add_entity_attribute(person, "is_a", self.ltmc.get_concept("hypothetical"))

        if userdata.name is not None:
            self.ltmc.add_entity_attribute(person, "person_name", userdata.name)

        location_rule = plan_exec_helpers.to_asp_rule_head("can_be_located_concept", [person, userdata.location])
        goal_facing_rule = plan_exec_helpers.to_asp_rule_body("not is_facing", [1, person])
        goal.aspGoal = [location_rule, goal_facing_rule]


class MoveObject(FormulateGoal):
    def __init__(self, ltmc):
        FormulateGoal.__init__(self, ['object', 'origin', 'destination'])
        self.ltmc = ltmc

    def formulate_goal(self, userdata, goal):
        specific_object_con = self.ltmc.get_concept(userdata.object)
        object_id = self.ltmc.add_entity()
        self.ltmc.add_entity_attribute(object_id, "is_a", specific_object_con)
        self.ltmc.add_entity_attribute(object_id, "is_a", self.ltmc.get_concept("hypothetical"))

        if userdata.origin is not None:
            location_rule = plan_exec_helpers.to_asp_rule_head("can_be_placed_concept", [object_id, userdata.origin])
            goal.aspGoal = [location_rule]

        goal_rule = plan_exec_helpers.to_asp_rule_body("not is_placed_concept", [userdata.object, userdata.destination])
        goal.aspGoal.append(goal_rule)


class DeliverToPerson(FormulateGoal):
    def __init__(self, ltmc):
        FormulateGoal.__init__(self, ['object', 'location', 'name', 'destination'])
        self.ltmc = ltmc

    def formulate_goal(self, userdata, goal):
        specific_object_con = self.ltmc.get_concept(userdata.object)
        object_id = self.ltmc.add_entity()
        self.ltmc.add_entity_attribute(object_id, "is_a", specific_object_con)
        self.ltmc.add_entity_attribute(object_id, "is_a", self.ltmc.get_concept("hypothetical"))

        person_con = self.ltmc.get_concept("person")
        person = self.ltmc.add_entity()
        self.ltmc.add_entity_attribute(person, "is_a", person_con)
        self.ltmc.add_entity_attribute(person, "is_a", self.ltmc.get_concept("hypothetical"))
        self.ltmc.add_entity_attribute(person, "person_name", userdata.name)

        if userdata.location is not None:
            location_rule = plan_exec_helpers.to_asp_rule_head("can_be_placed_concept", [object_id, userdata.location])
            goal.aspGoal = [location_rule]

        person_location_rule = plan_exec_helpers.to_asp_rule_head("can_be_located_concept",
                                                                  [person, userdata.destination])
        goal.aspGoal.append(person_location_rule)

        goal_rule = plan_exec_helpers.to_asp_rule_body("not is_delivered_concept", [userdata.object, person])
        goal.aspGoal.append(goal_rule)


class BringToOperator(FormulateGoal):
    def __init__(self, ltmc):
        FormulateGoal.__init__(self, ['object', 'location', 'operator_id'])
        self.ltmc = ltmc

    def formulate_goal(self, userdata, goal):
        specific_object_con = self.ltmc.get_concept(userdata.object)
        object_id = self.ltmc.add_entity()
        self.ltmc.add_entity_attribute(object_id, "is_a", specific_object_con)
        self.ltmc.add_entity_attribute(object_id, "is_a", self.ltmc.get_concept("hypothetical"))

        person = userdata.operator_id

        if userdata.location is not None:
            location_rule = plan_exec_helpers.to_asp_rule_head("can_be_placed_concept", [object_id, userdata.location])
            goal.aspGoal = [location_rule]

        goal_rule = plan_exec_helpers.to_asp_rule_body("not is_delivered_concept", [userdata.object, person])

        goal.aspGoal.append(goal_rule)
