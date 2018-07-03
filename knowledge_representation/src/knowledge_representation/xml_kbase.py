from knowledge_representation.xml_parsers.locations import LocationParser
from knowledge_representation.xml_parsers.objects import ObjectParser
from knowledge_representation.xml_parsers.questions import QuestionParser
from knowledge_representation.xml_parsers.names import NameParser
from knowledge_representation.xml_parsers.crowd import CrowdParser
import random
import os
from rospkg.rospack import RosPack


def get_default_xml_files():
    rospack = RosPack()
    xml_files_location = os.path.abspath(os.path.join(rospack.get_path('spr_qa'), '../GPSRCmdGen/CommonFiles'))
    assert (os.path.isdir(xml_files_location))
    location_xml_filename = os.path.join(xml_files_location, 'Locations.xml')
    object_xml_filename = os.path.join(xml_files_location, 'Objects.xml')
    question_xml_filename = os.path.join(xml_files_location, 'Questions.xml')
    name_xml_filename = os.path.join(xml_files_location, 'Names.xml')
    
    crowd_xml_filename = os.path.abspath(os.path.join(rospack.get_path('spr_qa'), 'resources/people_info.xml'))
    
    assert (os.path.isfile(location_xml_filename) and os.path.isfile(object_xml_filename) and os.path.isfile(question_xml_filename) and os.path.isfile(name_xml_filename))
    return location_xml_filename, object_xml_filename, question_xml_filename, name_xml_filename, crowd_xml_filename


def get_default_xml_kbase():
    return XMLKnowledgeBase(*get_default_xml_files())


class XMLKnowledgeBase(object):
    def __init__(self, location_xml_file, object_xml_file, question_xml_file, name_xml_filename, crowd_xml_filename):
        self.location_parser = LocationParser(location_xml_file)
        self.object_parser = ObjectParser(object_xml_file)
        self.question_parser = QuestionParser(question_xml_file)
        self.name_parser = NameParser(name_xml_filename)
        self.crowd_parser = CrowdParser(crowd_xml_filename)
    
    def query(self, logic, argument_tuple):
        if logic == "find_location(x)":
            return self.location_parser.where_is_located(argument_tuple[0])
        elif logic == "count(x,doors)":
            return self.location_parser.how_many_doors(argument_tuple[0])
        elif logic == "count_crowd(x)":
            person_type = argument_tuple[0]
            total_num_people = self.crowd_parser.count_crowd()
            num_women = self.crowd_parser.count_females()
            num_men = self.crowd_parser.count_males()
            #hard code in no children
            if person_type == "adults":
                return total_num_people
            if person_type == "children" or person_type == "boys" or person_type == "girls" or person_type == "elders":
                return 0
            elif person_type == "females" or person_type == "women":
                #TODO: don't hard code this
                return num_women
            else: 
                #TODO: need to connect crowd questions, for now just guess zero, one,or two.
                #return random.randint(0,2)
                return num_men
        elif logic == "count_posture(x)":
            posture = argument_tuple[0]
            if posture == "waving":
                #TODO connect waving from xml
                return self.crowd_parser.count_waving()
            elif posture == "raising their left arm":
                return self.crowd_parser.count_raising_left_arm()
            elif posture == "raising their right arm":
                return self.crowd_parser.count_raising_right_arm()
            else:
                #just guess
                return random.randint(0,3)
        elif logic == "count_wearing(x)":
            #TODO: need to connect crowd questions, for now just guess zero, one,or two.
            return random.randint(0,3)
        elif logic == "count(x,y)":
            #TODO what do we do about crowd questions?
            #Ambiguous so need to check with multiple queries and see what works
            object_query = self.object_parser.how_many_objects_in_location(argument_tuple[0], argument_tuple[1])
            location_query = self.location_parser.how_many_location_in_room(argument_tuple[0], argument_tuple[1])
            #print object_query, location_query
            if object_query is None and location_query is None:
                return None
            elif object_query is None:
                return location_query
            else:
                return object_query
        elif logic == "which_person(x,y,z)":
            #guess for now.
            if (random.random() < 0.5):
                return argument_tuple[1]
            else:
                return argument_tuple[2]
        elif logic=="is_person(x,y)":
            #hard code zero children
            person_type = argument_tuple[1]
            if person_type == "boy" or person_type == "girl":
                return 0
            #TODO: make this based on XML
            if random.random() < 0.5:
                return True
            else:
                return False
            
        elif logic == "get_object_location(x)":
            return self.object_parser.get_default_location(argument_tuple[0])
        elif logic == "count_category(x)":
            return self.object_parser.get_num_in_category(argument_tuple[0])
        elif logic == "list_objects_in(x)":
            return self.object_parser.list_objects_in_location(argument_tuple[0])
        elif logic == "find_category(x)":
            return self.object_parser.find_object_category(argument_tuple[0])
        elif logic == "same_category(x,y)":
            return self.object_parser.are_objects_same_category(argument_tuple[0], argument_tuple[1])
        elif logic == "object_color(x)":
            return self.object_parser.get_object_color(argument_tuple[0])
        elif logic == "find_object(x,y)":
            if argument_tuple[0] == "heaviest" or argument_tuple[0] == "lightest" or argument_tuple[0] == "smallest" or argument_tuple[0] == "biggest":
                return self.object_parser.find_object(argument_tuple[0], argument_tuple[1])
            else:
                return self.object_parser.find_object(argument_tuple[1], argument_tuple[0])
        elif logic == "which_object(x,y,z)":
            return self.object_parser.which_object(argument_tuple[0], argument_tuple[1], argument_tuple[2])
        elif logic == "find_location(x) or get_object_location(x)":
            location = self.location_parser.where_is_located(argument_tuple[0])
            obj = self.object_parser.get_default_location(argument_tuple[0])
            if location == []:
                return ("obj",obj)
            return ("loc", location)
        elif logic == "fetch_object(x,y)":
            return (argument_tuple[0], argument_tuple[1])
        #elif logic == "saw_person(x,y)":
        #    return self.person_parser.get_age_and_gender(argument_tuple[0], argument_tuple[1])
        #elif logic == "count(crowd, x)":
        #    return self.person_parser.count_crowd(arguments_tuple[0])
