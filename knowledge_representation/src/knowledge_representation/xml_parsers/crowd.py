import xml.etree.ElementTree as ET
import os
from rospkg.rospack import RosPack

class CrowdParser():
    def __init__(self, crowd_xml_file):
        self.object_file = crowd_xml_file
        self.tree = ET.parse(crowd_xml_file)
        
    #XML query for "Where can I find a ({object} | {category})?"
    #returns the defaultlocation or None if item is not an object or category
    def get_age_and_gender(self, pose, person):
        #loop through the poses
            #find match position i to pose
            #if person == "boy":
                #if genders[i] == 'M' and ages[i]  == '0-2' or '4-6' or '8-12':
                    #return True
            #elif person == "girl":
                #if genders[i] == 'F' and ages[i]  == '0-2' or '4-6' or '8-12':
                    #return True
            #elif person == "man" or "male":
                #if genders[i] == 'F' and ages[i]  == '15-20' or '25-32' or '38-43' or '48-53' or '60-100':
                    #return True
            #elif person == "woman" or "female":
                #if genders[i] == 'F' and ages[i]  == '15-20' or '25-32' or '38-43' or '48-53' or '60-100':
                    #return True
        return False

    '''return number of people in crowd'''        
    def count_crowd(self):
        crowd_cnt = 0 
        root = self.tree.getroot()
        for item in root:
            if item.tag == "gender":
                crowd_cnt += 1
        #check if item is a category, if not search the objects in each category
#        for cat in root.findall("./category"):
#            if cat.attrib['name'].lower() == item.lower():
#                return cat.attrib['defaultLocation']
#            else:
#                for obj in cat:
#                    if obj.attrib['name'].lower() == item.lower():
#                        return cat.attrib['defaultLocation']
        return crowd_cnt
            
    def count_males(self):
        male_cnt = 0 
        root = self.tree.getroot()
        for item in root:
            if item.tag == "gender" and item.attrib['gender'] == "M":
                male_cnt += 1
        return male_cnt
        
    def count_females(self):
        female_cnt = 0 
        root = self.tree.getroot()
        for item in root:
            if item.tag == "gender" and item.attrib['gender'] == "F":
                female_cnt += 1
        return female_cnt
        
    def count_gesture(self, gesture_name):
        cnt = 0
        root = self.tree.getroot()
        for item in root:
            if item.tag == "activity" and item.attrib['activity'] == gesture_name:
                cnt += 1
        return cnt
        
        
    def count_waving(self):
        return self.count_gesture("WAVING")
        
        
    def count_raising_right_arm(self):
        return self.count_gesture("RIGHT_HAND_UP")
    
    def count_raising_left_arm(self):
        return self.count_gesture("LEFT_HAND_UP")

if __name__=="__main__":
    rospack = RosPack()
    crowd_xml_filename = os.path.abspath(os.path.join(rospack.get_path('spr_qa'), 'resources/people_info.xml'))
    crowd_parser = CrowdParser(crowd_xml_filename)
    print crowd_parser.count_crowd()
    print crowd_parser.count_females()
    print crowd_parser.count_males()
    print crowd_parser.count_waving()
    print crowd_parser.count_raising_right_arm()
    print crowd_parser.count_raising_left_arm()