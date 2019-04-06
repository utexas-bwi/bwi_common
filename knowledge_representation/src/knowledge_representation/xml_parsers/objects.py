import xml.etree.ElementTree as ET


class ObjectParser:
    def __init__(self, object_xml_file):
        self.object_file = object_xml_file
        self.tree = ET.parse(object_xml_file)

    #XML query for "Where can I find a ({object} | {category})?"
    #returns the defaultlocation or None if item is not an object or category
    def get_default_location(self, item):
        #get root (rooms in this case)
        root = self.tree.getroot()
        #check if item is a category, if not search the objects in each category
        for cat in root.findall("./category"):
            if cat.attrib['name'].lower() == item.lower():
                return cat.attrib['defaultLocation']
            else:
                for obj in cat:
                    if obj.attrib['name'].lower() == item.lower():
                        return cat.attrib['defaultLocation']
        return None

    def all_objects(self):
        all_objects = []
        root = self.tree.getroot()
        for cat in root.findall("./category"):
            for obj in cat:
                all_objects.append(obj.attrib['name'])
        all_objects.sort(key=len)
        all_objects.reverse()
        return all_objects

    def all_categories(self):
        all_categories = []
        root = self.tree.getroot()
        for cat in root.findall("./category"):
            all_categories.append(cat.attrib['name'])
        return all_categories


    #XML query for "How many {category} there are?" questions
    #returns count or None is that category doesn't exist
    def get_num_in_category(self, category):
        count = 0
        #parse xml
        tree = ET.parse(self.object_file)
        #get root (rooms in this case)
        root = tree.getroot()
        for cat in root.findall("./category"):
            if cat.attrib['name'].lower() == category.lower():
                for obj in cat:
                    count += 1
                return count
        return None


    #XML query for "How many ({category} | objects) are in the {placement}?"
    def how_many_objects_in_location(self, category, location):
        #if category is "objects", then parse xml and find all categories with location as defaultLocation and add up all objects in these 
        count = 0
        found_location = False
        #parse xml
        tree = ET.parse(self.object_file)
        #get root (rooms in this case)
        root = tree.getroot()

        if category == "objects" or category == "object":
            for cat in root.findall("./category"):
                if cat.attrib['defaultLocation'].lower() == location.lower():
                    count += self.get_num_in_category(cat.attrib['name'])
                    found_location = True
        
        #otherwise find appropriate category and check if defaultLocation matches
        #return number of objects if matches, zero otherwise
        else:
            for cat in root.findall("./category"):
                if cat.attrib['defaultLocation'].lower() == location.lower():
                    found_location = True
                    if cat.attrib['name'].lower() == category.lower():
                        count = self.get_num_in_category(category)
        
        #return None if can't find placement in XML file
        if not found_location:
            return None
        else: 
            return count
         


    #XML query for "What objects are stored in the {placement}?" question_text
    def list_objects_in_location(self, location):
        obj_list = []
        found_location = False
        #parse xml
        tree = ET.parse(self.object_file)
        #get root (rooms in this case)
        root = tree.getroot()

        for cat in root.findall("./category"):
            if cat.attrib['defaultLocation'].lower() == location.lower():
                found_location = True
                for obj in cat:
                    obj_list.append(obj.attrib['name'])
        if found_location:
            return obj_list
        else:
            return None
        
        #return None if can't find placement in XML file
        if not found_location:
            return None 
        else: 
            return count

    '''return dictionary mapping categories to items'''
    def get_categories(self):
        #parse xml
        tree = ET.parse(self.object_file)
        #get root (rooms in this case)
        root = tree.getroot()

        categories = {}
        for cat in root.findall("./category"):
             cat_name = cat.attrib['name'].lower()
             cat_objs = []
             for obj in cat:
                 cat_objs.append(obj.attrib['name'].lower()) 
             categories[cat_name] = cat_objs
        return categories
        

    #XML query for question "To which category belong the {object}?"
    def find_object_category(self, object_name):
        #parse xml
        tree = ET.parse(self.object_file)
        #get root (rooms in this case)
        root = tree.getroot()

        for cat in root.findall("./category"):
            for obj in cat:
                if obj.attrib['name'].lower() == object_name.lower():
                    return cat.attrib['name']
        return None
        
    def get_object_color(self, object_name):
        tree = ET.parse(self.object_file)
        root = tree.getroot()
        for cat in root.findall("./category"):
            for obj in cat:
                if obj.attrib['name'].lower() == object_name.lower():
                    return obj.attrib['color']
        return None
    
    def find_object(self, adjective, cat_name):
        tree = ET.parse(self.object_file)
        root = tree.getroot()
        name = None
        if cat_name == "object":
                    
            for cat in root.findall("./category"):
                most_num = 0
                least_num = 100000
                for obj in cat:
                    if adjective == "heaviest":
                        if int(obj.attrib['weight']) > most_num:
                            most_num = int(obj.attrib['weight'])
                            name = obj.attrib['name']
                    if adjective == "lightest":
                        if int(obj.attrib['weight']) < least_num:
                            least_num = int(obj.attrib['weight'])
                            name = obj.attrib['name']
                    if adjective == "biggest":
                        if int(obj.attrib['size']) > most_num:
                            most_num = int(obj.attrib['size'])
                            name = obj.attrib['name']
                    if adjective == "smallest":
                        if int(obj.attrib['size']) < least_num:
                            least_num = int(obj.attrib['size'])
                            name = obj.attrib['name']
        else:
            for cat in root.findall("./category"):
                if cat_name in cat.attrib['name']:
                    most_num = 0
                    least_num = 100000
                    for obj in cat:
                        if adjective == "heaviest":
                            if int(obj.attrib['weight']) > most_num:
                                most_num = int(obj.attrib['weight'])
                                name = obj.attrib['name']
                        if adjective == "lightest":
                            if int(obj.attrib['weight']) < least_num:
                                least_num = int(obj.attrib['weight'])
                                name = obj.attrib['name']
                        if adjective == "biggest":
                            if int(obj.attrib['size']) > most_num:
                                most_num = int(obj.attrib['size'])
                                name = obj.attrib['name']
                        if adjective == "smallest":
                            if int(obj.attrib['size']) < least_num:
                                least_num = int(obj.attrib['size'])
                                name = obj.attrib['name']
        return name
                    
        
    def which_object(self, object_1, object_2, adjective):
        tree = ET.parse(self.object_file)
        root = tree.getroot()
        obj1_weight = 0
        obj2_weight = 0
        obj1_size = 0
        obj2_size = 0
        if adjective == "heavier":
            for cat in root.findall("./category"):
                for obj in cat:
                    if obj.attrib['name'].lower() == object_1.lower():
                        obj1_weight = obj.attrib['weight']
                    if obj.attrib['name'].lower() == object_2.lower():
                        obj2_weight = obj.attrib['weight']
            if obj1_weight > obj2_weight:
                return object_1
            elif obj2_weight > obj1_weight:
                return object_2
            else:
                return "equal"
        if adjective == "lighter":
            for cat in root.findall("./category"):
                for obj in cat:
                    if obj.attrib['name'].lower() == object_1.lower():
                        obj1_weight = obj.attrib['weight']
                    if obj.attrib['name'].lower() == object_2.lower():
                        obj2_weight = obj.attrib['weight']
            if obj1_weight < obj2_weight:
                return object_1
            elif obj2_weight < obj1_weight:
                return object_2
            else:
                return "equal"
        if adjective == "bigger":
            for cat in root.findall("./category"):
                for obj in cat:
                    if obj.attrib['name'] == object_1:
                        obj1_size = obj.attrib['size']
                    if obj.attrib['name'] == object_2:
                        obj2_size = obj.attrib['size']
            if obj1_size > obj2_size:
                return object_1
            elif obj2_size > obj1_size:
                return object_2
            else:
                return "equal"
        if adjective == "smaller":
            for cat in root.findall("./category"):
                for obj in cat:
                    if obj.attrib['name'].lower() == object_1.lower():
                        obj1_size = obj.attrib['size']
                    if obj.attrib['name'].lower() == object_2.lower():
                        obj2_size = obj.attrib['size']
            if obj1_size < obj2_size:
                return object_1
            elif obj2_size < obj1_size:
                return object_2
            else:
                return "equal"
            

    #XML query for "Do the {object 1} and {object 2} belong to the same category?"
    def are_objects_same_category(self, object_1, object_2):
        category_1 = self.find_object_category(object_1)
        category_2 = self.find_object_category(object_2)
        return category_1 == category_2
        

#What's the colour of the {kobject}?
#Which is the $adja ({category} | object)?
#Between the {object 1} and {object 2}, which one is $adjr?
def main():
    object_parser = ObjectParser("../../resources/Objects_mod.xml")
    print (object_parser.get_num_in_category("food"))
    print (object_parser.get_default_location("food"))
    print (object_parser.get_default_location("soap"))
    print (object_parser.how_many_objects_in_location("containers", "counter"))
    print (object_parser.list_objects_in_location("cupboard"))
    print (object_parser.find_object_category("sushi"))
    print (object_parser.are_objects_same_category("sushi", "tea"))
    print (object_parser.are_objects_same_category("plate", "mug"))

if __name__ == "__main__":
    main()
