import xml.etree.ElementTree as ET
from utils import normalize


class LocationParser(object):

    def __init__(self, location_xml_file):
        self.location_file = location_xml_file
        self.tree = ET.parse(self.location_file)

    """query Locations xml file to get room for location_obj
        will return list of multiple rooms if object is in multiple locations
        TODO: Figure what to do if can't find it, currently returns []
        This works for where is located the ? and In which room is the ? questions
    """
    def where_is_located(self, obj_name):
        location_list = []
        #get root (rooms in this case)
        root = self.tree.getroot()
        #for each room in rooms check if it has child with name equal to location_obj
        for room in root.findall("./room"):
            #print room.tag, room.attrib
            for thing in room:
                if normalize(thing.attrib['name']) == obj_name:
                    location_list.append(room.attrib['name'])
        return location_list
        
    """query Locations.xml file to get number of doors of room
        Note that this needs to be manually entered into the Locations.xml file given
        Returns number of doors or None is room not found
    """
    def how_many_doors(self, room_name):
        #get root (rooms in this case)
        root = self.tree.getroot()
        #find room that matches room_name and get num doors
        for room in root.findall("./room"):
            if normalize(room.attrib['name']) == room_name:
                return room.attrib['doors']
        return None
        
    def how_many_location_in_room(self,location_name, room_name):
        object_cnt = 0
        found_room = False
        #get root (rooms in this case)
        root = self.tree.getroot()
        #for each room in rooms check if it has child with name equal to location_obj
        for room in root.findall("./room"):
            if normalize(room.attrib['name']) == room_name:
                found_room = True
                for thing in room:
                    if normalize(thing.attrib['name']) == location_name:
                        object_cnt += 1
        if found_room:
            return object_cnt
        else:
            return None


    '''return dictionary of room:location_list'''
    def get_room_locations(self):
        room_locations = {}
        tree = ET.parse(self.location_file)
        #get root (rooms in this case)
        root = tree.getroot()
        #for each room in rooms check if it has child with name equal to location_obj
        for room in root.findall("./room"):
            location_list = []
            room_name = room.attrib['name'].lower()
            for thing in room:
                location_list.append(thing.attrib['name'].lower())
            room_locations[room_name] = location_list
        return room_locations

    def get_all_locations(self):
        locations = []    
        #parse xml
        root = self.tree.getroot()
        #for each room in rooms check if it has child with name equal to location_obj
        for location in root.findall("./room/location"):
            locations.append(location.attrib['name'])
        return locations

    def get_all_placements(self):
        all_placements = []
        root = self.tree.getroot()
        for room in root.findall("./room"):
            for location in room:
                if 'isPlacement' in location.attrib:
                    if location.attrib['isPlacement'] == "true":
                        all_placements.append(location.attrib['name'])
        return all_placements

    def parse_beacons(self):
        all_beacons = []
        root = self.tree.getroot()
        for room in root.findall("./room"):
            for location in room:
                if 'isBeacon' in location.attrib:
                    if location.attrib['isBeacon'] == "true":
                        all_beacons.append(location.attrib['name'])
        return all_beacons

    def get_all_rooms(self):
        all_rooms = []
        root = self.tree.getroot()
        for room in root.findall("./room"):
            all_rooms.append(room.attrib['name'])
        return all_rooms


def main():
    location_parser = LocationParser("../../resources/Locations_mod.xml")
    #example usage
    print (location_parser.where_is_located("living shelf"))
    print (location_parser.how_many_doors("corridor"))
    print ("how many desk in bathroom?", location_parser.how_many_location_in_room("desk","bathroom"))
    print (location_parser.get_all_locations())
    
if __name__ == "__main__":
    main()

