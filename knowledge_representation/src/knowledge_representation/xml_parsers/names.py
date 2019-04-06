import xml.etree.ElementTree as ET


class NameParser:
    def __init__(self, names_xml_file):
        self.tree = ET.parse(names_xml_file)

    def all_names(self):
        all_names = []
        root = self.tree.getroot()
        for name in root.findall("./name"):
            all_names.append(name.text)
        return all_names