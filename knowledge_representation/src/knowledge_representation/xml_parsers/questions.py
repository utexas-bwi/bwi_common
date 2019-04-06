import xml.etree.ElementTree as ET


class QuestionParser:
    def __init__(self, questions_xml_file):
        self.tree = ET.parse(questions_xml_file)

    def get_question_answer_dict(self):
        qa_dictionary = {}

        root = self.tree.getroot()
        for question in root.findall("./question"):
            question_text = ""
            answer_text = ""
            for child in question:
                if child.tag == "q":
                    question_text = child.text
                elif child.tag == "a":
                    answer_text = child.text
            qa_dictionary[question_text] = answer_text
        return qa_dictionary

    def get_all_questions(self):
        return list(self.get_question_answer_dict().keys())