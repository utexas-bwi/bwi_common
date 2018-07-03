#include "../include/knowledge_representation/LongTermMemoryConduit.h"
#include <unordered_map>
#include <iterator>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string.h>
#include <stdio.h>


int main(int argc, char *argv[]) {

    using namespace knowledge_rep;
    using namespace std;

    LongTermMemoryConduit interface("localhost", 33060, "root", "cyb3rPun!", "villa_krr");

// We want a map to store all of the {string : id} meta-concept pairs that will be added to the ltmc so that
// they can be accessed for later reference. For example: If multiple 'Container' objects are stored, they can all
// reference the same 'Container' object class.

    unordered_map<std::string, int> used_concepts;


    /*		Important Note:
     *
     * When you run this program's executable, make sure to either supply all csv files you intend
     * to populate the database with, or at the very least run it using both objects.csv and locations.csv.
     *
     * Also make sure that the items in the third column of locations.csv are the same as those in the second column of
     * objects.csv. This is important, so that they may be registered as the same thing in the ltmc.
     *
     * If necessary, capitalize or take off an 's' at the end*/

    for(int file = 1; file < argc; ++file) {

        std::string filename = argv[file];

        ifstream in;
        in.open(filename);

        std::istream &stream = in;

        std::string linestring;

        while(std::getline(stream, linestring)) {


            char * writable = new char[linestring.size() + 1];
            std::copy(linestring.begin(), linestring.end(), writable);
            writable[linestring.size()] = '\0';

            char *p = strtok(writable, ",");


            while(p != NULL) {

                std::string first(p);
                p = strtok(NULL, ",");
                if(p != NULL) {

                    std::string second(p);

                    int first_concept_id;
                    int second_concept_id;


                    if(char(*second.begin()) == ' ') second.erase(second.begin());

                    if(filename == "objects.csv") {
                        if(used_concepts.find(second) == used_concepts.end()) {


                            second_concept_id = interface.add_entity();
                            used_concepts[second] = second_concept_id;
                            interface.add_entity_attribute(second_concept_id, "concept", second);
                        }
                        else {

                            second_concept_id = used_concepts.find(second)->second;
                        }
                        interface.add_entity_attribute(first_concept_id = interface.add_entity(), "concept", first);
                        // Adding another reference to the entity that was just added 'first', that it 'is_a' second.
                        interface.add_entity_attribute(first_concept_id, "is_a", second_concept_id);
                    }

                    // There are different cases of processing depending on the csv file. The relations are slightly different for each.

                    if(filename == "names.csv") {
                        if(first == "Female") continue;
                        interface.add_entity_attribute(interface.add_entity(), "person_name", first);
                        interface.add_entity_attribute(interface.add_entity(), "person_name", second);
                    }

                    if(filename == "questions.csv") {
                        if(first == "QUESTION") continue;
                        interface.add_entity_attribute(first_concept_id = interface.add_entity(), "question", first);
                        int answer_id;
                        interface.add_entity_attribute(answer_id = interface.add_entity(), "answer", second);
                        interface.add_entity_attribute(answer_id, "answer_to", first_concept_id);

                    }

                    if(filename == "locations.csv") {

                        p = strtok(NULL, ",");
                        if(first == "Number") continue;

                        std::string third;

                        int location_id;
                        interface.add_entity_attribute(location_id = interface.add_entity(), "location", second);


                        if(p != NULL) {
                            third = p;
                            if(char(*third.begin()) == ' ') third.erase(third.begin());

                            int concept_id;
                            if(used_concepts.find(third) != used_concepts.end()) {
                                concept_id = used_concepts.find(third)->second;
                            }
                            else {
                                interface.add_entity_attribute(concept_id = interface.add_entity(), "concept", third);
                                used_concepts[third] = concept_id;
                            }



                            interface.add_entity_attribute(concept_id, "found_in", location_id);
                        }


                    }

                }

                p = strtok(NULL, ",");

            }


        }
    }

}













































