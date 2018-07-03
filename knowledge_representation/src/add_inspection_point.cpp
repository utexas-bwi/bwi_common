#include <knowledge_representation/LongTermMemoryConduit.h>
#include <iostream>
#include <string>
#include <knowledge_representation/MemoryConduit.h>

using ::std::cout;
using ::std::endl;
using namespace std;


int main(int argc, char *argv[]) {
    ros::init(argc, argv, "test_planner");
    knowledge_rep::MemoryConduit mc;
    knowledge_rep::LongTermMemoryConduit ltmc("127.0.0.1", 33060, "root", "", "villa_krr");

    int inspection_point_con = ltmc.get_concept("inspection point");
    int object_con = ltmc.get_concept("object");
    ltmc.add_entity_attribute(inspection_point_con , "is_a", object_con);

    int inspection_point = ltmc.add_entity();

    ltmc.add_entity_attribute(inspection_point, "is_a" , inspection_point_con);
    ltmc.add_entity_attribute(inspection_point, "map_name", "inspection point");
    
    cout << "Done!" << endl;
} 

