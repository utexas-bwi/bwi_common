#include <knowledge_representation/LongTermMemoryConduitInterface.h>
#include <iostream>
#include <string>
#include <knowledge_representation/MemoryConduit.h>
#include <knowledge_representation/convenience.h>
#include <knowledge_representation/LTMCConcept.h>
#include <knowledge_representation/LTMCInstance.h>


using std::vector;
using std::string;
using std::cout;
using std::endl;
using knowledge_rep::EntityAttribute;
using knowledge_rep::Entity;
using knowledge_rep::Concept;
using knowledge_rep::Instance;

static auto ltmc = knowledge_rep::get_default_ltmc();

void about_id(const int id) {
    auto entity = Entity(id, ltmc);
    if (!entity.is_valid()) {
        cout << id << " does not exist" << endl;
    }
    auto attributes = entity.get_attributes();
    for (const auto &attr: attributes) {
        if (attr.value.type() == typeid(int)) {
            Entity target(attr.get_int_value(), ltmc);
            string name_to_print = "-";
            auto name = target.get_name();
            if (name) {
                name_to_print = *name;
            }
            cout << attr.attribute_name << "\t" << std::to_string(target.entity_id) << " (" << name_to_print << ")" << endl;
        } else {
            cout << attr.attribute_name << "\t" << knowledge_rep::to_string(attr.value) << endl;
        }
    }
}

void concept_instances(const std::string &concept_name) {
    auto concept = ltmc.get_concept(concept_name);
    auto instances = concept.get_instances();
    cout << instances.size() << " instances of " << concept_name << endl;
    cout << "ID \t Name" << endl;
    for (auto &instance: instances) {
        auto name = instance.get_name();
        string name_to_print = "-";
        if (name) {
            name_to_print = *name;
        }
        cout << instance.entity_id << "\t" << name_to_print << endl;
    }
}

int main(int argc, const char *argv[]) {
    if (argc == 1) {
        cout << "Please provide a concept name or entity id" << endl;
        exit(1);
    }

    auto first_arg = argv[1];

    try {
        int entity_id = boost::lexical_cast<int>(first_arg);
        about_id(entity_id);
        return 0;
    } catch (boost::bad_lexical_cast &e) {

    }

    concept_instances(first_arg);
    return 0;

}

