#include <knowledge_representation/LongTermMemoryConduit.h>
#include <iostream>
#include <string>
#include <knowledge_representation/MemoryConduit.h>
#include <knowledge_representation/convenience.h>
#include <knowledge_representation/Concept.h>


using std::vector;
using std::string;
using std::cout;
using std::endl;
using knowledge_rep::EntityAttribute;
using knowledge_rep::Concept;


int main(int argc, const char *argv[]) {
    auto ltmc = knowledge_rep::get_default_ltmc();

    ltmc.delete_all_entities();
    // Robot should exist
    assert(ltmc.entity_exists(1));

    knowledge_rep::Entity coke = ltmc.add_object();
    assert(ltmc.entity_exists(coke.entity_id));

    // Get concept returns the one true concept id
    Concept soda = ltmc.get_concept("soda");
    assert(soda.entity_id == ltmc.get_concept("soda").entity_id);

    Concept pitcher_con = ltmc.get_concept("soylent pitcher");
    knowledge_rep::Entity pitcher = ltmc.get_object_named("soylent pitcher");
    assert(ltmc.get_object_named("soylent pitcher").entity_id != pitcher_con.entity_id);
    assert(ltmc.entity_exists(pitcher.entity_id));

    // Named entity returns the only entity by that name
    assert(ltmc.get_object_named("soylent pitcher") == pitcher);

    // Get concept should return high number ids because it creates new entities and auto increments the key
    assert(coke.entity_id < ltmc.get_concept("never seen before").entity_id);

    knowledge_rep::Entity drinkable = ltmc.get_concept("drinkable");
    knowledge_rep::Entity can = ltmc.add_entity();

    soda.add_attribute("is_a", drinkable);
    // Adding a second time should fail
    //assert(ltmc.add_entity_attribute(soda, "is_a", drinkable) == false);
    can.add_attribute("is_a", soda);
    //assert(ltmc.add_entity_attribute(can, "not a real attribute", coke) == false);

    // Concept is an attribute, so we should be able to poke at it through here
    auto attrs = soda.get_attributes("is_concept");

    auto at = attrs[0];
    assert(boost::get<bool>(at.value) == true);

    attrs = can.get_attributes("is_a");
    assert(attrs.size() == 1);

    // The concrete object should vanish...
    can.delete_entity();
    assert(!ltmc.entity_exists(can.entity_id));

    // Along with its attributes
    attrs = can.get_attributes();
    assert(attrs.empty());

    auto all_objs = ltmc.get_all_entities();

    ltmc.delete_all_entities();
    all_objs = ltmc.get_all_entities();
    // The robot itself is always in the knowledge base
    assert(all_objs.size() == 2);
    assert(all_objs.at(0).entity_id == 1);

    
    //test recursive remove
    Concept parent = ltmc.get_concept("parent concept");
    Concept child = ltmc.get_concept("child concept");
    child.add_attribute("is_a", parent);
    knowledge_rep::Entity object = ltmc.add_entity();
    object.add_attribute("instance_of", child);
    parent.remove_instances();
    assert(!object.is_valid());

    {
        auto some_object = ltmc.get_object_named("test object");
        some_object.add_attribute("sensed", 1);
        attrs = some_object.get_attributes("sensed");
        assert(attrs.at(0).value.type() == typeid(int));
        assert(boost::get<int>(attrs.at(0).value) == 1);
        some_object.remove_attribute("sensed");

        some_object.add_attribute("sensed", 1.0f);
        attrs = some_object.get_attributes("sensed");
        assert(attrs.at(0).value.type() == typeid(float));
        assert(boost::get<float>(attrs.at(0).value) == 1.0f);
        some_object.remove_attribute("sensed");

        some_object.add_attribute("sensed", "test");
        attrs = some_object.get_attributes("sensed");
        assert(attrs.at(0).value.type() == typeid(string));
        assert(boost::get<string>(attrs.at(0).value) == "test");
        some_object.remove_attribute("sensed");

        some_object.add_attribute("sensed", true);
        attrs = some_object.get_attributes("sensed");
        assert(attrs.at(0).value.type() == typeid(bool));
        assert(boost::get<bool>(attrs.at(0).value) == true);
        some_object.remove_attribute("sensed");

        some_object.add_attribute("sensed", false);
        attrs = some_object.get_attributes("sensed");
        assert(attrs.at(0).value.type() == typeid(bool));
        assert(boost::get<bool>(attrs.at(0).value) == false);
        some_object.remove_attribute("sensed");
    }




    vector<EntityAttribute> query_result;
    ltmc.select_query<string>("SELECT * FROM entity_attributes_str", query_result);

    cout << "Done!" << endl;
} 

