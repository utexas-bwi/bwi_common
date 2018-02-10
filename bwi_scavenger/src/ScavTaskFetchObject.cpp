
#include <actionlib/client/simple_action_client.h>
#include <vector>
#include <fstream>

#include "ScavTaskFetchObject.h"
#include "plan_execution/ExecutePlanAction.h"
#include "bwi_msgs/QuestionDialog.h"

std::string path_to_text; 

ScavTaskFetchObject::ScavTaskFetchObject(ros::NodeHandle *nh, std::string dir) {
    this->nh = nh; 
    directory = dir; 
    task_description = "fetch an object from a place to another"; 
    task_name = "Fetch object"; 
    certificate = ""; 
    _target_detected = false; 
    task_completed = false; 
}


void ScavTaskFetchObject::executeTask(int timeout, TaskResult &result, std::string &record) {

    boost::thread motion( &ScavTaskFetchObject::motionThread, this);
    boost::thread hri( &ScavTaskFetchObject::hriThread, this);

    _target_detected = false; 

    ros::Rate r(2); 
    while (ros::ok() and r.sleep()) {
        if (task_completed) {
            search_planner_simple->cancelCurrentGoal(); 
            break; 
        }
    }

    motion.detach();
    hri.detach();

    search_planner_simple->cancelCurrentGoal(); 

    record = path_to_text;
    result = SUCCEEDED; 

}
    
// motion thread
void ScavTaskFetchObject::motionThread() {

    std::string path_to_yaml; 

    if (false == nh->hasParam("path_to_search_points"))
        ROS_ERROR("path to yaml file of search points not set"); 
    ros::param::get("path_to_search_points", path_to_yaml); 

    search_planner_simple = new SearchPlannerSimple(nh);
    ros::Rate r(2);           

    while (ros::ok() and !_target_detected and r.sleep()) {
        search_planner_simple->moveToNextDoor();
    }
}

// human-robot interaction thread
void ScavTaskFetchObject::hriThread() {
    gui_service_client = new ros::ServiceClient(nh->serviceClient <bwi_msgs::QuestionDialog> ("question_dialog")); 

    ros::Duration(0.5).sleep(); 
    bwi_msgs::QuestionDialog srv;
    
    // specify the task 
    srv.request.type = bwi_msgs::QuestionDialogRequest::CHOICE_QUESTION; 
    srv.request.message = "Please click the button, if you can help with specifying the 'fetch object' task"; 
    srv.request.options = std::vector<std::string>(1, "button");
    srv.request.timeout = bwi_msgs::QuestionDialogRequest::NO_TIMEOUT; 
    gui_service_client->waitForExistence(); 
    gui_service_client->call(srv); 

    _target_detected = true; 

    // what's the object's name? saved to room_from
    srv.request.type = 2;
    srv.request.message = "Please follow me until I stop.\nWhat is the object's name?"; 
    gui_service_client->call(srv);
    object_name = srv.response.text;

    // where to get the object, saved to room_from
    srv.request.type = 2;
    srv.request.message = "Where to get the object? E.g., l3_420."; 
    gui_service_client->call(srv);
    room_name_from = srv.response.text;

    // where to deliver the object, saved to room_to
    srv.request.type = 2;
    srv.request.message = "Where to deliver the object? E.g., l3_420."; 
    gui_service_client->call(srv);
    room_name_to = srv.response.text;

    actionlib::SimpleActionClient<plan_execution::ExecutePlanAction> asp_plan_client("/action_executor/execute_plan", true); 
    asp_plan_client.waitForServer();

    // print to gui: moving to the first room
    srv.request.type = 0;
    srv.request.message = "Moving to room: " + room_name_from; 
    gui_service_client->call(srv);

    // wrapping up a "goal" for moving the robot platform
    plan_execution::ExecutePlanGoal goal;
    plan_execution::AspRule rule;
    plan_execution::AspFluent fluent;

    // move to the room to ask for the object
    fluent.name = "not at";
    fluent.variables.push_back(room_name_from);
    rule.body.push_back(fluent);
    goal.aspGoal.push_back(rule);
    ROS_INFO("sending goal");
    asp_plan_client.sendGoalAndWait(goal);

    ROS_INFO(asp_plan_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED ? "Succeeded!" : "Failed");

    // request to load object
    srv.request.type = 1; 
    srv.request.message = "Please help me load object: " + object_name; 
    srv.request.options = std::vector <std::string> (1, "Loaded"); 
    gui_service_client->call(srv); 

    // print to gui: moving to the second room
    srv.request.type = 0;
    srv.request.message = "Moving to room: " + room_name_to; 
    gui_service_client->call(srv);

    // move to the second room for delivery
    fluent.variables.clear(); 
    rule.body.clear();
    goal.aspGoal.clear(); 

    fluent.name = "not at";
    fluent.variables.push_back(room_name_to);
    rule.body.push_back(fluent);
    goal.aspGoal.push_back(rule);
    ROS_INFO("sending goal");
    asp_plan_client.sendGoalAndWait(goal);

    ROS_INFO(asp_plan_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED ? "Succeeded!" : "Failed!");

    // request to unload object
    srv.request.type = 1; 
    srv.request.message = "Please help me unload object: " + object_name; 
    std::vector<std::string> buttons(1, "Unloaded"); 
    srv.request.options = buttons;
    gui_service_client->call(srv); 

    boost::posix_time::ptime curr_time = boost::posix_time::second_clock::local_time(); 
    std::string path_to_text = directory;
    path_to_text += "fetch_object_" + boost::posix_time::to_simple_string(curr_time); 

    if (false == boost::filesystem::is_directory(directory)) {
        boost::filesystem::path tmp_path(directory); 
        boost::filesystem::create_directory(tmp_path); 
    } else {
        std::ofstream fs;
        fs.open(path_to_text.c_str(), std::ofstream::app); 
        fs << "origin room: " << room_name_from + "\n" 
            << "object name: " << object_name + "\n" 
            << "destination room: "+ room_name_to + "\n"; 
        fs.close(); 
    }

    ROS_INFO("fetch_object_service task done"); 
    task_completed = true; 
}

void ScavTaskFetchObject::stopEarly() {
    task_completed = true; 
}
