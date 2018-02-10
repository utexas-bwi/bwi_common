

#include <geometry_msgs/Twist.h> 
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>

#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_datatypes.h>


#include "SearchPlanner.h"

#include <fstream>

#define SCENE_ANALYZATION_COST (60)

geometry_msgs::PoseWithCovarianceStamped curr_position; 
//bwi_scavenger::VisionTaskGoal *goal; 

//actionlib::SimpleActionClient <bwi_scavenger::VisionTaskAction> * ac; 

// callback function that saves robot's current position
void callbackCurrPos(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    curr_position = *msg;
}


SearchPlannerSimple::SearchPlannerSimple(ros::NodeHandle *node_handle) {

    nh = node_handle;

    doors.push_back("d3_414b1");
    doors.push_back("d3_414b2");
    doors.push_back("d3_414a1");
    doors.push_back("d3_414a2");

    client = new KrClient("/action_executor/execute_plan", true);
    client->waitForServer(); 
    goal_door = 0; 
    srand(time(NULL)); 
   busy = false; 
}

bool SearchPlannerSimple::cancelCurrentGoal() {

    ros::Publisher pub1 = nh->advertise<actionlib_msgs::GoalID> 
        ("/move_base/cancel", 1000); 
    ros::Publisher pub2 = nh->advertise<actionlib_msgs::GoalID> 
        ("/move_base_interruptable/cancel", 1000); 
    actionlib_msgs::GoalID msg; 

    ros::spinOnce(); 
    pub1.publish(msg); 

    ros::spinOnce(); 
    pub2.publish(msg); 

    ros::spinOnce(); 

    client->cancelAllGoals(); 

}

bool SearchPlannerSimple::moveToNextDoor() {
    
    goal_door = (goal_door + rand()) % doors.size(); 

    ROS_INFO_STREAM("going to " << doors.at(goal_door));
    plan_execution::ExecutePlanGoal goal; 
    plan_execution::AspRule rule;
    plan_execution::AspFluent fluent;
    fluent.name = "not beside";
    fluent.variables.push_back(doors.at(goal_door));
    rule.body.push_back(fluent);
    goal.aspGoal.push_back(rule);

    ROS_INFO("sending goal"); 
    client->sendGoalAndWait(goal); 
    busy = true; 

    if (client->getState() == actionlib::SimpleClientGoalState::ABORTED) {
        ROS_INFO("Aborted");
    } else if (client->getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
        ROS_INFO("Preempted");
    }

    else if (client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Succeeded!");
    } else
        ROS_INFO("Terminated");

    busy = false; 
    return 0; 
}


SearchPlanner::SearchPlanner(ros::NodeHandle *node_handle, std::string path_to_yaml, float goal_tolerance) : 
        tolerance(goal_tolerance) {

    nh = node_handle; 
    client = new KrClient("/action_executor/execute_plan", true);
    client->waitForServer(); 

    YAML::Node yaml_positions; 
    std::ifstream fin(path_to_yaml.c_str()); 

#ifdef HAVE_NEW_YAMLCPP
    yaml_positions = YAML::Load(fin);
#else
    YAML::Parser parser(fin);
    parser.GetNextDocument(yaml_positions); 
#endif

    // YAML::Node yaml_positions = YAML::LoadFile(path_to_yaml); 

    belief = std::vector<float> (yaml_positions.size(), 1.0/yaml_positions.size()); 

    client_make_plan = nh->serviceClient <nav_msgs::GetPlan> ("move_base/GlobalPlanner/make_plan"); 

    pub_simple_goal = nh->advertise<geometry_msgs::PoseStamped>("/move_base_interruptable_simple/goal", 100);

    sub_amcl_pose = nh->subscribe("amcl_pose", 100, callbackCurrPos); 
    ros::Duration(1.0).sleep(); 

    for (int i=0; i<yaml_positions.size(); i++) {
#ifdef HAVE_NEW_YAMLCPP
        geometry_msgs::PoseStamped tmp_pose; 
        tmp_pose.header.frame_id = "level_mux_map";
        tmp_pose.pose.position.x = yaml_positions[i][0].as<double>(); 
        tmp_pose.pose.position.y = yaml_positions[i][1].as<double>(); 
        tmp_pose.pose.orientation.z = yaml_positions[i][2].as<double>(); 
        tmp_pose.pose.orientation.w = yaml_positions[i][3].as<double>(); 
        positions.push_back(tmp_pose); 
#else
#warning "yaml-cpp version < 0.5.0 does not work"
        // For Ubuntu Saucy to work, this would require a yaml-cpp
        // 0.3.0 compatible implementation.  Since we don't use Saucy,
        // we just let it fail.
#endif

    }        

    busy = false; 
    setTargetDetection(false); 
    ros::spinOnce(); 
}

geometry_msgs::PoseStamped SearchPlanner::selectNextScene(const std::vector<float> &belief, int &next_goal_index) {

    geometry_msgs::PoseStamped nextScene; 

    nav_msgs::GetPlan srv; 

    std::vector<float> distances(belief.size(), 0.0); 
    std::vector<float> fitness(belief.size(), 0.0); 
    
    for (unsigned i=0; i<positions.size(); i++) {

        srv.request.tolerance = tolerance; 
        srv.request.start.header.frame_id = "level_mux_map"; 
        srv.request.start.pose.position.x = curr_position.pose.pose.position.x; 
        srv.request.start.pose.position.y = curr_position.pose.pose.position.y; 

        srv.request.goal.header.frame_id = "level_mux_map"; 
        srv.request.goal.pose.position.x = positions[i].pose.position.x; 
        srv.request.goal.pose.position.y = positions[i].pose.position.y; 
    
        client_make_plan.waitForExistence();
        if (false == client_make_plan.call(srv))
            ROS_ERROR("Failed in calling service for making a path"); 
    
        distances[i] = srv.response.plan.poses.size();
    }
    
    next_goal_index = -1; 
    float max_value = -1.0; 

    float resolution;
    ros::param::param <float> ("/move_base/local_costmap/resolution", resolution, 0.05); 
    
    for (unsigned i=0; i<positions.size(); i++) {
        
        fitness[i] = belief[i] / (distances[i]*resolution + SCENE_ANALYZATION_COST); 
        next_goal_index = (fitness[i] >= max_value) ? i : next_goal_index; 
        max_value = (fitness[i] >= max_value) ? fitness[i] : max_value;
    }
    
    nextScene = positions[next_goal_index]; 
    nextScene.header.frame_id = "level_mux_map";
    return nextScene; 
}

void SearchPlanner::moveToNextScene(const geometry_msgs::PoseStamped &msg_goal) {

    bool hasArrived = false; 
    ROS_INFO("Moving to next scene"); 

    busy = true; 
    ros::Rate r(2);
    while (ros::ok() and r.sleep() and !hasArrived and false == getTargetDetection()) {

        ros::spinOnce(); 
        float x = msg_goal.pose.position.x - curr_position.pose.pose.position.x;
        float y = msg_goal.pose.position.y - curr_position.pose.pose.position.y;
        float dis = pow(x*x + y*y, 0.5); 

        // ROS_INFO("distance to next goal: %f", dis); 
        // ROS_INFO("tolerance: %f", tolerance); 
        hasArrived = dis < tolerance;

        // sometimes motion plannerg gets aborted for unknown reasons, so here 
        // we periodically re-send the goal
        pub_simple_goal.publish(msg_goal); 
    }
    ROS_INFO("Arrived"); 
    busy = false; 
}

void SearchPlanner::analyzeScene(float angle, float angular_vel) {

    ros::Publisher pub = nh->advertise <geometry_msgs::Twist> ("/cmd_vel", 100); 
    geometry_msgs::Twist vel; 
    vel.linear.x = 0;
    vel.linear.y = 0;

    busy = true; 
    for (int i=0; i<170; i++) {
        if (i<10 or i>160) {
            vel.angular.z = 0;
        } else if (i<60) {
            //ROS_INFO("Look to the left..."); 
            vel.angular.z = angular_vel;
        } else if (i<160) {
            //ROS_INFO("Look to the right...");
            vel.angular.z = (-1.0) * angular_vel;
        } 
        ros::spinOnce();
        pub.publish(vel); 
        ros::Duration(0.1).sleep();

        if (getTargetDetection())
            break; 
    }
    busy = false; 
}

void SearchPlanner::updateBelief(int next_goal_index) {
    
    float true_positive_rate, true_negative_rate;
    true_positive_rate = true_negative_rate = 0.95; 

    std::vector <float> tmp_belief (belief); 

    // if (detected)
    //     tmp_belief[next_goal_index] = tmp_belief[next_goal_index] * true_positive_rate / 
    //         ((tmp_belief[next_goal_index] * true_positive_rate) + 
    //          ((1.0 - tmp_belief[next_goal_index]) * (1.0 - true_negative_rate))); 
    // else
    //     tmp_belief[next_goal_index] = tmp_belief[next_goal_index] * (1.0 - true_positive_rate) / 
    //         ((tmp_belief[next_goal_index] * (1.0 - true_positive_rate)) + 
    //          ((1.0 - tmp_belief[next_goal_index]) * true_negative_rate)); 

    tmp_belief[next_goal_index] = tmp_belief[next_goal_index] * 
                                  (1.0 - true_positive_rate) / 
                                  ( (tmp_belief[next_goal_index] * (1.0 - true_positive_rate)) + 
                                    ((1.0 - tmp_belief[next_goal_index]) * true_negative_rate)
                                  ); 
    float normalizer = 0.0; 

    for (int i = 0; i < tmp_belief.size(); i++)
        normalizer += tmp_belief[i]; 

    for (int i = 0; i < tmp_belief.size(); i++)
        belief[i]  = tmp_belief[i] / normalizer; 
}

