#include <actionlib/client/simple_action_client.h>
#include <plan_execution/ExecutePlanAction.h>

#include <visualization_msgs/MarkerArray.h>

#include <ros/ros.h>

#include <math.h>
#include <stdlib.h>
#include <time.h>
typedef actionlib::SimpleActionClient<plan_execution::ExecutePlanAction> Client;

using namespace std;
bool RANDOM_MOVE = false;
bool DEBUG = false;
bool roberto_success = true;
bool marvin_success = true;

int marvin_goal = 4;
int roberto_goal = 0;

void executePlan(Client *client, string location){
  plan_execution::ExecutePlanGoal goal;
  plan_execution::AspRule rule;
  plan_execution::AspFluent fluent;

  fluent.name = "not facing";
  fluent.variables.push_back(location);
  rule.body.push_back(fluent);
  goal.aspGoal.push_back(rule);

  client->sendGoal(goal);
}
/*
// This one does works
void robertoPlan(const visualization_msgs::MarkerArray::ConstPtr plan){
  ROS_INFO_STREAM("PLAN FROM ROBERTO");
}
*/
// This version also works!
vector<geometry_msgs::Pose> my_plan;
void robertoPlan(const visualization_msgs::MarkerArray plan){
  if(DEBUG) ROS_INFO_STREAM("PLAN FROM ROBERTO: " << plan.markers.size());
  my_plan.clear();
  geometry_msgs::Pose plan_pt;
  for(int i=0; i<plan.markers.size(); i++){
    plan_pt = plan.markers[i].pose;
    if(DEBUG){
      ROS_INFO_STREAM(i<<" th plan info");
      ROS_INFO_STREAM("x : "<<plan_pt.position.x<<", y : "<<plan_pt.position.y);
    }
    my_plan.push_back(plan_pt);
  }
}
vector<geometry_msgs::Pose> others_plan;
void marvinPlan(const visualization_msgs::MarkerArray plan){
  if(DEBUG) ROS_INFO_STREAM("PLAN FROM MARVIN: " << plan.markers.size());
  others_plan.clear();
  geometry_msgs::Pose plan_pt;
  for(int i=0; i<plan.markers.size(); i++){
    plan_pt = plan.markers[i].pose;
    others_plan.push_back(plan_pt);
  }
}
float dist(geometry_msgs::Point my_point, geometry_msgs::Point others_point){
  return hypot(my_point.x-others_point.x, my_point.y-others_point.y);
}
bool distanceBaseAlarm(vector<geometry_msgs::Pose> my_plan, vector<geometry_msgs::Pose> others_plan){
  bool danger = false;

  vector<geometry_msgs::Pose>::iterator my_iter = my_plan.begin();
  vector<geometry_msgs::Pose>::iterator others_iter = others_plan.begin();
  
  vector<geometry_msgs::Pose>::iterator my_position = my_plan.begin();
  vector<geometry_msgs::Pose>::iterator others_position = others_plan.begin();
  if(DEBUG) ROS_INFO("size of my plan %zd, size of others plan %zd", my_plan.size(), others_plan.size());
  // Find where two robots will collide in future;
  if(!my_plan.size() || !others_plan.size()){
//    ROS_INFO_STREAM("Someone doesn't have a plan!");
    return danger;
  }
  while(true){
    if(dist(my_iter->position, others_iter->position)<0.6){
      // Warn if two robot's future plan is overlapped
      ROS_INFO_STREAM("Case 1");
      ROS_INFO("IP : (%2.3f, %2.3f)", my_iter->position.x, my_iter->position.y);

      danger = true;
      break;
    }
    else if(dist(my_iter->position, others_position->position)<0.6){
      // Warn if I am approaching to stopped robot
      ROS_INFO_STREAM("Case 2");
      ROS_INFO("IP : (%2.3f, %2.3f)", others_position->position.x, others_position->position.y);

      danger = true;
      break;
    }
    else if(dist(my_position->position, others_iter->position)<0.6){
      // Warn if other robot is approaching to me(stopped)
      ROS_INFO_STREAM("Case 3");
      ROS_INFO("IP : (%2.3f, %2.3f)", my_position->position.x, my_position->position.y);

      danger = true;
      break;
    }
    ++my_iter;
    ++others_iter;
    if(my_iter == my_plan.end() || others_iter == others_plan.end()) break;
  }
  return danger;
}

int main(int argc, char**argv){

  ros::init(argc, argv, "test");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
  Client* roberto_client;
  Client* marvin_client;
  roberto_client = new Client("/roberto/action_executor/execute_plan", true);
  marvin_client = new Client( "/marvin/action_executor/execute_plan", true);
  roberto_client->waitForServer();
  marvin_client->waitForServer();

  ros::Subscriber roberto_plan = nh.subscribe("/roberto/move_base/EBandPlannerROS/eband_visualization_array", 1, &robertoPlan);
  ros::Subscriber marvin_plan = nh.subscribe("/marvin/move_base/EBandPlannerROS/eband_visualization_array", 1, &marvinPlan);
  ros::AsyncSpinner spinner(2);
  spinner.start();

  vector<string> doors;
  doors.push_back("d3_414b1");
  doors.push_back("d3_414b2");
  doors.push_back("d3_414a1");
  doors.push_back("d3_414a2");
  doors.push_back("d3_418");
  executePlan(roberto_client, doors.at(roberto_goal));
  executePlan(marvin_client, doors.at(marvin_goal));
  while(ros::ok()){
    bool roberto_status = (roberto_client->getState()==actionlib::SimpleClientGoalState::SUCCEEDED);
    bool marvin_status =  (marvin_client->getState() ==actionlib::SimpleClientGoalState::SUCCEEDED);
//    ROS_INFO_STREAM("Status of roberto : " << (status?"Success!":"Proceeding"));
    if(roberto_status){
//      ROS_INFO_STREAM("roberto proceed to "<<roberto_goal);
      roberto_goal = (++roberto_goal)%doors.size();
      executePlan(roberto_client, doors.at(roberto_goal));
    }
    else if(roberto_client->getState()==actionlib::SimpleClientGoalState::ABORTED){
      executePlan(roberto_client, doors.at(roberto_goal));
    }
    if(marvin_status){
      marvin_goal = (marvin_goal?--marvin_goal:doors.size()-1);
      executePlan(marvin_client, doors.at(marvin_goal));
    }
    else if(marvin_client->getState()==actionlib::SimpleClientGoalState::ABORTED){
      executePlan(marvin_client, doors.at(marvin_goal));
    }

    bool Danger = distanceBaseAlarm(my_plan, others_plan);
    loop_rate.sleep();
  }

  return 0;
}
