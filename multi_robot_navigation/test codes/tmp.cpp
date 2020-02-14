#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <bwi_msgs/LogicalActionAction.h>
#include <plan_execution/ExecutePlanAction.h>
#include <stdlib.h>
#include <time.h>
typedef actionlib::SimpleActionClient<plan_execution::ExecutePlanAction> Client;

using namespace std;
bool RANDOM_MOVE = false;
bool roberto_success = true;
bool marvin_success = true;

int marvin_goal = 0;
int roberto_goal = 0;

Client* client;
actionlib::SimpleActionClient<bwi_msgs::LogicalActionAction>* lnac;

void executePlan(string location){
  if(roberto_success == true){
    ROS_INFO_STREAM("Roberto going to " << location);

    plan_execution::ExecutePlanGoal goal;
    plan_execution::AspRule rule;
    plan_execution::AspFluent fluent;

    fluent.name = "not facing";
    fluent.variables.push_back(location);
    rule.body.push_back(fluent);
    goal.aspGoal.push_back(rule);

    ROS_INFO("sending goal");
    client->sendGoal(goal);
    roberto_success = false;
  }
  if(client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    roberto_success = true;
    roberto_goal = (roberto_goal + 1)%10;
  }
}
void bypassPlan(string location){
  if(marvin_success == true){
    bwi_msgs::LogicalActionGoal goal;
    std::vector<std::string> parameters;
    ROS_INFO_STREAM("Marvin going to "<< location);

    goal.command.name = "goto";
    parameters.push_back(location);
    parameters.push_back("1");
    goal.command.value = parameters;
    lnac->sendGoal(goal);
    marvin_success = false;
  }
  if(lnac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    marvin_success = true;
    marvin_goal = (marvin_goal + 1)%10;
  }
}

int main(int argc, char**argv){

  ros::init(argc, argv, "test");
  ros::NodeHandle nh("~");

  client = new Client("roberto/action_executor/execute_plan", true);
  lnac = new actionlib::SimpleActionClient<bwi_msgs::LogicalActionAction>("marvin/execute_logical_goal", true);
  client->waitForServer();
  lnac->waitForServer();

  vector<string> locations;
  locations.push_back("p3_15");
  locations.push_back("p3_16");
  locations.push_back("p3_17");
  locations.push_back("p3_18");
  locations.push_back("p3_19");
  locations.push_back("p3_20");
  locations.push_back("p3_21");
  locations.push_back("p3_22");
  locations.push_back("p3_44");
  locations.push_back("p3_45");


  if(RANDOM_MOVE){
    srand(time(NULL));
    int loc = rand()%locations.size();
    string location = locations.at(loc);
    ROS_INFO_STREAM("random number : "<<loc);
    ROS_INFO_STREAM("location size : "<<locations.size());
  //  n.getParam("location", location);
    marvin_goal = roberto_goal = loc;
  }
  else{
    ros::Rate loop_rate(10);
    while(ros::ok()){
      executePlan(locations.at(roberto_goal));
      bypassPlan(locations.at(marvin_goal));
      loop_rate.sleep();
    }
  }

  return 0;
}
