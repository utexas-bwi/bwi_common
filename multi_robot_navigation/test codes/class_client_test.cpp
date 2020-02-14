#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <bwi_msgs/LogicalActionAction.h>
#include <plan_execution/ExecutePlanAction.h>
#include <stdlib.h>
#include <time.h>
typedef actionlib::SimpleActionClient<plan_execution::ExecutePlanAction> Client;

using namespace std;
bool once = true;

void executePlan(Client* client, string location){
  ROS_INFO_STREAM("Going to "<<location);
  plan_execution::ExecutePlanGoal goal;
  plan_execution::AspRule rule;
  plan_execution::AspFluent fluent;

  fluent.name = "not facing";
  fluent.variables.push_back(location);
  rule.body.push_back(fluent);
  goal.aspGoal.push_back(rule);

  client->sendGoal(goal);
}

class testClient{
public:
  testClient(Client* client){
    this->client_ = client;
  }
  void cancelASPplan(){
    this->client_->cancelGoal();
  }
  void replanASPplan(string location){
    //executeASPplan(location);
    executeLogicalAction(location);
    while(this->client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){}
    //bool check = this->client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED;
    //ROS_INFO_STREAM((check?"No still succeeded":"Yes. It works!"));
    usleep(3000000);
  }
  void executeASPplan(string location){
    ROS_INFO_STREAM("Going to "<<location);
    plan_execution::ExecutePlanGoal goal;
    plan_execution::AspRule rule;
    plan_execution::AspFluent fluent;

    fluent.name = "not facing";
    fluent.variables.push_back(location);
    rule.body.push_back(fluent);
    goal.aspGoal.push_back(rule);

    this->client_->sendGoal(goal);
  }
  void executeLogicalAction(string location){
    actionlib::SimpleActionClient<bwi_msgs::LogicalActionAction>* lnac;
    bwi_msgs::LogicalActionGoal goal;

    lnac = new actionlib::SimpleActionClient<bwi_msgs::LogicalActionAction>("roberto/execute_logical_goal", true);
    lnac->waitForServer();

    vector<string> parameters;
    goal.command.name = "goto";
    parameters.push_back(location);
    parameters.push_back("1");
    goal.command.value = parameters;

    lnac->sendGoal(goal);
    parameters.clear();
  }

protected:
  Client* client_;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "class_client_test");
  ros::NodeHandle nh;
  ros::Rate loop_rate(100);
  Client* roberto_client;
  roberto_client = new Client("/roberto/action_executor/execute_plan", true);
  roberto_client->waitForServer();

  testClient test(roberto_client);

  vector<string> doors;
  doors.push_back("d3_414b1");
  doors.push_back("d3_414b2");
  doors.push_back("d3_414a1");
  doors.push_back("d3_414a2");
  doors.push_back("d3_418");
  int door = 0;

  executePlan(roberto_client, doors.at(door));

  ROS_INFO_STREAM("sleep for 15 s");
  usleep(15000000);
  ROS_INFO_STREAM("sleep finished");

  while(ros::ok()){
    if(once){
      test.cancelASPplan();
      test.replanASPplan("p3_45");
      ROS_INFO_STREAM("replan done");
      once = false;
    }
    if(roberto_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      door = (door+1)%doors.size();
      executePlan(roberto_client, doors.at(door));
    }
    loop_rate.sleep();
  }
}
