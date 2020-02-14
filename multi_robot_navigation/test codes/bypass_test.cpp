#include <actionlib/client/simple_action_client.h>

#include <bwi_msgs/LogicalActionAction.h>

#include "plan_execution/LogicalAction.h"


#include "actasp/AspFluent.h"


#include <bwi_planning_common/PlannerInterface.h>
#include <plan_execution/UpdateFluents.h>
#include <plan_execution/AspFluent.h>


#include <ros/ros.h>
#include <sstream>

using namespace ros;
using namespace std;
using namespace actasp;

int main(int argc, char**argv){
  ros::init(argc, argv, "bypass_test");
  ros::NodeHandle nh("~");

  actionlib::SimpleActionClient<bwi_msgs::LogicalActionAction>* lnac;
  bwi_msgs::LogicalActionGoal goal;

  lnac = new actionlib::SimpleActionClient<bwi_msgs::LogicalActionAction>("marvin/execute_logical_goal", true);
  lnac->waitForServer();

  std::vector<std::string> parameters;
/* // no need
  goal.command.name = "at";
  parameters.push_back("l3_500");
  parameters.push_back("0");
  goal.command.value = parameters;
  lnac->sendGoal(goal);
  lnac->waitForResult();
  parameters.clear();

  goal.command.name = "facing";
  parameters.push_back("p3_15");
  parameters.push_back("1");
  goal.command.value = parameters;
  lnac->sendGoal(goal);
  lnac->waitForResult();
  parameters.clear();
*/
  goal.command.name = "goto";
  parameters.push_back("p3_15");
  parameters.push_back("1");
  goal.command.value = parameters;
  lnac->sendGoal(goal);
  lnac->waitForResult();
  parameters.clear();
/*
  goal.command.name = "beside";
  parameters.push_back("p3_15");
  parameters.push_back("1");
  goal.command.value = parameters;
  lnac->sendGoal(goal);
  lnac->waitForResult();
  parameters.clear();
  return 0;
  */
}
