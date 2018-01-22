
#include "bwi_kr_execution/ExecutePlanAction.h"

#include <actionlib/client/simple_action_client.h>
#include <bwi_msgs/LogicalNavigationAction.h>
#include <tf/transform_listener.h>
#include <bwi_kr_execution/UpdateFluents.h>
#include <bwi_kr_execution/CurrentStateQuery.h>
#include <bwi_msgs/UpdateObject.h>

#include <geometry_msgs/Pose.h>
#include <std_srvs/Empty.h>
#include <ros/ros.h>

typedef actionlib::SimpleActionClient<bwi_kr_execution::ExecutePlanAction> Client;

using namespace std;

struct IsFluentAt {
  
  bool operator()(const bwi_kr_execution::AspFluent& fluent) {
    return fluent.name == "at";
  }
  
};

int main(int argc, char**argv) {
  ros::init(argc, argv, "look_for_person_node");
  ros::NodeHandle n;

  ros::NodeHandle privateNode("~");
  string person_to_find;
  privateNode.param<string>("person_to_find",person_to_find,"peter");
  string requester;
  privateNode.param<string>("requester",requester,"yuqian");

  Client client("action_executor/execute_plan", true);
  client.waitForServer();

  //get current pose as approach point for the requester
  tf::TransformListener listener;
  tf::StampedTransform transform;
  try{
    listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(1.0));
    while (! listener.canTransform("/map", "/base_link", ros::Time(0))) {
      ros::Duration(0.1).sleep();
    }
    listener.lookupTransform("/map", "/base_link",  
                             ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::shutdown();
  }
  geometry_msgs::Pose msg;
  tf::poseTFToMsg(transform, msg);

  //add object to map
  ros::ServiceClient lnClient = n.serviceClient<bwi_msgs::UpdateObject> ("update_object");
  lnClient.waitForExistence();
  bwi_msgs::UpdateObject uo;
  uo.request.object_name = requester+"_marker";
  uo.request.pose = msg;
  lnClient.call(uo);

  //get current logical location
  ros::ServiceClient krClient = n.serviceClient<bwi_kr_execution::CurrentStateQuery> ("current_state_query");
  krClient.waitForExistence();
  bwi_kr_execution::CurrentStateQuery csq;
  krClient.call(csq);
  vector<bwi_kr_execution::AspFluent>::const_iterator atIt = 
                    find_if(csq.response.answer.fluents.begin(), csq.response.answer.fluents.end(), IsFluentAt());
                    
  if (atIt == csq.response.answer.fluents.end()) {
    ROS_ERROR("look_for_person_node: fluent \"at\" missing ");
    ros::shutdown();
  }
  string location = atIt->variables[0];

  //update task information to KR
  krClient = n.serviceClient<bwi_kr_execution::UpdateFluents> ("update_fluents");
  krClient.waitForExistence();

  bwi_kr_execution::UpdateFluents uf;

  bwi_kr_execution::AspFluent lookingfor;
  lookingfor.name = "lookingfor";
  lookingfor.variables.push_back(person_to_find);
  uf.request.fluents.push_back(lookingfor);

  bwi_kr_execution::AspFluent inroom;
  inroom.name = "inroom";
  inroom.variables.push_back(requester);
  inroom.variables.push_back(location);
  uf.request.fluents.push_back(inroom);

  bwi_kr_execution::AspFluent locationmarker;
  locationmarker.name = "locationmarker";
  locationmarker.variables.push_back(requester);
  locationmarker.variables.push_back(requester+"_marker");
  uf.request.fluents.push_back(locationmarker);

  bwi_kr_execution::AspFluent message;
  message.name = "message";
  message.variables.push_back(requester);
  message.variables.push_back("message");
  uf.request.fluents.push_back(message);

  krClient.call(uf);

  ROS_INFO_STREAM("Looking for " << person_to_find);

  //construct ASP planning goal
  bwi_kr_execution::ExecutePlanGoal goal;

  //find the person or conclude that the person cannot be found at the moment
  bwi_kr_execution::AspRule found_rule;

  bwi_kr_execution::AspFluent found;
  found.name = "not found";
  found.variables.push_back(person_to_find);
  found_rule.body.push_back(found);

  bwi_kr_execution::AspFluent not_found;
  not_found.name = "not -found";
  not_found.variables.push_back(person_to_find);
  found_rule.body.push_back(not_found);

  goal.aspGoal.push_back(found_rule);

  //report back to where the task was sent
  bwi_kr_execution::AspRule report_back_rule;

  bwi_kr_execution::AspFluent delivered;
  delivered.name = "not messagedelivered";
  delivered.variables.push_back(requester);
  delivered.variables.push_back("message");
  report_back_rule.body.push_back(delivered);

  bwi_kr_execution::AspFluent not_inroom;
  not_inroom.name = "not -inroom";
  not_inroom.variables.push_back(requester);
  not_inroom.variables.push_back(location);
  report_back_rule.body.push_back(not_inroom);

  goal.aspGoal.push_back(report_back_rule);

  bwi_kr_execution::AspRule at_rule;

  bwi_kr_execution::AspFluent at;
  at.name = "not at";
  at.variables.push_back(location);
  at_rule.body.push_back(at);

  goal.aspGoal.push_back(at_rule);

  ROS_INFO("sending goal");
  client.sendGoalAndWait(goal);

  if (client.getState() == actionlib::SimpleClientGoalState::ABORTED) {
    ROS_INFO("Aborted");
  } else if (client.getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
    ROS_INFO("Preempted");
  }

  else if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Succeeded!");
  } else
    ROS_INFO("Terminated");
    
  client.cancelAllGoals();
  
  return 0;
}
