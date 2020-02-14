#include <actionlib/client/simple_action_client.h>

#include "plan_execution/ExecutePlanAction.h"
//To use actionlib::SimpleActionClient<plan_execution::ExecutePlanAction>;
#include "bwi_msgs/LogicalActionAction.h"
//To use actionlib::SimpleActionClient<bwi_msgs::LogicalActionAction>;

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/GetPlan.h>

#include <ros/ros.h>
#include <math.h>
using namespace std;

bool SIMULATION = true;
bool DEBUG = false;
float WarningDistance = 4;

//string ROBOT_NAME = (SIMULATION?"/roberto":""); // IF runs on multi robot simulation, control roberto.
string ROBOT_NAME = (SIMULATION?"/roberto":""); // IF runs on multi robot simulation, control roberto.
string global_frame_id_ = (SIMULATION?"roberto/level_mux_map":"level_mux_map");
string MY_POSITION_TOPIC = (ROBOT_NAME + "/amcl_pose");
string OTHERS_POSITION_TOPIC = (SIMULATION?"/marvin/amcl_pose":"/others_amcl_pose");

void executeHigherPlan(const string location, actionlib::SimpleActionClient<plan_execution::ExecutePlanAction> *client){
  ROS_INFO_STREAM(ROBOT_NAME << " going to "<< location);

  plan_execution::ExecutePlanGoal goal;
  plan_execution::AspRule rule;
  plan_execution::AspFluent fluent;

  fluent.name = "not facing";
  fluent.variables.push_back(location);
  rule.body.push_back(fluent);
  goal.aspGoal.push_back(rule);
  client->sendGoal(goal);
}
int task_initialize(vector<string> *door_list, actionlib::SimpleActionClient<plan_execution::ExecutePlanAction> *client){
  // Initialize task here.
  client->waitForServer();
  door_list->push_back("d3_414b1");
  door_list->push_back("d3_414b2");
  door_list->push_back("d3_414a1");
  door_list->push_back("d3_414a2");
  door_list->push_back("d3_418");

  int current_door = 0;
  if(DEBUG)
    ROS_INFO_STREAM("Size of the door list is " << door_list->size());
  executeHigherPlan(door_list->at(current_door), client);
  return current_door+1;
}
int task_callback(int current_door, vector<string> door_list, actionlib::SimpleActionClient<plan_execution::ExecutePlanAction> *client){
  // Define task robot will do.
  if(client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    executeHigherPlan(door_list.at(current_door), client);
    return (current_door+1)%door_list.size();
  }
  return current_door;
};

class smartAvoidPolicy{
private:
  ros::NodeHandle* nh_;
  //geometry_msgs::PoseWithCovariance my_pose;
  //geometry_msgs::PoseWithCovariance others_pose;
  geometry_msgs::PoseStamped my_pose;
  geometry_msgs::PoseStamped others_pose;
  ros::ServiceClient GetPlanClient;

public:
  smartAvoidPolicy(ros::NodeHandle* nh){
    nh_ = nh;

  }
  void readMyPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose){
    if(DEBUG)
      ROS_INFO("My pose: %2.3g, %2.3g", pose->pose.pose.position.x, pose->pose.pose.position.y);
    //my_pose = pose->pose;
    my_pose.header = pose->header;
    my_pose.pose = pose->pose.pose;
  }
  void readOthersPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose){
    if(DEBUG)
      ROS_INFO("Other robot pose: %2.3g, %2.3g", pose->pose.pose.position.x, pose->pose.pose.position.y);
//    others_pose = pose->pose;
    others_pose.header = pose->header;
    others_pose.pose = pose->pose.pose;
  }
  float getDistance();
};
float smartAvoidPolicy::getDistance(){
  nav_msgs::GetPlan srv;
  GetPlanClient = nh_->serviceClient<nav_msgs::GetPlan>("roberto/move_base/NavfnROS/make_plan");
  GetPlanClient.waitForExistence();

  srv.request.start = my_pose;
  srv.request.start.header.frame_id = global_frame_id_;
  srv.request.goal = others_pose;
  srv.request.goal.header.frame_id = global_frame_id_;
  /*
  geometry_msgs::PoseStamped &start = srv.request.start;
  geometry_msgs::PoseStamped &goal  = srv.request.goal;

  start.header.frame_id = goal.header.frame_id = global_frame_id_;
  start.header.stamp    = goal.header.stamp    = ros::Time::now();

  start.pose.position.x = my_pose.pose.position.x;
  start.pose.position.y = my_pose.pose.position.y;
  start.pose.position.z = 0;
  start.pose.orientation.w = my_pose.pose.orientation.w;

  goal.pose.position.x = others_pose.pose.position.x;
  goal.pose.position.y = others_pose.pose.position.y;
  goal.pose.position.z = 0;
  goal.pose.orientation.w = others_pose.pose.orientation.w;
*/
  srv.request.tolerance = 0.0f;

  float distance = 0;

//  float prev_x = start.pose.position.x;
//  float prev_y = start.pose.position.y;
  float prev_x = my_pose.pose.position.x;
  float prev_y = my_pose.pose.position.y;

  float curr_x, curr_y;

  if(GetPlanClient.call(srv)){
    if(srv.response.plan.poses.size() != 0){
      if(DEBUG) ROS_INFO_STREAM("plan size: "<<srv.response.plan.poses.size());
      //ROS_INFO_STREAM("From ("<<start.pose.position.x<<", "<<start.pose.position.y<<")");
      //ROS_INFO_STREAM("  to ("<<goal.pose.position.x <<", "<<goal.pose.position.y <<")");
      ROS_INFO_STREAM("From ("<<my_pose.pose.position.x<<", "<<my_pose.pose.position.y<<")");
      ROS_INFO_STREAM("  to ("<<others_pose.pose.position.x <<", "<<others_pose.pose.position.y <<")");
      ROS_INFO_STREAM("plan size: "<<srv.response.plan.poses.size());
      for(int i = 1; i<srv.response.plan.poses.size(); i++){
        geometry_msgs::PoseStamped p = srv.response.plan.poses[i];
        curr_x = p.pose.position.x;
        curr_y = p.pose.position.y;
        if(abs(curr_x - prev_x) + abs(curr_y - prev_y) > 1){
          distance = distance + hypot(curr_x - prev_x, curr_y - prev_y);
          prev_x = curr_x;
          prev_y = curr_y;
        }
      }
    }else{
      return {99999.9f};
    }
  }
  else{
    ROS_INFO_STREAM("Planning failed");
  }
  if(DEBUG) ROS_INFO_STREAM("Distance to other robot : " << distance);
  return distance;
};
int main(int argc, char**argv){
  ros::init(argc, argv, "between_doors_safely");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);

  smartAvoidPolicy sap(&nh);
  ros::Subscriber sub1 = nh.subscribe(MY_POSITION_TOPIC, 1, &smartAvoidPolicy::readMyPose, &sap);
  ros::Subscriber sub2 = nh.subscribe(OTHERS_POSITION_TOPIC, 1, &smartAvoidPolicy::readOthersPose, &sap);
  ros::AsyncSpinner spinner(3);
  spinner.start();
  actionlib::SimpleActionClient<plan_execution::ExecutePlanAction> *client;
  client = new actionlib::SimpleActionClient<plan_execution::ExecutePlanAction>(ROBOT_NAME + "/action_executor/execute_plan", true);
  vector<string> door_list;
  int current_door;
  current_door = task_initialize(&door_list, client);
  ROS_INFO_STREAM("wait For 20 s");
  usleep(12000000);
  ROS_INFO_STREAM("DONE");
  client->cancelGoal();
  //
  ros::Publisher pub;
  pub = nh.advertise<actionlib_msgs::GoalID>(ROBOT_NAME + "/move_base/cancel", 1, true);
  //
  usleep(6000000);
  while(ros::ok()){
    ROS_INFO_STREAM(ROBOT_NAME + "/move_base/cancel");
    actionlib_msgs::GoalID msg;
    msg.id = "";
    for(int i=0; i<1; i++){
      usleep(1000); // 1/1000 s
      pub.publish(msg);
    }
    break;
  }
/*
  while(ros::ok()){
    // Do task
    current_door = task_callback(current_door, door_list, client);
    // Check distances
    float dist = sap.getDistance();
    loop_rate.sleep();
  }*/
  return 0;
}
