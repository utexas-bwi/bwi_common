/*
This is a visit door list with 1.6 m hallway.
Currently, Client using plan_execution::ExecutePlanAction is not working.

*/
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <tf/transform_listener.h>
#include <ros/ros.h>

#define PI 3.141592

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace std;

void move(MoveBaseClient *client, vector<float> coord){
  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "marvin/level_mux_map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = coord[0];
  goal.target_pose.pose.position.y = coord[1];
  tf::Quaternion quat;
  quat.setRPY(0.0, 0.0, coord[2]);
  tf::quaternionTFToMsg(quat, goal.target_pose.pose.orientation);

  client->sendGoal(goal);
}
vector< vector<float> > add_coordinate(vector< vector<float> > list, float x, float y, float yaw){
  vector<float> tmp;
  tmp.push_back(x);
  tmp.push_back(y);
  tmp.push_back(yaw);
  list.push_back(tmp);
  return list;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "marvin_visit_door_list");
  ros::NodeHandle nh;
  ros::Rate rate(10);
  
  MoveBaseClient client("marvin/move_base");
  // initialize door list
  ros::Duration(0.5).sleep();

  int door = 0;
  vector< vector< float > > doors;
  doors = add_coordinate(doors, 7.249006, 9.592085, PI/2.);
  doors = add_coordinate(doors, 28.773991, 9.560767, PI/2.);
  doors = add_coordinate(doors, 29.370174, 17.971224, -PI/2.);
  doors = add_coordinate(doors, 7.682734, 17.971308, -PI/2.);

  move(&client, doors.at(door));
  while(ros::ok()){
    if(client.getState() == actionlib::SimpleClientGoalState::ABORTED){
      move(&client, doors.at(door));
    }
    if(client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      door = (door+1)%doors.size();
      move(&client, doors.at(door));
    }
    rate.sleep();
  }
}
