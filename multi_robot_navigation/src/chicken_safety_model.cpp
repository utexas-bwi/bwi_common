/*
This is a visit door list with 1.6 m hallway.
Currently, Client using plan_execution::ExecutePlanAction is not working.

*/

#include "ChickenSafety.h"

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <tf/transform_listener.h>
#include <ros/ros.h>

#define PI 3.141592

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace std;

bool DEBUG = false;
bool REVERSE = true;
string ROBOT_NAME = "/roberto";
string HOST_NAME  = "/jinsoo";


vector< vector<float> > add_coordinate(vector< vector<float> > list, float x, float y, float yaw){
  vector<float> tmp;
  tmp.push_back(x);
  tmp.push_back(y);
  tmp.push_back(yaw);
  list.push_back(tmp);
  return list;
}
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
void goToParkingZone(MoveBaseClient *client, string location){
  vector<float> coord;
  if(location=="p3_15"){
    coord.push_back(3.627);
    coord.push_back(10.41);
    coord.push_back(-PI/2);
    move(client, coord);
  }
  else if(location=="p3_16"){
    coord.push_back(5.65);
    coord.push_back(10.46);
    coord.push_back(-PI/2);
    move(client, coord);
  }
  else if(location=="p3_18"){
    coord.push_back(17.922947);
    coord.push_back(10.459691);
    coord.push_back(-PI/2);
    move(client, coord);
  }
  else if(location=="p3_20"){
    coord.push_back(20.495056);
    coord.push_back(8.198261);
    coord.push_back(PI/2);
    move(client, coord);
  }
  else if(location=="p3_30"){
    coord.push_back(32.467041);
    coord.push_back(8.474984);
    coord.push_back(PI/2);
    move(client, coord);
  }
  else if(location=="p3_31"){
    coord.push_back(39.029182);
    coord.push_back(8.375641);
    coord.push_back(PI/2);
    move(client, coord);
  }
  else if(location=="p3_44"){
    coord.push_back(23.564869);
    coord.push_back(19.213001);
    coord.push_back(-PI/2);
    move(client, coord);
  }
  else if(location=="p3_45"){
    coord.push_back(3.834564);
    coord.push_back(19.154118);
    coord.push_back(-PI/2);
  }
  else{
    ROS_INFO_STREAM(location << " not understood");
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "chicken_visit_door");
  ros::NodeHandle nh;
  ros::Rate rate(10);
  ChickenSafetyProtocol csp(ROBOT_NAME, HOST_NAME);
  MoveBaseClient client("roberto/move_base");
  // initialize door list
  ros::Duration(0.5).sleep();

  int door = 0;
  vector< vector< float > > doors;
  doors = add_coordinate(doors, 7.249006, 9.592085, PI/2.);
  doors = add_coordinate(doors, 28.773991, 9.560767, PI/2.);
  doors = add_coordinate(doors, 29.370174, 17.971224, -PI/2.);
  doors = add_coordinate(doors, 7.682734, 17.971308, -PI/2.);

  bool danger = false;
  bool resume = false;
  string parkingZone;

  move(&client, doors.at(door));
  while(ros::ok()){
    bool danger = csp.distanceBaseAlarm(0.6);
    if(danger == true){ // if it detect collision
      client.cancelGoal(); // cancel current goal
      parkingZone = csp.findSafeZone();
//      csp.goToSafeZone(parkingZone); // This is NOT working in 1.6m hallway due to some map mismatch
      goToParkingZone(&client, parkingZone);

      ROS_INFO_STREAM("Potential Collision detected");
      ROS_INFO_STREAM("Going to "<< parkingZone);

      while(ros::ok()){
        if(csp.waitUntilSafe()) break;
        rate.sleep();
      }
      resume = true;
    }
    else{
      if(resume || client.getState() == actionlib::SimpleClientGoalState::ABORTED){
        resume = false;
        move(&client, doors.at(door));
      }
      if(client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        if(REVERSE == true){
          door = (door?door-1:doors.size()-1);
        }
        else{
          door = (door+1)%doors.size();
        }
        move(&client, doors.at(door));
      }
    }
    rate.sleep();
  }
}
