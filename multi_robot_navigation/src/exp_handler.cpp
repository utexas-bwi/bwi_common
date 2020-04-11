#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

// Modules for client
#include "plan_execution/ExecutePlanAction.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
// Modules for msg
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_srvs/Empty.h>

#include <visualization_msgs/MarkerArray.h>
// Teleport interface
#include <bwi_msgs/RobotTeleporterInterface.h>

// Safety protocol
#include "ChickenSafety.h"
#include "multi_robot_collision_avoidance/EvalWaypoint.h"

#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <ros/package.h>
// Write simulation results in file.
#include <fstream>

#include <random>

#define PI (3.1415926)
typedef actionlib::SimpleActionClient<plan_execution::ExecutePlanAction> Client;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace std;
// global collision check parameter
bool collision_check = false;
bool episode_fail = false;

class MRH_Experiment{
protected:
	MoveBaseClient marvin_ac;
	MoveBaseClient roberto_ac;
  ros::NodeHandle *nh_;

  geometry_msgs::PoseWithCovarianceStamped roberto_spawn_pose;
  move_base_msgs::MoveBaseGoal roberto_init_pose;
  move_base_msgs::MoveBaseGoal roberto_goal_pose;
  move_base_msgs::MoveBaseGoal roberto_standby_pose;
  ros::ServiceClient roberto_teleport;
  ros::ServiceClient roberto_clear_costmap;
  ros::Publisher     roberto_localizer;

  geometry_msgs::PoseWithCovarianceStamped marvin_spawn_pose;
  move_base_msgs::MoveBaseGoal marvin_init_pose;
  move_base_msgs::MoveBaseGoal marvin_goal_pose;
  move_base_msgs::MoveBaseGoal marvin_standby_pose;
  ros::ServiceClient marvin_teleport;
  ros::ServiceClient marvin_clear_costmap;
  ros::Publisher     marvin_localizer;
	// Measuring times
	ros::Time start_time;
	ros::Rate *rate;
	// subscribe plans
  vector<geometry_msgs::Pose> robertoPlan_;
  vector<geometry_msgs::Pose> marvinPlan_;
  ros::Subscriber subscribeRobertoPlan_;
  ros::Subscriber subscribeMarvinPlan_;
  ros::AsyncSpinner *spinner;
	void subscribeRobertoPlan(const visualization_msgs::MarkerArray plan);
	void subscribeMarvinPlan(const visualization_msgs::MarkerArray plan);

	ros::ServiceClient siren_find;

public:
  MRH_Experiment():marvin_ac(""),roberto_ac(""){}
  MRH_Experiment(ros::NodeHandle* nh);
	// Reset environments
  void setSpawnPose(string name, float x, float y, float yaw);
  void setInitPose(string name, float x, float y, float yaw);
	void setGoalPose(string name, float x, float y, float yaw);
	void setStandbyPose(string name, float x, float y, float yaw);
  void showInitPose(){
    ROS_INFO_STREAM(roberto_init_pose);
    ROS_INFO_STREAM(marvin_init_pose);
  }
  bool reset_environment();
	// basic fuctions
	void move(string robot_name, move_base_msgs::MoveBaseGoal goal);
	float dist(geometry_msgs::Point my_plan_pt, geometry_msgs::Point others_plan_pt);
	bool checkCollision();
	// Experiments
	// without safety protocol
	vector<double> run_raw_experiment(int timeout, int delay);
	// Chicken safety protocol
	vector<double> run_chicken_experiment(int timeout);
	void goToParkingZone(string location);
	void go(vector<float> coord);
	// Siren safety protocol
	vector<double> run_siren_experiment(int timeout);
};

MRH_Experiment::MRH_Experiment(ros::NodeHandle* nh):marvin_ac("marvin/move_base"),roberto_ac("roberto/move_base")
{
	// Load Gazebo

  nh_ = nh;
	rate = new ros::Rate(10);

  nh_ -> setParam("roberto/move_base/recovery_behavior_enabled", false);
  nh_ -> setParam("marvin/move_base/recovery_behavior_enabled", false);

  // Teleport service client
  roberto_teleport = nh_->serviceClient<bwi_msgs::RobotTeleporterInterface>("roberto/teleport_robot");
  roberto_teleport.waitForExistence(ros::Duration(30));
  roberto_clear_costmap = nh_->serviceClient<std_srvs::Empty>("roberto/move_base/clear_costmaps");
  roberto_clear_costmap.waitForExistence(ros::Duration(30));
  marvin_teleport = nh_->serviceClient<bwi_msgs::RobotTeleporterInterface>("marvin/teleport_robot");
  marvin_teleport.waitForExistence(ros::Duration(30));
  marvin_clear_costmap = nh_->serviceClient<std_srvs::Empty>("marvin/move_base/clear_costmaps");
  marvin_clear_costmap.waitForExistence(ros::Duration(30));

  roberto_localizer = nh_->advertise<geometry_msgs::PoseWithCovarianceStamped>("/roberto/initialpose",1,true);
  marvin_localizer = nh_->advertise<geometry_msgs::PoseWithCovarianceStamped>("/marvin/initialpose",1,true);

	// subscribe EBandPlanner topic
	string commonEBand = "/move_base/EBandPlannerROS/eband_visualization_array";
	string myEBandTopic = "/roberto" + commonEBand;

	string othersEBandTopic = "/marvin" + commonEBand;
	this->subscribeRobertoPlan_ = nh_->subscribe(myEBandTopic, 1, &MRH_Experiment::subscribeRobertoPlan, this);
	this->subscribeMarvinPlan_ = nh_->subscribe(othersEBandTopic, 1, &MRH_Experiment::subscribeMarvinPlan, this);
	this->spinner = new ros::AsyncSpinner(2);
	this->spinner->start();

	siren_find = nh->serviceClient<multi_robot_collision_avoidance::EvalWaypoint>("eval_waypoint");
};
void MRH_Experiment::subscribeRobertoPlan(const visualization_msgs::MarkerArray plan){
  this->robertoPlan_.clear();
  for(int i=0; i<plan.markers.size(); i++){
    this->robertoPlan_.push_back(plan.markers[i].pose);
  }
};
void MRH_Experiment::subscribeMarvinPlan(const visualization_msgs::MarkerArray plan){
  this->marvinPlan_.clear();
  for(int i=0; i<plan.markers.size(); i++){
    this->marvinPlan_.push_back(plan.markers[i].pose);
  }
};
void MRH_Experiment::setInitPose(string name, float x, float y, float yaw){
  if(name == "marvin"){
    marvin_init_pose.target_pose.header.frame_id = "marvin/level_mux_map";
    marvin_init_pose.target_pose.header.stamp = ros::Time::now();

    marvin_init_pose.target_pose.pose.position.x = x;
    marvin_init_pose.target_pose.pose.position.y = y;
    tf::Quaternion marvin_quat;
    marvin_quat.setRPY(0.0, 0.0, yaw);
    tf::quaternionTFToMsg(marvin_quat, marvin_init_pose.target_pose.pose.orientation);
  }
  else if(name=="roberto"){
    roberto_init_pose.target_pose.header.frame_id = "roberto/level_mux_map";
    roberto_init_pose.target_pose.header.stamp = ros::Time::now();

    roberto_init_pose.target_pose.pose.position.x = x;
    roberto_init_pose.target_pose.pose.position.y = y;
    tf::Quaternion roberto_quat;
    roberto_quat.setRPY(0.0, 0.0, yaw);
    tf::quaternionTFToMsg(roberto_quat, roberto_init_pose.target_pose.pose.orientation);
  }
  else{
    ROS_INFO_STREAM("Available Options: [roberto marvin]");
  }
};
void MRH_Experiment::setGoalPose(string name, float x, float y, float yaw){
  if(name == "marvin"){
    marvin_goal_pose.target_pose.header.frame_id = "marvin/level_mux_map";
    marvin_goal_pose.target_pose.header.stamp = ros::Time::now();

    marvin_goal_pose.target_pose.pose.position.x = x;
    marvin_goal_pose.target_pose.pose.position.y = y;
    tf::Quaternion marvin_quat;
    marvin_quat.setRPY(0.0, 0.0, yaw);
    tf::quaternionTFToMsg(marvin_quat, marvin_goal_pose.target_pose.pose.orientation);
  }
  else if(name=="roberto"){
    roberto_goal_pose.target_pose.header.frame_id = "roberto/level_mux_map";
    roberto_goal_pose.target_pose.header.stamp = ros::Time::now();

    roberto_goal_pose.target_pose.pose.position.x = x;
    roberto_goal_pose.target_pose.pose.position.y = y;
    tf::Quaternion roberto_quat;
    roberto_quat.setRPY(0.0, 0.0, yaw);
    tf::quaternionTFToMsg(roberto_quat, roberto_goal_pose.target_pose.pose.orientation);
  }
  else{
    ROS_INFO_STREAM("Available Options: [roberto marvin]");
  }
};
void MRH_Experiment::setStandbyPose(string name, float x, float y, float yaw){
  if(name == "marvin"){
    marvin_standby_pose.target_pose.header.frame_id = "marvin/level_mux_map";
    marvin_standby_pose.target_pose.header.stamp = ros::Time::now();

    marvin_standby_pose.target_pose.pose.position.x = x;
    marvin_standby_pose.target_pose.pose.position.y = y;
    tf::Quaternion marvin_quat;
    marvin_quat.setRPY(0.0, 0.0, yaw);
    tf::quaternionTFToMsg(marvin_quat, marvin_standby_pose.target_pose.pose.orientation);
  }
  else if(name=="roberto"){
    roberto_standby_pose.target_pose.header.frame_id = "roberto/level_mux_map";
    roberto_standby_pose.target_pose.header.stamp = ros::Time::now();

    roberto_standby_pose.target_pose.pose.position.x = x;
    roberto_standby_pose.target_pose.pose.position.y = y;
    tf::Quaternion roberto_quat;
    roberto_quat.setRPY(0.0, 0.0, yaw);
    tf::quaternionTFToMsg(roberto_quat, roberto_standby_pose.target_pose.pose.orientation);
  }
  else{
    ROS_INFO_STREAM("Available Options: [roberto marvin]");
  }
};
void MRH_Experiment::setSpawnPose(string name, float x, float y, float yaw){
  if(name == "marvin"){
    marvin_spawn_pose.header.frame_id = "marvin/level_mux_map";
    marvin_spawn_pose.header.stamp = ros::Time::now();

    marvin_spawn_pose.pose.pose.position.x = x;
    marvin_spawn_pose.pose.pose.position.y = y;
    marvin_spawn_pose.pose.pose.position.z = 0.0;
    tf::Quaternion marvin_quat;
    marvin_quat.setRPY(0.0, 0.0, yaw);
    tf::quaternionTFToMsg(marvin_quat, marvin_spawn_pose.pose.pose.orientation);

    marvin_spawn_pose.pose.covariance[6*0+0] = 0.5 * 0.5;
    marvin_spawn_pose.pose.covariance[6*1+1] = 0.5 * 0.5;
    marvin_spawn_pose.pose.covariance[6*5+5] = M_PI/12.0 * M_PI/12.0;
  }
  else if(name=="roberto"){
    roberto_spawn_pose.header.frame_id = "roberto/level_mux_map";
    roberto_spawn_pose.header.stamp = ros::Time::now();

    roberto_spawn_pose.pose.pose.position.x = x;
    roberto_spawn_pose.pose.pose.position.y = y;
    roberto_spawn_pose.pose.pose.position.z = 0.0;
    tf::Quaternion roberto_quat;
    roberto_quat.setRPY(0.0, 0.0, yaw);
    tf::quaternionTFToMsg(roberto_quat, roberto_spawn_pose.pose.pose.orientation);

    roberto_spawn_pose.pose.covariance[6*0+0] = 0.5 * 0.5;
    roberto_spawn_pose.pose.covariance[6*1+1] = 0.5 * 0.5;
    roberto_spawn_pose.pose.covariance[6*5+5] = M_PI/12.0 * M_PI/12.0;
  }
  else{
    ROS_INFO_STREAM("Available Options: [roberto marvin]");
  }
};
bool MRH_Experiment::reset_environment(){
  std_srvs::Empty clear;

  // Teleport robots to spawn pose
  bwi_msgs::RobotTeleporterInterface roberto_rti;
  roberto_rti.request.pose = roberto_spawn_pose.pose.pose;
  roberto_teleport.call(roberto_rti);
  bwi_msgs::RobotTeleporterInterface marvin_rti;
  marvin_rti.request.pose = marvin_spawn_pose.pose.pose;
  marvin_teleport.call(marvin_rti);
	// Wait for teleport to complete.
  ros::Duration(0.5).sleep();// Always NEED TIME!
  // Publish current position to localization node
  roberto_spawn_pose.header.stamp = ros::Time::now();
  roberto_localizer.publish(roberto_spawn_pose);
  roberto_clear_costmap.call(clear);
  marvin_spawn_pose.header.stamp = ros::Time::now();
  marvin_localizer.publish(marvin_spawn_pose);
  marvin_clear_costmap.call(clear);

  // Further localize marvin by moving it to initial position
  roberto_ac.sendGoal(roberto_init_pose);
  marvin_ac.sendGoal(marvin_init_pose);

	// wait until both robot reaches experiment start position.
	ros::Rate r(10);
	int timeout = 0;
	while(ros::ok()){
		if(roberto_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED &&
	  marvin_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
			break;
		}
		r.sleep();
		if(roberto_ac.getState() == actionlib::SimpleClientGoalState::ABORTED ||
	  marvin_ac.getState() == actionlib::SimpleClientGoalState::ABORTED){
			return false;
		}
		timeout = timeout+1;
		if(timeout>300){
			ROS_INFO_STREAM("Reset environment Timeout");
			return false;
		}
	}
  roberto_clear_costmap.call(clear);
  marvin_clear_costmap.call(clear);
	return true;
};
void MRH_Experiment::move(string robot_name, move_base_msgs::MoveBaseGoal goal){
	if(robot_name == "roberto") roberto_ac.sendGoal(goal);
	else if(robot_name == "marvin") marvin_ac.sendGoal(goal);
	else ROS_INFO_STREAM("Available robots are [roberto, marvin]");
};
float MRH_Experiment::dist(geometry_msgs::Point my_plan_pt, geometry_msgs::Point others_plan_pt){
  return hypot(my_plan_pt.x - others_plan_pt.x, my_plan_pt.y - others_plan_pt.y);
};
bool MRH_Experiment::checkCollision(){
	// subscribe Gazebo contact topic to decide collision.
	if(collision_check == true){
		collision_check = false;
		return true;
	}
	return false;
};
vector<double> MRH_Experiment::run_raw_experiment(int timeout, int delay=0.0){
	// This experiment is the baseline experiments
	// Two robot will simply go to the others location with vanila navigation stack
	// return how long it took for them to passby
	// delay: How much delay for marvin to wait after roberto moves.
	vector<double> endtime;
	endtime.push_back(-1.0);
	endtime.push_back(-1.0);

	//-----------------------------------------------------------------------------------
	// Prof. Joydeep's suggestion. Use buffer waypoint before experiment.
	/*
	move_base_msgs::MoveBaseGoal marvin_wp;
	marvin_wp.target_pose.header.frame_id = "marvin/level_mux_map";
	marvin_wp.target_pose.header.stamp = ros::Time::now();

	marvin_wp.target_pose.pose.position.x = marvin_init_pose.target_pose.pose.position.x;
	marvin_wp.target_pose.pose.position.y = marvin_init_pose.target_pose.pose.position.y;
	tf::Quaternion marvin_quat;
	marvin_quat.setRPY(0.0, 0.0, PI/180. * -20);
	tf::quaternionTFToMsg(marvin_quat, marvin_wp.target_pose.pose.orientation);

	marvin_ac.sendGoalAndWait(marvin_wp);
	*/
	//------------------------------------------------------------------------------------
	//Check Markovity ---------------- Additional experiments
	//marvin_ac.sendGoalAndWait(this->marvin_init_pose);
	//------------------------------------------------------------------------------------
	this->start_time = ros::Time::now();

	this->move("roberto", roberto_goal_pose);
	this->move("marvin", marvin_goal_pose);

	int t = 0;
	bool roberto_check = true;
	bool marvin_check = true;
	while(ros::ok()){
		// check collision between two robots
		if(this->checkCollision()){
			endtime[0] = endtime[1] = 1000.0 + (ros::Time::now() - this->start_time).toSec();;
		}
		// if success, return how long it took
		if(roberto_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && roberto_check){
			roberto_check = false;
			endtime[0] = (ros::Time::now() - this->start_time).toSec();
			this->move("roberto", roberto_standby_pose);
		}
		if(marvin_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && marvin_check){
			marvin_check = false;
			endtime[1] = (ros::Time::now() - this->start_time).toSec();
			this->move("marvin", marvin_standby_pose);
		}
		if(roberto_ac.getState() == actionlib::SimpleClientGoalState::ABORTED){
			ROS_INFO_STREAM("roberto aborted. RESUME");
			this->move("roberto", roberto_goal_pose);
		}
		if(marvin_ac.getState() == actionlib::SimpleClientGoalState::ABORTED){
			ROS_INFO_STREAM("marvin  aborted. RESUME");
			this->move("marvin", marvin_goal_pose);
		}
		// if timeout, return timeout value
		if(t > timeout * 10){
			if(endtime[0]<0) endtime[0] = double(timeout);
			if(endtime[1]<0) endtime[1] = double(timeout);
		}
		t += 1;
		if(endtime[0]>0 && endtime[1]>0) break;
		this->rate->sleep();
	}
	ofstream log_data;
	std::string path = ros::package::getPath("multi_robot_collision_avoidance");

	log_data.open(path + "/script/data/8m_contextual.txt", ios::out | ios::app);
	log_data<<-endtime[0]<<","<<-endtime[1]<<","<<-endtime[0]-endtime[1]<<endl;
	return endtime;
};
vector<double> MRH_Experiment::run_chicken_experiment(int timeout){
	// This experiment is the experiment with baseline algorithm
	// One bold robot(roberto) will go to it's destination with vanila navigation stack
	// Other chicken robot(marvin) will run away to the pre-defined parking spot when they detect potential collision.
	// return how long it took for them to passby
	vector<double> endtime;
	endtime.push_back(-1.0);
	endtime.push_back(-1.0);
	// Activate safety protocol
	ChickenSafetyProtocol csp("/marvin", "/jinsoo");
	ros::Duration(0.5).sleep();

	int t = 0;
	bool roberto_check = true;
	bool marvin_check = true;

	bool danger = false;
	bool wait   = false;
	bool resume = false;
	string parkingZone;

	// Set both robots to go to their goal position
	this->move("roberto", roberto_goal_pose);// bold robot
	this->move("marvin", marvin_goal_pose); // chicken robot
	this->start_time = ros::Time::now();
	while(ros::ok()){
		if(wait == false){
			bool danger = csp.distanceBaseAlarm(0.6);
			if(danger == true){
				marvin_ac.cancelGoal();
				parkingZone = csp.findSafeZone();
				goToParkingZone(parkingZone);

				wait=true;
				this->rate->sleep();
			}
		}
		if(wait == true && csp.waitUntilSafe()){
			if(marvin_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
				wait = false;
				this->move("marvin", marvin_goal_pose);
			}
		}
		// check collision between two robots
		if(this->checkCollision()){
			endtime[0] = endtime[1] = 1000.0 + (ros::Time::now() - this->start_time).toSec();
			return endtime;
		}
		// if success, return how long it took
		if(roberto_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && roberto_check){
			roberto_check = false;
			endtime[0] = (ros::Time::now() - this->start_time).toSec();

			// For reliable detection of chicken bot, move further.
			move_base_msgs::MoveBaseGoal tmp_goal_pose;
	    tmp_goal_pose.target_pose.header.frame_id = "roberto/level_mux_map";
	    tmp_goal_pose.target_pose.header.stamp = ros::Time::now();

	    tmp_goal_pose.target_pose.pose.position.x = 0.00;
	    tmp_goal_pose.target_pose.pose.position.y = 14.00;
	    tf::Quaternion tmp_quat;
	    tmp_quat.setRPY(0.0, 0.0, -PI/2);
	    tf::quaternionTFToMsg(tmp_quat, tmp_goal_pose.target_pose.pose.orientation);

			this->move("roberto", tmp_goal_pose);
		}
		if(marvin_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && marvin_check && wait == false){
			marvin_check = false;
			endtime[1] = (ros::Time::now() - this->start_time).toSec();
		}
		if(roberto_ac.getState() == actionlib::SimpleClientGoalState::ABORTED){
			ROS_INFO_STREAM("roberto aborted. RESUME");
			this->move("roberto", roberto_goal_pose);
		}
		if(marvin_ac.getState() == actionlib::SimpleClientGoalState::ABORTED){
			ROS_INFO_STREAM("marvin  aborted. RESUME");
			this->move("marvin", marvin_goal_pose);
		}
		// if timeout, return timeout value
		if(t > timeout * 10){
			if(endtime[0]<0) endtime[0] = double(timeout);
			if(endtime[1]<0) endtime[1] = double(timeout);
		}
		t += 1;
		if(endtime[0]>0 && endtime[1]>0) break;
		this->rate->sleep();
	}
	ofstream log_data;
	std::string path = ros::package::getPath("multi_robot_collision_avoidance");

	log_data.open(path + "/script/data/8m_contextual.txt", ios::out | ios::app);
	log_data<<-endtime[0]<<","<<-endtime[1]<<","<<-endtime[0]-endtime[1]<<endl;
	return endtime;
};
vector<double> MRH_Experiment::run_siren_experiment(int timeout){
	// This experiment is the experiment with baseline algorithm
	// One bold robot(roberto) will go to it's destination with vanila navigation stack
	// Other chicken robot(marvin) will run away to the pre-defined parking spot when they detect potential collision.
	// return how long it took for them to passby
	vector<double> endtime;
	endtime.push_back(-1.0);
	endtime.push_back(-1.0);
	// Activate safety protocol
	ChickenSafetyProtocol csp("/marvin", "/jinsoo");
	ros::Duration(0.5).sleep();

	int t = 0;
	bool roberto_check = true;
	bool marvin_check = true;

	bool danger = false;
	bool wait   = false;
	bool resume = false;
	bool firstContact = true;
	string parkingZone;

	// Set both robots to go to their goal position
	this->move("roberto", roberto_goal_pose);// bold robot
	this->move("marvin", marvin_goal_pose); // chicken robot
	this->start_time = ros::Time::now();
	while(ros::ok()){
		if(wait == false){
			bool danger = csp.distanceBaseAlarm(0.6);
			if(danger == true && firstContact == true){
				firstContact = false;
				marvin_ac.cancelGoal();

				multi_robot_collision_avoidance::EvalWaypoint srv;
				srv.request.chicken_pose = marvinPlan_[0];
				srv.request.bold_pose = robertoPlan_[0];

				move_base_msgs::MoveBaseGoal wp;
				if (siren_find.call(srv))
			  {
			    wp.target_pose.header.frame_id = "marvin/level_mux_map";
			    wp.target_pose.header.stamp = ros::Time::now();

			    wp.target_pose.pose.position.x = srv.response.waypoint.position.x;
			    wp.target_pose.pose.position.y = srv.response.waypoint.position.y;
//					wp.target_pose.pose.orientation = srv.response.waypoint.orientation;
			    wp.target_pose.pose.orientation = marvin_goal_pose.target_pose.pose.orientation;
			  }
			  else
			  {
			    ROS_ERROR("Failed to call service eval_waypoint");
			  }

				this->marvin_ac.sendGoal(wp);

				wait=true;
			}
		}
		if(wait == true && csp.waitUntilSafe()){
			if(marvin_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
				wait = false;
				this->move("marvin", marvin_goal_pose);
			}
		}
		// check collision between two robots
		if(this->checkCollision()){
			endtime[0] = endtime[1] = 1000.0 + (ros::Time::now() - this->start_time).toSec();
			break;
		}
		// if success, return how long it took
		if(roberto_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && roberto_check){
			roberto_check = false;
			endtime[0] = (ros::Time::now() - this->start_time).toSec();
			this->move("roberto", roberto_standby_pose);
		}
		if(marvin_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && marvin_check && wait == false){
			marvin_check = false;
			endtime[1] = (ros::Time::now() - this->start_time).toSec();
			this->move("marvin", marvin_standby_pose);
		}
		if(roberto_ac.getState() == actionlib::SimpleClientGoalState::ABORTED){
			ROS_INFO_STREAM("roberto aborted. RESUME");
			this->move("roberto", roberto_goal_pose);
		}
		if(marvin_ac.getState() == actionlib::SimpleClientGoalState::ABORTED){
			ROS_INFO_STREAM("marvin  aborted. RESUME");
			this->move("marvin", marvin_goal_pose);
		}
		// if timeout, return timeout value
		if(t > timeout * 10){
			if(endtime[0]<0) endtime[0] = double(timeout);
			if(endtime[1]<0) endtime[1] = double(timeout);
		}
		t += 1;
		if(endtime[0]>0 && endtime[1]>0) break;
		this->rate->sleep();
	}
	if(firstContact == true){
		episode_fail = true;
		return endtime;
	}
	ofstream log_data;
	std::string path = ros::package::getPath("multi_robot_collision_avoidance");

	log_data.open(path + "/script/data/8m_contextual.txt", ios::out | ios::app);
	log_data<<-endtime[0]<<","<<-endtime[1]<<","<<-endtime[0]-endtime[1]<<endl;

	return endtime;
};
void MRH_Experiment::go(vector<float> coord){
  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "marvin/level_mux_map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = coord[0];
  goal.target_pose.pose.position.y = coord[1];
  tf::Quaternion quat;
  quat.setRPY(0.0, 0.0, coord[2]);
  tf::quaternionTFToMsg(quat, goal.target_pose.pose.orientation);
	this->marvin_ac.sendGoal(goal);
};
void MRH_Experiment::goToParkingZone(string location){
  vector<float> coord;
  if(location=="p3_15"){
    coord.push_back(3.627);
    coord.push_back(10.41);
    coord.push_back(-PI/2);
    go(coord);
  }
  else if(location=="p3_16"){
    coord.push_back(5.65);
    coord.push_back(10.46);
    coord.push_back(-PI/2);
    go(coord);
  }
  else if(location=="p3_18"){
    coord.push_back(17.922947);
    coord.push_back(10.459691);
    coord.push_back(PI/2);
    go(coord);
  }
  else if(location=="p3_20"){
    coord.push_back(20.495056);
    coord.push_back(8.198261);
    coord.push_back(-PI/2);
    go(coord);
  }
  else if(location=="p3_30"){
    coord.push_back(32.467041);
    coord.push_back(8.474984);
    coord.push_back(-PI/2);
    go(coord);
  }
  else if(location=="p3_31"){
    coord.push_back(39.029182);
    coord.push_back(8.375641);
    coord.push_back(-PI/2);
    go(coord);
  }
  else if(location=="p3_44"){
    coord.push_back(23.564869);
    coord.push_back(19.213001);
    coord.push_back(-PI/2);
    go(coord);
  }
  else if(location=="p3_45"){
    coord.push_back(3.834564);
    coord.push_back(19.154118);
    coord.push_back(-PI/2);
    go(coord);
  }
  else{
    ROS_INFO_STREAM(location << " not understood");
  }
};

void cb(ConstContactsPtr &_msg){
  string delimiter = "::";
  for(int i=0; i< _msg->contact_size(); ++i){
    string contact = _msg->contact(i).collision2();
    contact = contact.substr(0, contact.find(delimiter));
		if(contact != "plane1_model"){
			collision_check = true;
			return;
		}
  }
}

int main(int argc, char** argv){
	if(argc != 3){
		ROS_INFO_STREAM("How to use:");
		ROS_INFO_STREAM("rosrun multi_robot_navigation exp_handler ${exp_type} ${num_exp}");
		ROS_INFO_STREAM("exp_type: [raw chicken]");
		exit(0);
		ROS_INFO_STREAM("rosrun multi_robot_navigation exp_handler ${exp_type} ${num_exp} ${time_out}");
	}

	std::random_device rd;
	std:mt19937 e2(rd());
	std::uniform_real_distribution<> xrand(-4,4);
	std::uniform_real_distribution<> yrand(-0.3, 0.3);
	std::uniform_real_distribution<> wrand(-PI/12, PI/12);

	// Read Gazebo contact topic to decide collision.
	gazebo::client::setup(argc, argv);
	gazebo::transport::NodePtr node(new gazebo::transport::Node());
	node->Init();
	gazebo::transport::SubscriberPtr sub = node->Subscribe("/gazebo/default/physics/contacts", cb);

  ros::init(argc, argv, "environment_reset_test");
  ros::NodeHandle nh;

	ofstream outfile;
	string filename;
	string exp_name = argv[1];

	MRH_Experiment eh(&nh);
  // Set initial environment for expermients
  // Hallway No 1.
/*
	eh.setSpawnPose("marvin", 8.8, 17.85, PI);
	eh.setInitPose("marvin", 12.5, 17.85, PI/180.*0.); // Test: -PI/12 0 PI/12  150 each
	eh.setGoalPose("marvin", 28.5, 17.85, PI/180.*0.);
	eh.setStandbyPose("marvin", 30.45, 17.85, 0.0);
	eh.setSpawnPose("roberto", 26.8, 17.85, 0.0);
	eh.setInitPose("roberto", 22.5, 17.85, PI);
	eh.setGoalPose("roberto", 6.5, 17.85, PI);
	eh.setStandbyPose("roberto", 2.15, 17.855, PI);
*/
	eh.setSpawnPose("roberto", 8.8, 17.85, PI);
	eh.setInitPose("roberto", 12.5, 17.85, PI/180.*0.); // Test: -PI/12 0 PI/12  150 each
	eh.setGoalPose("roberto", 28.5, 17.85, PI/180.*0.);
	eh.setStandbyPose("roberto", 30.45, 17.85, 0.0);
	eh.setSpawnPose("marvin", 26.8, 17.85, 0.0);
	eh.setInitPose("marvin", 22.5, 17.85, PI);
	eh.setGoalPose("marvin", 6.5, 17.85, PI);
	eh.setStandbyPose("marvin", 2.15, 17.855, PI);
/*
	eh.setSpawnPose("marvin", 7.5, 9.5, PI);
	eh.setInitPose("marvin", 10.5, 9.5, PI/180.*0.); // Test: -PI/12 0 PI/12  150 each
	eh.setGoalPose("marvin", 22.5, 9.5, PI/180.*0.);
	eh.setStandbyPose("marvin", 30.5, 9.5, 0.0);
	eh.setSpawnPose("roberto", 23.5, 9.5, 0.0);
	eh.setInitPose("roberto", 20.5, 9.5, PI);
	eh.setGoalPose("roberto", 3.5, 9.5, PI);
	eh.setStandbyPose("roberto", 0., 9.5, PI);
*//*
	eh.setSpawnPose("marvin", 29.5, 9.5, PI);
	eh.setInitPose("marvin", 33.0, 9.5, PI/180.*0.); // Test: -PI/12 0 PI/12  150 each
	eh.setGoalPose("marvin", 39.0, 15.5, PI/180.*90.);
	eh.setStandbyPose("marvin", 39.3, 17.5, 0.0);
	eh.setSpawnPose("roberto", 39.3, 17.5, PI/2);
	eh.setInitPose("roberto", 39.3, 13.5, -PI/2);
	eh.setGoalPose("roberto", 31.0, 9.5, PI);
	eh.setStandbyPose("roberto", 20.0, 9.5, PI);
*/
  //eh.showInitPose();
	float delay = 0.0;
	//Set experiments here
	double reset_time, xdiff,ydiff,wdiff;
	for(int i=0; i<atoi(argv[2]); i++){
		if(ros::ok()!=true) break;
		// -4 on x
		xdiff = xrand(e2);
		ydiff = yrand(e2);
		wdiff = wrand(e2);
	  // Hallway No 1.
		eh.setSpawnPose("marvin", 8.8, 17.85, PI);
		eh.setInitPose("marvin", 12.5, 17.85+yrand(e2), 0 + wrand(e2)); // Test: -PI/12 0 PI/12  150 each
		eh.setGoalPose("marvin", 28.5, 17.85+yrand(e2), 0 + wrand(e2));
		eh.setStandbyPose("marvin", 28.5, 17.85, 0.0);
//		eh.setStandbyPose("marvin", 30.45, 17.85, 0.0);
		eh.setSpawnPose("roberto", 26.8, 17.85, 0.0);
		eh.setInitPose("roberto", 22.5, 17.85+yrand(e2), PI + wrand(e2));
		eh.setGoalPose("roberto", 6.5, 17.85+yrand(e2), 3.13);
		eh.setStandbyPose("roberto", 6.5, 17.85, 3.13);
//		eh.setStandbyPose("roberto", 2.15, 17.855, PI);

		ros::Time start_time = ros::Time::now();
	  bool success = eh.reset_environment();
		// if reset fails, throw that experiment and reset again.
		if(!success){
			i = i-1;
			continue;
		}
		eh.checkCollision();
		ROS_INFO_STREAM((i+1)<<"th experiment");
		reset_time = (ros::Time::now() - start_time).toSec();
		vector<double> result;
		if(exp_name == "raw") result = eh.run_raw_experiment(200, delay);
		else if(exp_name == "chicken") result = eh.run_chicken_experiment(200);
		else if(exp_name == "siren"){
			result = eh.run_siren_experiment(200);
			if(episode_fail == true){
				episode_fail = false;
				i = i - 1;
				continue;
			}
		}
	}
	gazebo::client::shutdown();
}
