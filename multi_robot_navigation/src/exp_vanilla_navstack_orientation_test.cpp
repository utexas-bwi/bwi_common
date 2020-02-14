// Modules for client
#include "plan_execution/ExecutePlanAction.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
// Modules for msg
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>

#include <visualization_msgs/MarkerArray.h>
// Teleport interface
#include <bwi_msgs/RobotTeleporterInterface.h>

// Safety protocol
#include "ChickenSafety.h"
#include "multi_robot_collision_avoidance/EvalWaypoint.h"

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <ros/ros.h>

// Write simulation results in file.
#include <sys/stat.h>
#include <fstream>

#define PI (3.1415926)
typedef actionlib::SimpleActionClient<plan_execution::ExecutePlanAction> Client;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace std;

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
	float                       robertoW_;
	float                       robertoX_;
	float                       robertoY_;

  vector<geometry_msgs::Pose> marvinPlan_;
	float                       marvinVel_;
	float                       marvinW_;
	float                       marvinX_;
	float                       marvinY_;

  ros::Subscriber subscribeRobertoPlan_;
  ros::Subscriber subscribeMarvinPlan_;

	ros::Subscriber subscribeMarvinVel_;

	ros::Subscriber subscribeRobertoOdom_;
	ros::Subscriber subscribeMarvinOdom_;

	ros::AsyncSpinner *spinner;
	void subscribeRobertoPlan(const visualization_msgs::MarkerArray plan);
	void subscribeMarvinPlan(const visualization_msgs::MarkerArray plan);
	void subscribeRobertoOdom(const nav_msgs::Odometry odom);
	void subscribeMarvinOdom(const nav_msgs::Odometry odom);
	void subscribeMarvinCmdvel(const geometry_msgs::Twist vel);

	ros::ServiceClient siren_find;
public:
  MRH_Experiment():marvin_ac(""),roberto_ac(""){}
  MRH_Experiment(ros::NodeHandle* nh);
	// Reset environments
  void setSpawnPose(string name, float x, float y, float yaw);
  void setInitPose(string name, float x, float y, float yaw);
	void setGoalPose(string name, float x, float y, float yaw);
	void setStandbyPose(string name, float x, float y, float yaw);

  bool reset_environment();
	// basic fuctions
	void move(string robot_name, move_base_msgs::MoveBaseGoal goal);
	float dist(geometry_msgs::Point my_plan_pt, geometry_msgs::Point others_plan_pt);
	bool checkCollision();
	// Experiments
	// without safety protocol
	vector<double> run_raw_experiment(int timeout, ofstream& recRoberto, ofstream& recMarvin, int delay);
};

MRH_Experiment::MRH_Experiment(ros::NodeHandle* nh):marvin_ac("marvin/move_base"),roberto_ac("roberto/move_base")
{
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
	this->subscribeMarvinPlan_  = nh_->subscribe(othersEBandTopic, 1, &MRH_Experiment::subscribeMarvinPlan, this);
	this->subscribeMarvinVel_   = nh_->subscribe("/marvin/cmd_vel", 1, &MRH_Experiment::subscribeMarvinCmdvel, this);
	this->subscribeRobertoOdom_  = nh_->subscribe("/roberto/odom", 1, &MRH_Experiment::subscribeRobertoOdom, this);
	this->subscribeMarvinOdom_  = nh_->subscribe("/marvin/odom", 1, &MRH_Experiment::subscribeMarvinOdom, this);
	this->spinner = new ros::AsyncSpinner(5);
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
void MRH_Experiment::subscribeMarvinCmdvel(const geometry_msgs::Twist vel){
	float linear_velocity = vel.linear.x + vel.linear.y + vel.linear.z;
	this->marvinVel_ = linear_velocity;
};
void MRH_Experiment::subscribeRobertoOdom(const nav_msgs::Odometry odom){
	tf::Quaternion quat;
	tf::quaternionMsgToTF(odom.pose.pose.orientation, quat);
	double roll, pitch, yaw;
	tf::Matrix3x3 m(quat);
	m.getRPY(roll, pitch, yaw);
	this->robertoX_ = float(odom.pose.pose.position.x);
	this->robertoY_ = float(odom.pose.pose.position.y);
	this->robertoW_ = float(yaw);
};
void MRH_Experiment::subscribeMarvinOdom(const nav_msgs::Odometry odom){
	tf::Quaternion quat;
	tf::quaternionMsgToTF(odom.pose.pose.orientation, quat);
	double roll, pitch, yaw;
	tf::Matrix3x3 m(quat);
	m.getRPY(roll, pitch, yaw);
	this->marvinX_ = float(odom.pose.pose.position.x);
	this->marvinY_ = float(odom.pose.pose.position.y);
	this->marvinW_ = float(yaw);
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
		if(timeout>200){
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
	// subscribe two local_plan topic
	// check distance between localplan[0]
	// if dist < 0.7, collision occur. return True;
	if(this->robertoPlan_.size() && this->marvinPlan_.size()){
		if(dist(this->robertoPlan_[0].position, this->marvinPlan_[0].position) < 0.7){
			return true;
		}
	}
	return false;
};

vector<double> MRH_Experiment::run_raw_experiment(int timeout, ofstream& recRoberto, ofstream& recMarvin, int delay=0.0){
	// This experiment is the baseline experiments
	// Two robot will simply go to the others location with vanila navigation stack
	// return how long it took for them to passby
	// delay: How much delay for marvin to wait after roberto moves.
	vector<double> endtime;
	ROS_INFO_STREAM("Record initial pose");
	endtime.push_back(-1.0);           // Roberto reward
	endtime.push_back(-1.0);           // Marvin  reward
	endtime.push_back(this->robertoW_);// Roberto initial orientation [rad]
	endtime.push_back(this->marvinW_); // Marvin initial orientation [rad]
	endtime.push_back(-1000);          // Impact orientation (Roberto)
	endtime.push_back(-1000);          // Impact orientation (Marvin)

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
	//------------------------------------------------------------------------------------
	//Check Markovity ---------------- Additional experiments
	marvin_ac.sendGoalAndWait(this->marvin_init_pose);
	//------------------------------------------------------------------------------------
	*/
	this->start_time = ros::Time::now();
	if(delay == 0){
		this->move("marvin", marvin_goal_pose);
		this->move("roberto", roberto_goal_pose);
	}
	else if(delay<0){
		this->move("marvin", marvin_goal_pose);
		ros::Duration(delay).sleep();
		this->move("roberto", roberto_goal_pose);
	}
	else{
		this->move("roberto", roberto_goal_pose);
		ros::Duration(delay).sleep();
		this->move("marvin", marvin_goal_pose);
	}

	float prev_vel = 0;
	bool initialized = false;
	bool first_impact = true;

	bool before_roberto_touch_down = true;
	bool before_marvin_touch_down = true;
	while(ros::ok()){
		// grab impact orientation
		if(first_impact){
			if(initialized){
				if(this->marvinVel_ == 0){
					ROS_INFO_STREAM("IMPACT!");
					endtime[4] = this->robertoW_;
					endtime[5] = this->marvinW_;
					first_impact = false;
				}
			}
			else{
				if(prev_vel == 0 && this->marvinVel_!=0){
					initialized = true;
				}
				prev_vel = this->marvinVel_;
			}
		}
		if(before_roberto_touch_down == true){
			recRoberto<<this->robertoX_<<","<<this->robertoY_<<",";
		}
		if(before_marvin_touch_down == true){
			recMarvin <<this->marvinX_<<","<<this->marvinY_<<",";
		}
		// check collision between two robots
		if(this->checkCollision()){
			endtime[0] = endtime[1] = 1000.0 + (ros::Time::now() - this->start_time).toSec();
			return endtime;
		}

		// if success, return how long it took
		if(roberto_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && before_roberto_touch_down == true){
			before_roberto_touch_down = false;
			endtime[0] = (ros::Time::now() - this->start_time).toSec();
			ROS_INFO_STREAM("roberto touch down");
			this->move("roberto", roberto_standby_pose);
		}
		if(marvin_ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && before_marvin_touch_down == true){
			before_marvin_touch_down = false;
			endtime[1] = (ros::Time::now() - this->start_time).toSec();
			ROS_INFO_STREAM("marvin touch down");
			this->move("marvin", marvin_standby_pose);
		}
		if(roberto_ac.getState() == actionlib::SimpleClientGoalState::ABORTED){
			//ROS_INFO_STREAM("roberto aborted. RESUME");
			//this->move("roberto", roberto_goal_pose);
			this->move("roberto", roberto_goal_pose);
		}
		if(marvin_ac.getState() == actionlib::SimpleClientGoalState::ABORTED){
			//ROS_INFO_STREAM("marvin  aborted. RESUME");
			//this->move("marvin", marvin_goal_pose);
			this->move("marvin", marvin_goal_pose);
		}
		// if timeout, return timeout value
		if( (ros::Time::now() - this->start_time).toSec() > timeout ){
			if(endtime[0]<0) endtime[0] = double(timeout);
			if(endtime[1]<0) endtime[1] = double(timeout);
		}

		if(endtime[0]>0 && endtime[1]>0) break;
		this->rate->sleep();
	}
	recRoberto << endl;
	recMarvin << endl;
	return endtime;
};

int main(int argc, char** argv){
	// make data storing directory
	system("mkdir -p ~/Desktop/simulation_EXP/trajectory");

	// read arguments
	if(argc != 4){
		ROS_INFO_STREAM("How to use:");
		ROS_INFO_STREAM("rosrun multi_robot_navigation exp_handler ${exp_type} ${exp_start} ${exp_end}");
		ROS_INFO_STREAM("exp_type: [raw chicken]");
		exit(0);
	}
  ros::init(argc, argv, "vanilla_impact_orientation_test");
  ros::NodeHandle nh;

	ofstream outfile;
	ofstream record_Marvin;
	ofstream record_Roberto;

	string exp_name = argv[1];
	if(!(exp_name == "raw" || exp_name == "chicken" || exp_name == "siren")){
		ROS_INFO_STREAM("Cannot understand experiments type: " << exp_name<<endl<<"Please use [raw chicken siren]");
		exit(0);
	}
  MRH_Experiment eh(&nh);
  // Set initial environment for expermients
	eh.setSpawnPose("marvin", 4.8, 18.1, 0);//PI);
  eh.setInitPose("marvin", 8.8, 18.1, PI/180.*0.); // Test: -PI/12 0 PI/12  150 each
	eh.setGoalPose("marvin", 19.8, 18.1, 0.0);
	eh.setStandbyPose("marvin", 26.45, 18.15, 0.0);

	eh.setSpawnPose("roberto", 23.8, 18.1, PI);//0.0);
  eh.setInitPose("roberto", 19.8, 18.1, PI);
	eh.setGoalPose("roberto", 8.8, 18.1, PI);
	eh.setStandbyPose("roberto", 2.15, 18.15, PI);
	//Set experiments here
	int num_exp = 150;
	float exp_degree = 15;
	float delay = 0;
	double reset_time;
	bool new_file = true;

	for(int i=atoi(argv[2]); i<atoi(argv[3]); i++){
		// Generate text file to store experiment results.
		if(i%num_exp == 0 && new_file == true){
			float angle = (int(i/num_exp) - (float(atoi(argv[3]))/num_exp - 1)/2) * exp_degree;
			eh.setInitPose("marvin", 8.8, 18.1, PI/180. * angle);

			outfile.close();
			record_Roberto.close();
			record_Marvin.close();
			outfile.clear();
			record_Roberto.clear();
			record_Marvin.clear();

			ostringstream filename;
			filename << "/home/jinsoo/Desktop/simulation_EXP/"<<exp_name<<"_exp_"<<angle<<"_degree.txt";
			outfile.open(filename.str().c_str(),ios::out|ios::app);
			outfile << "reset_time,roberto_time,marvin_time,roberto_orientation,marvin_orientation,roberto_impact_degree,marvin_impact_degree"<<endl;

			filename.str("");
			filename.clear();
			filename << "/home/jinsoo/Desktop/simulation_EXP/trajectory/roberto_"<<exp_name<<"_exp_"<<angle<<"_degree_trajectory.txt";
			record_Roberto.open(filename.str().c_str(), ios::out|ios::app);

			filename.str("");
			filename.clear();
			filename << "/home/jinsoo/Desktop/simulation_EXP/trajectory/marvin_"<<exp_name<<"_exp_"<<angle<<"_degree_trajectory.txt";
			record_Marvin.open(filename.str().c_str(), ios::out|ios::app);
			new_file = false;
		}

		ros::Time start_time = ros::Time::now();
	  bool success = eh.reset_environment();
		// if reset fails, throw that experiment and reset again.
		if(!success){
			i = i-1;
			continue;
		}
		new_file = true;
		ROS_INFO_STREAM((i+1)<<"th experiment");
		reset_time = (ros::Time::now() - start_time).toSec();
		vector<double> result;
		if(exp_name == "raw") result = eh.run_raw_experiment(50, record_Roberto, record_Marvin, delay);


		outfile << reset_time << "," << result[0] << "," << result[1] << "," << result[2] <<"," <<result[3]<<","<<result[4]<<","<<result[5]<<","<< endl;
	}
}
