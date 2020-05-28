#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/bag.h>

#include <sensor_msgs/CompressedImage.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_srvs/Empty.h>

#include "VanillaPolicy.h"

using namespace std;

YAML::Node loadYaml(){
  string path = ros::package::getPath("multi_robot_navigation");
  string yamlPath = path + "/config/description.yaml";
  YAML::Node config = YAML::LoadFile(yamlPath);
  return config;
};

class Log{
private:
  string ROBOT_NAME;
  ros::NodeHandle nh_;
  ros::Subscriber image_sub;
  ros::Subscriber amcl_pose_sub;
  geometry_msgs::PoseStamped amcl;
  ros::AsyncSpinner *spinner;

  void imageCallBack(const sensor_msgs::CompressedImageConstPtr& msg){
    if(this->write == true){
      this->amcl.header.stamp = ros::Time::now();
      bag.write("/image", ros::Time::now(), msg);
      bag.write("/amcl_pose", ros::Time::now(), this->amcl);
    }
  };
  void amclCallBack(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
    this->amcl.header = msg->header;
    this->amcl.pose = msg->pose.pose;
  }

  bool write;
  rosbag::Bag bag;
public:
  Log(ros::NodeHandle* nh, string name):nh_(*nh), ROBOT_NAME(name){
    this->write = false;
    ROS_INFO_STREAM("Initialize");
    string topic_name = (ROBOT_NAME==""?"":"/"+ROBOT_NAME);
    ROS_INFO_STREAM(topic_name);
    image_sub = this->nh_.subscribe(topic_name + "/nav_kinect/rgb/image_raw/compressed", 1, &Log::imageCallBack, this);
    amcl_pose_sub = this->nh_.subscribe(topic_name + "/amcl_pose", 1, &Log::amclCallBack, this);
    this->spinner = new ros::AsyncSpinner(2);
    this->spinner->start();
  }

  void start(int num_episode, string name){
    string pkgPath = ros::package::getPath("multi_robot_navigation");
    string dataPath = pkgPath + "/result/";
    string filePath = dataPath + name + "episode_" + to_string(num_episode) + ".bag";

    bag.open(filePath, rosbag::bagmode::Write);
    write = true;
  }
  void end(){
    this->write = false;
    bag.close();
  }
};

int main(int _argc, char** _argv){
  ros::init(_argc, _argv, "real_experiment12");
  ros::NodeHandle nh;
  ros::Rate r(10);

  YAML::Node params = loadYaml();

  bool verbose = params["verbose"].as<bool>();
  int num_episode = params["Num_episode"].as<int>();
  string name = params["Robot_name"].as<string>();
  string policy = params["Policy"].as<string>();
  string platform = params["Platform"].as<string>();


  string topic_prefix;
  if(platform == "real"){
    topic_prefix = "";
    policy = "vanilla";
  }
  else{
    if(name == "bold") topic_prefix = "roberto";
    else topic_prefix = "marvin";
  }
  Log log(&nh, topic_prefix);
  VanillaPolicy agent(&nh, topic_prefix);

  ros::ServiceClient clear_costmap = nh.serviceClient<std_srvs::Empty>(topic_prefix + "/move_base/clear_costmaps");
  std_srvs::Empty clear;

  vector<float> init_pose, goal_pose, temp_pose;
  vector<float> check_init_pose, check_goal_pose;

  if(name == "bold"){
    init_pose = params["Bold"]["init_pose"].as<vector<float>>();
    goal_pose = params["Bold"]["goal_pose"].as<vector<float>>();
    temp_pose = params["Bold"]["temp_pose"].as<vector<float>>();
    check_init_pose = params["Coward"]["init_pose"].as<vector<float>>();
    check_goal_pose = params["Coward"]["goal_pose"].as<vector<float>>();
  }
  else{
    init_pose = params["Coward"]["init_pose"].as<vector<float>>();
    goal_pose = params["Coward"]["goal_pose"].as<vector<float>>();
    temp_pose = params["Coward"]["temp_pose"].as<vector<float>>();
    check_init_pose = params["Bold"]["init_pose"].as<vector<float>>();
    check_goal_pose = params["Bold"]["goal_pose"].as<vector<float>>();
  }
  if(verbose == true){
    ROS_INFO_STREAM("Number of Episodes: " + to_string(num_episode));
    ROS_INFO_STREAM("Robot Name        : " + name);
    ROS_INFO_STREAM("Policy            : " + policy);
    ROS_INFO_STREAM("Platform          : " + platform);
  }

  for(int i=0; i<num_episode; i++){
    // Go to init_pose
    agent.move( agent.coord2Pose(init_pose) );
    // Wait Until other robot reaches check_init_pose
    ROS_INFO_STREAM("Check 2");
    while(ros::ok() && (agent.is_other_arrive(check_init_pose) == false) ){
      r.sleep();
    }
    // Episode Start.
    // Go to Goal pose
    agent.move( agent.coord2Pose(goal_pose) );
    log.start(i+1, name + "_" + policy + "_");
    ROS_INFO_STREAM("Check 3");
    // Do proper reaction
    while(ros::ok()){
      if(agent.isArrive() == true){break;}
      if(agent.isAborted() == true){
        agent.move( agent.coord2Pose(goal_pose) );
      }
      if(policy == "vanilla"){}
      else if(policy == "chicken"){
        agent.safety_chicken();
      }
      else if(policy == "obey"){
        agent.safety_obey();
      }
    }
    ROS_INFO_STREAM("Check 4");
    // log finish
    log.end();
    // Go to temp pose
    agent.move( agent.coord2Pose(temp_pose) );
    // reset costmap
    clear_costmap.call(clear);
  }
}
