#include "VanillaPolicy.h"

// Define Class
VanillaPolicy::VanillaPolicy(ros::NodeHandle* nodehandle, std::string ROBOT_NAME = ""):
  nh_(*nodehandle),
  rate(100),
  client_ac_( (ROBOT_NAME=="")?"move_base":(ROBOT_NAME+"/move_base").c_str() )
{
  if( ROBOT_NAME == ""){
    this->PLATFORM = "real";
    this->frame_id_ = "level_mux_map";
  }
  else{
    this->myName = ROBOT_NAME;
    this->PLATFORM = "simulation";
    this->frame_id_ = ROBOT_NAME + "/level_mux_map";
  }
  this->loadParams();
  this->initializeSubscribers();

  //ROS_INFO_STREAM("Waiting for move_base of "<<ROBOT_NAME<<"...");
  if( this->client_ac_.waitForServer(ros::Duration(10)) == false){
    ROS_INFO_STREAM("move_base for "<<ROBOT_NAME<<" does not exists.");
    exit(1);
  }
  //ROS_INFO_STREAM("Connected to "<<ROBOT_NAME<<" move_base");
  ros::Duration(0.5).sleep();
  this->toWaypoint = false;

  //Chicken=========================================
  this->makePlanService_ = this->nh_.serviceClient<nav_msgs::GetPlan>(
    ROBOT_NAME + "/move_base/NavfnROS/make_plan"
  );
  this->readParkingZones();

  if(this->parkingZones_.size() == 0){
    ROS_INFO_STREAM("No predefined parking zones");
    exit(0);
  }

  bool exist = this->makePlanService_.waitForExistence(ros::Duration(30));
  if(exist == false){
    ROS_INFO_STREAM("Make Plan Service is not exitsts.");
    exit(0);
  }
  // OBEY======================================
  obey_find = this->nh_.serviceClient<multi_robot_collision_avoidance::EvalWaypoint>("eval_waypoint");
}

void VanillaPolicy::loadParams()
{
  this->storeNewGoal = true;

  this->nh_.param<bool>("/debug", this->DEBUG, false);
  this->nh_.param<int>("/prudence", this->PRUDENCE, 3);

  if (this->DEBUG == true){
    ROS_INFO_STREAM("List of Parameters");
    ROS_INFO_STREAM("Platform: " + this->PLATFORM);
    ROS_INFO_STREAM("Prudence: " + std::to_string(this->PRUDENCE));
  }
}
void VanillaPolicy::initializeSubscribers()
{
  std::string commonEBand = "/move_base/EBandPlannerROS/eband_visualization_array";

  std::string myEBandTopic;
  std::string othersEBandTopic;
  std::string myGoalTopic;

  if(this->PLATFORM == "simulation"){
    if(this->DEBUG) ROS_INFO_STREAM("Working with simulation");

    std::string otherName;

    if(this->myName == "marvin"){
      otherName = "roberto";
    }
    else{
      otherName = "marvin";
    }

    myEBandTopic = this->myName + commonEBand;
    othersEBandTopic = otherName + commonEBand;
    myGoalTopic = this->myName + "/move_base/goal";
  }
  else if(this->PLATFORM == "real"){
    if(this->DEBUG) ROS_INFO_STREAM("Working with real robot");
    myEBandTopic = commonEBand;
    othersEBandTopic = "/others" + commonEBand;
    myGoalTopic = "move_base/goal";
  }
  else{
    ROS_INFO_STREAM("Invalid option.");
    ROS_INFO_STREAM("Try one of [ simulation real ]");
  }
  this->subscribeMyLocalPlan_ = this->nh_.subscribe(
    myEBandTopic, 1, &VanillaPolicy::subscribeMyLocalPlan, this
  );
  this->subscribeMyGoal_ = this->nh_.subscribe(
    myGoalTopic, 1, &VanillaPolicy::subscribeMyGoal, this
  );
  this->subscribeOthersLocalPlan_ = this->nh_.subscribe(
    othersEBandTopic, 1, &VanillaPolicy::subscribeOthersLocalPlan, this
  );
  this->spinner = new ros::AsyncSpinner(3);
  this->spinner->start();
}
void VanillaPolicy::subscribeMyLocalPlan(const visualization_msgs::MarkerArray plan)
{
  // subscribe my local plan
  this->myLocalPlan_.clear();
  for(int i=0; i<plan.markers.size(); i++) this->myLocalPlan_.push_back(plan.markers[i].pose);
};
void VanillaPolicy::subscribeOthersLocalPlan(const visualization_msgs::MarkerArray plan)
{
  // subscribe other robot's local plan
  this->othersLocalPlan_.clear();
  for(int i=0; i<plan.markers.size(); i++) this->othersLocalPlan_.push_back(plan.markers[i].pose);
};
void VanillaPolicy::subscribeMyGoal(const move_base_msgs::MoveBaseActionGoal goal)
{
  if(storeNewGoal == true){
    previous_goal = goal.goal.target_pose.pose;
  }
};
float VanillaPolicy::dist(geometry_msgs::Point myPoint, geometry_msgs::Point othersPoint)
{
  // return distance between given two points
  return hypot(myPoint.x - othersPoint.x, myPoint.y - othersPoint.y);
};
bool VanillaPolicy::detectCollision(float radius = 0.6)
{
  // return true if it detect overlap between local plan
  int i = myLocalPlan_.size()-1;
  int j = othersLocalPlan_.size()-1;

  if(i==-1 || j==-1) return false;

  for( ; i!=-1 || j!=-1; ){
    if(i==-1){
      if(dist(myLocalPlan_[0].position, othersLocalPlan_[j].position) < radius){
        if(DEBUG) ROS_INFO_STREAM("ME <-- OTHER");
        return true;
      }
      j--;
    }
    else if(j==-1){
      if(dist(myLocalPlan_[i].position, othersLocalPlan_[0].position) < radius){
        if(DEBUG) ROS_INFO_STREAM("ME --> OTHER");
        return true;
      }
      i--;
    }
    else{
      if(dist(myLocalPlan_[i].position, othersLocalPlan_[0].position) < radius){
        if(DEBUG) ROS_INFO_STREAM("ME --> OTHER");
        return true;
      }
      if(dist(myLocalPlan_[0].position, othersLocalPlan_[j].position) < radius){
        if(DEBUG) ROS_INFO_STREAM("ME <-- OTHER");
        return true;
      }
      if(dist(myLocalPlan_[i].position, othersLocalPlan_[j].position) < radius){
        if(DEBUG) ROS_INFO_STREAM("ME <-> OTHER");
        return true;
      }
      i--;
      j--;
    }
  }
  return false;
};
bool VanillaPolicy::waitUntilSafe()
{
  // If ${prudence} number of safe situation detected
  // return true otherwise return false
  // about = 0.1 * prudence [m] safe distance

  if( myLocalPlan_.size()==0 || othersLocalPlan_.size()==0 ) return false;
  if( dist(prev_pose.position, othersLocalPlan_[0].position) < 0.08 ) return false;

  prev_pose = othersLocalPlan_[0];
  bool isMovingAway = true;

  float prev_dist = dist(myLocalPlan_[0].position, othersLocalPlan_[0].position);
  float curr_dist = -1.0;

  for(int j=1; j<othersLocalPlan_.size(); j++){
    curr_dist = dist(myLocalPlan_[0].position, othersLocalPlan_[j].position);
    if(curr_dist < prev_dist){
      isMovingAway = false;
      break;
    }
  }
  if(isMovingAway == true){
    this->safetyStack.push_back(true);
  }
  else{
    this->safetyStack.clear();
  }

  if(this->safetyStack.size() >= this->PRUDENCE){
    this->safetyStack.clear();
    //ROS_INFO_STREAM(othersLocalPlan_[0].position.x<<", "<<othersLocalPlan_[0].position.y);
    //ROS_INFO_STREAM("SAFE!");
    return true;
  }
  return false;
};
geometry_msgs::Pose VanillaPolicy::coord2Pose(float x, float y, float yaw)
{
  geometry_msgs::Pose pose;

  tf::Vector3 point;
  point.setValue(x,y,0);
  tf::pointTFToMsg(point, pose.position);

  tf::Quaternion quat;
  quat.setRPY(0.0, 0.0, yaw);
  tf::quaternionTFToMsg(quat, pose.orientation);

  return pose;
};
geometry_msgs::Pose VanillaPolicy::coord2Pose(std::vector<float> coord)
{
  if(coord.size() != 3){
    ROS_INFO_STREAM("Coordinate must be [x, y, yaw] form.");
    exit(1);
  }
  return coord2Pose(coord[0], coord[1], coord[2]);
};
void VanillaPolicy::setGoal(float x, float y, float yaw)
{
  this->goal_pose_ = coord2Pose(x, y, yaw);
};
void VanillaPolicy::move(geometry_msgs::Pose goal_pose)
{
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = this->frame_id_;
  goal.target_pose.header.stamp    = ros::Time::now();
  goal.target_pose.pose            = goal_pose;

  client_ac_.sendGoal(goal);
};
void VanillaPolicy::stop()
{
  this->client_ac_.cancelGoal();
}
bool VanillaPolicy::isArrive()
{
  if(client_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && this->toWaypoint == false){
    return true;
  }
  return false;
};
bool VanillaPolicy::isAborted()
{
  if(client_ac_.getState() == actionlib::SimpleClientGoalState::ABORTED){
    return true;
  }
  return false;
};
void VanillaPolicy::safety()
{
  // If turn around,
  // switch to chicken strategy.
}
void VanillaPolicy::resume(){
  this->move(this->previous_goal);
}
// CHICKEN==============
bool VanillaPolicy::readObjectApproachFile(const std::string& filename,
    std::map<std::string, geometry_msgs::Pose>& object_approach_map)
{

  object_approach_map.clear();
  if (!boost::filesystem::exists(filename)) {

    return false;
  }
  std::ifstream fin(filename.c_str());

  YAML::Node doc;
  doc = YAML::Load(fin);

  for (size_t i = 0; i < doc.size(); i++) {
    std::string name;
    geometry_msgs::Pose pose;
    float yaw;
    name = doc[i]["name"].as<std::string>();
    pose.position.x = doc[i]["point"][0].as<float>();
    pose.position.y = doc[i]["point"][1].as<float>();
    pose.position.z = 0;
    yaw = doc[i]["point"][2].as<float>();
    yaw += M_PI;
    tf::quaternionTFToMsg(
        tf::createQuaternionFromYaw(yaw), pose.orientation);
    object_approach_map[name] = pose;
  }

  fin.close();
  return true;
};
float VanillaPolicy::dist2ParkingZone(geometry_msgs::Pose start_pose, geometry_msgs::Pose goal_pose)
{
  float distance = 0.;

  nav_msgs::GetPlan srv;
  geometry_msgs::PoseStamped &start = srv.request.start;
  geometry_msgs::PoseStamped &goal  = srv.request.goal;

  start.header.frame_id = goal.header.frame_id = frame_id_;
  start.header.stamp    = goal.header.stamp    = ros::Time::now();

  start.pose = start_pose;
  goal.pose  = goal_pose;

  srv.request.tolerance = 0.5+1e-6;
  if(this->makePlanService_.call(srv)){
    // This is an estimation of distance.
    // The error does not exceed +- 10 cm.
    // Considering the uncertainty of system, this is accurate enough.
    distance = 0.025 * srv.response.plan.poses.size();
  }else{
    distance = 9999.;
    ROS_INFO_STREAM("Planning failed");
  }
  //if(DEBUG) ROS_INFO_STREAM("Distance to parking zone: "<<distance);
  //ROS_INFO_STREAM("Distance to parking zone: "<<distance);
  return distance;
}
geometry_msgs::Pose VanillaPolicy::findWaypoint_chicken()
{
  float my_dist, others_dist;
  geometry_msgs::Pose best_spot;
  geometry_msgs::Pose backup_spot;
  float best_dist = 9999.;
  float backup_dist = 9999.;

  int timeout = 0; // if it makes wrong planning, retry at most 3 times
  for(iter_ = parkingZones_.begin(); iter_ != parkingZones_.end(); iter_++){
    //ROS_INFO_STREAM(iter_->first);
    my_dist     = dist2ParkingZone(iter_->second, myLocalPlan_[0]);
    others_dist = dist2ParkingZone(iter_->second, othersLocalPlan_[0]);

    if(timeout > 3){
      if( (my_dist == 0 || others_dist == 0)){
        //ROS_INFO_STREAM("Wrong Planning");
        iter_--;
        timeout++;
        continue;
      }
    }
    else{
      timeout = 0;
    }
    if(my_dist < others_dist && my_dist < best_dist){
      best_dist = my_dist;
      best_spot = iter_->second;
    }
    if(my_dist < backup_dist){
      backup_dist = my_dist;
      backup_spot = iter_->second;
    }
  }
  //ROS_INFO_STREAM(best_spot <<"\t"<<backup_spot);

  return best_spot;
}
void VanillaPolicy::safety_chicken()
{
  // Define general safety cycle in this function
  if(this->toWaypoint == true && this->waitUntilSafe() == true){
    this->resume();
    this->storeNewGoal = true;
    this->toWaypoint = false;
  }
  if(this->detectCollision(0.6) == true && this->toWaypoint == false){
    this->storeNewGoal = false;
    this->toWaypoint = true;

    this->move( this->findWaypoint_chicken() );
  }
}
void VanillaPolicy::readParkingZones()
{
  //ROS_INFO_STREAM("reading parking zones...");
  std::string pkg_path = ros::package::getPath("utexas_gdc");
  std::string file_path;
  if(this->PLATFORM == "simulation"){
    file_path = pkg_path + "/maps/simulation/3ne/objects.yaml";
  }
  else{
    file_path = pkg_path + "/maps/real/3/objects.yaml";
  }
  bool success = this->readObjectApproachFile(file_path, this->parkingZones_);
  if(success == false){
    ROS_INFO_STREAM(file_path + " does not exitsts!");
    exit(0);
  }
  if(this->DEBUG){
    for(std::map<std::string, geometry_msgs::Pose>::iterator iter = parkingZones_.begin();iter!=parkingZones_.end(); iter++){
      std::cout << iter->first << std::endl;
      std::cout << iter->second << std::endl;
    }
  }
}
// OBEY==================================
geometry_msgs::Pose VanillaPolicy::findWaypoint_obey()
{
  multi_robot_collision_avoidance::EvalWaypoint srv;
  srv.request.chicken_pose = this->myLocalPlan_[0];
  srv.request.bold_pose = this->othersLocalPlan_[0];

  move_base_msgs::MoveBaseGoal wp;
  obey_find.call(srv);

  geometry_msgs::Pose waypoint;
  waypoint = this->myLocalPlan_[0];
  waypoint.position.x = srv.response.waypoint.position.x;
  waypoint.position.y = srv.response.waypoint.position.y;

  return waypoint;
}

void VanillaPolicy::safety_obey()
{
  // Define general safety cycle in this function
  if(this->toWaypoint == true && this->waitUntilSafe() == true){
    this->resume();
    this->storeNewGoal = true;
//    this->toWaypoint = false;
  }
  if(this->detectCollision(0.6) == true && this->toWaypoint == false){
    this->storeNewGoal = false;
    this->toWaypoint = true;
    geometry_msgs::Pose wp = this->findWaypoint_obey();
    ROS_INFO_STREAM(wp);
    this->move( wp );
  }
}
// Added
bool VanillaPolicy::is_other_arrive(std::vector<float> coord){
  if(othersLocalPlan_.size() == 0){
    return false;
  }
  float dist = hypot(coord[0] - othersLocalPlan_[0].position.x, coord[1] - othersLocalPlan_[0].position.y);

  tf::Quaternion q(
    othersLocalPlan_[0].orientation.x,
    othersLocalPlan_[0].orientation.y,
    othersLocalPlan_[0].orientation.z,
    othersLocalPlan_[0].orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  ROS_INFO_STREAM(coord[0]<<", "<<othersLocalPlan_[0].position.x <<",   "<<coord[1] << othersLocalPlan_[0].position.y);
  ROS_INFO_STREAM(hypot(coord[2] - yaw, 0));

  double angle = hypot(coord[2] - yaw,0);
  if(dist < 0.2 && (angle < 0.2 || angle > 6.08) ){
    return true;
  }
  return false;
};
