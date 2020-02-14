#include <actionlib/client/simple_action_client.h>
#include <plan_execution/ExecutePlanAction.h>
#include <bwi_msgs/LogicalActionAction.h>

#include <nav_msgs/GetPlan.h>

#include <visualization_msgs/MarkerArray.h>

#include <ros/ros.h>

#include <bwi_planning_common/structures.h>
#include <boost/circular_buffer.hpp>

typedef actionlib::SimpleActionClient<plan_execution::ExecutePlanAction> ASPClient;
typedef actionlib::SimpleActionClient<bwi_msgs::LogicalActionAction> LAClient;

using namespace std;

class safetyProtocol{

protected:

  ASPClient *client_;
  LAClient *lac_;

  ros::Publisher stop;
  ros::Rate *stop_rate;
  ros::NodeHandle nh_;

  vector<geometry_msgs::Pose> myPlan_;
  vector<geometry_msgs::Pose> othersPlan_;
  boost::circular_buffer<geometry_msgs::Pose> othersHistory_;
  ros::Subscriber subscribeMyPlan_;
  ros::Subscriber subscribeOthersPlan_;
  ros::AsyncSpinner *spinner;

  map<string, geometry_msgs::Pose> parkingZones_;

  ros::ServiceClient makePlanService;
  string global_frame_id_;
  // private functions
  void subscribeMyPlan(const visualization_msgs::MarkerArray plan);
  void subscribeOthersPlan(const visualization_msgs::MarkerArray plan);
  float dist(geometry_msgs::Point my_plan_pt, geometry_msgs::Point others_plan_pt);
  float distance(const geometry_msgs::Point start_pose, const geometry_msgs::Point goal_pose);
  string findSafeZone();
  bool waitUntilSafe();
  bool distanceBaseAlarm(float warningDistance);

public:

  safetyProtocol(string ROBOT_NAME);
  void stopASPPlan();
  void goToSafeZone(string location);
  void resume(string location);
  void run(string location);
  void goTo(string location);
  bool isClientArrive(){
    if(this->client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) return true;
    return false;
  }
};
