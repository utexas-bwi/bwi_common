#include "tour_manager.h"
#include "bwi_virtour/RequestTour.h"
#include "bwi_virtour/PingTour.h"
#include "bwi_virtour/GetTourState.h"
#include "bwi_virtour/LeaveTour.h"
#include "bwi_virtour/Authenticate.h"

TourManager* tm;

bool requestTour(bwi_virtour::RequestTour::Request &req,
    bwi_virtour::RequestTour::Response &res) {

  if (tm->tourAllowed) {
    if (!tm->tourInProgress) {
      //TODO add mutex
      tm->tourLeader = req.user;
      tm->tourInProgress = true;
      tm->tourStartTime = ros::Time::now();
      tm->lastPingTime = ros::Time::now();
      res.startTime = tm->tourStartTime.toSec();
      res.result = tm->tourDuration;
      ROS_INFO("New tour leader is %s", req.user.c_str());
    } else if (ros::Time::now() - tm->lastPingTime > ros::Duration(30)) {
      /* If the last tour expired (last ping time was more than 30 seconds ago)*/
      tm->tourLeader = req.user;
      tm->tourInProgress = true;
      tm->tourStartTime = ros::Time::now();
      tm->lastPingTime = ros::Time::now();
      res.startTime = tm->tourStartTime.toSec();
      res.result = tm->tourDuration;
      ROS_INFO("Last tour expired! New tour leader is %s", req.user.c_str());
    } else {
      res.result = TourManager::ERROR_TOURINPROGRESS;
    }
  } else {
    res.result = TourManager::ERROR_NOTOURALLOWED;
  }

  return true;
}

void checkTourExpired() {
  ros::Duration d = ros::Time::now() - tm->lastPingTime;
  if (tm->tourAllowed && tm->tourInProgress &&
      ros::Time::now() - tm->lastPingTime > ros::Duration(30)) {
    tm->tourInProgress = false;
    tm->tourLeader = "";
    ROS_INFO("Tour expired!");
  }
}

bool pingTour(bwi_virtour::PingTour::Request &req,
    bwi_virtour::PingTour::Response &res) {

  if (tm->tourAllowed) {
    if (tm->tourInProgress) {
      if (req.user.compare(tm->tourLeader) == 0) {
        tm->lastPingTime = ros::Time::now();
        res.result = 1;
      } else {
        res.result = TourManager::ERROR_NOTTOURLEADER;
      }
    } else {
      res.result = TourManager::ERROR_NOTOURINPROGRESS;
    }
  } else {
    res.result = TourManager::ERROR_NOTOURALLOWED;
  }
  return true;
}

bool getTourState(bwi_virtour::GetTourState::Request &req,
    bwi_virtour::GetTourState::Response &res) {

  checkTourExpired();

  res.tourAllowed = tm->tourAllowed;
  res.tourInProgress = tm->tourInProgress;
  res.tourDuration = tm->tourDuration;
  res.tourStartTime = tm->tourStartTime.toSec();
  res.lastPingTime = tm->lastPingTime.toSec();
  res.tourLeader = ""; //tm->tourLeader; don't actually want to leak this


  return true;
}

bool leaveTour(bwi_virtour::LeaveTour::Request &req,
    bwi_virtour::LeaveTour::Response &res) {

  if (tm->tourInProgress && req.user.compare(tm->tourLeader) == 0) {
    //TODO add mutex
    tm->tourInProgress = false;
    tm->tourLeader = "";
      ROS_INFO("%s left the tour!", req.user.c_str());

    res.result = 1;
  } else {
    res.result = TourManager::ERROR_NOTTOURLEADER;
  }

  return true;
}

bool authenticate(bwi_virtour::Authenticate::Request &req,
    bwi_virtour::Authenticate::Response &res) {
 
  if (tm->tourAllowed) {
    if (tm->tourInProgress) {
      if (req.user.compare(tm->tourLeader) == 0) {
        res.result = 1;
      } else {
        res.result = TourManager::ERROR_NOTTOURLEADER;
      }
    } else {
      res.result = TourManager::ERROR_NOTOURINPROGRESS;
    }
  } else {
    res.result = TourManager::ERROR_NOTOURALLOWED;
  }

  return true;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "tour_manager");
  ros::NodeHandle n;

  /* Create tour manager */
  bool tour_enabled;
  n.param("/tour_manager/tour_enabled", tour_enabled, false);
  tm = new TourManager(tour_enabled);

  ROS_INFO("Starting tour manager. Tours enabled: %s", tour_enabled ? "true" : "false");

  /* Advertise services */
  ros::ServiceServer request_service = n.advertiseService("tour_manager/request_tour", requestTour);
  ros::ServiceServer ping_service = n.advertiseService("tour_manager/ping_tour", pingTour);
  ros::ServiceServer get_tour_state_service = n.advertiseService("tour_manager/get_tour_state", getTourState);
  ros::ServiceServer leave_service = n.advertiseService("tour_manager/leave_tour", leaveTour);
  ros::ServiceServer authenticate_service = n.advertiseService("tour_manager/authenticate", authenticate);

  ros::spin();
  return 0;
}
