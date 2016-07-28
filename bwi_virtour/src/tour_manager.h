#include<string>
#include <ros/ros.h>

class TourManager{
  public:
    bool tourAllowed;
    bool tourInProgress;
    long tourDuration; /* in seconds */

    ros::Time tourStartTime;
    ros::Time lastPingTime;
    std::string tourLeader;

    static const int ERROR_NOTOURALLOWED = -2;
    static const int ERROR_TOURINPROGRESS = -3;
    static const int ERROR_NOTTOURLEADER = -4;
    static const int ERROR_NOTOURINPROGRESS = -5;

    TourManager(bool tourAllowed) : tourAllowed(tourAllowed),
      tourInProgress(false), tourStartTime(0) {

      tourDuration = 30*60;
    }
};
