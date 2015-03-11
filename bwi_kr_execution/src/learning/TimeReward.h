#ifndef bwi_krexec_TimeReward_h__guard
#define bwi_krexec_TimeReward_h__guard

#include "actasp/ExecutionObserver.h"
#include "actasp/AspFluent.h"

#include "RewardFunction.h"

#include <map>

#include <ros/time.h>
#include <ros/console.h>

namespace bwi_krexec {

template <typename State>
class TimeReward : public RewardFunction<State>, public actasp::ExecutionObserver {
public:
  double r(const State &, const actasp::AspFluent &action, const State &) const throw() {
    
    ROS_INFO_STREAM("REWARD checking for: " << action.toString());
    
    std::map<actasp::AspFluent, ros::Time>::const_iterator element = startingTimes.find(action);
    
    if(element == startingTimes.end())
      return 0;
    
    double reward = ros::Duration(ros::Time::now() -  element->second).toSec();
    
    return -reward ;
    
  }
  
  void actionStarted(const actasp::AspFluent& action) throw() {
    
    if( startingTimes.find(action) != startingTimes.end())
      startingTimes.erase(action);
    
    startingTimes.insert(std::make_pair(action,ros::Time::now()));
    
  }
  
  void actionTerminated(const actasp::AspFluent& action) throw() {
    
  }
  
  virtual ~TimeReward() {}  
private:
  std::map<actasp::AspFluent, ros::Time, actasp::ActionComparator> startingTimes;
};

}

#endif
