#pragma once

#include <actasp_ros/RosAction.h>
#include <actasp_ros/ExecuteSmachStateMachineAction.h>
#include "RosAction.h"

namespace actasp_ros {
typedef RosAction<actasp_ros::ExecuteSmachStateMachineAction, actasp_ros::ExecuteSmachStateMachineGoal, actasp_ros::ExecuteSmachStateMachineResult> SmachRosAction;

class SmachAction : public SmachRosAction {
public:
  SmachAction(const std::string &topic_name) : SmachRosAction(topic_name) {
  }

protected:
  boost::optional<actasp_ros::ExecuteSmachStateMachineGoal> prepareGoal() override {
    return actasp_ros::ExecuteSmachStateMachineGoal();
  }

};

}
