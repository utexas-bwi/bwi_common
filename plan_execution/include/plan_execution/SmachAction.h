#pragma once

#include <plan_execution/RosAction.h>
#include <plan_execution/ExecuteSmachStateMachineAction.h>
#include "RosAction.h"

namespace plan_exec {
typedef RosAction<plan_execution::ExecuteSmachStateMachineAction, plan_execution::ExecuteSmachStateMachineGoal, plan_execution::ExecuteSmachStateMachineResult> SmachRosAction;

class SmachAction : public SmachRosAction {
public:
    SmachAction(const std::string &topic_name) : SmachRosAction(topic_name) {
    }

protected:
    boost::optional <plan_execution::ExecuteSmachStateMachineGoal> prepareGoal() override {
        return plan_execution::ExecuteSmachStateMachineGoal();
    }

};

}
