#ifndef BWI_KR_EXECUTION_OBSERVERS_H
#define BWI_KR_EXECUTION_OBSERVERS_H

#include "actions/SenseLocation.h"

namespace bwi_krexec {


struct KnowledgeUpdater : public actasp::ExecutionObserver, public actasp::PlanningObserver {
  std::function<void()> updater;

  explicit KnowledgeUpdater(std::function<void()> knowledge_update_function): updater(std::move(knowledge_update_function)) {}

  void actionTerminated(const actasp::AspFluent &action, bool succeeded) noexcept override {
    updater();
  }

  void goalChanged(const std::vector<actasp::AspRule>& newGoalRules) noexcept override {

    SenseLocation senseLogicalLocation;
    while (!senseLogicalLocation.hasFinished()) {
      senseLogicalLocation.run();
    }
    updater();
  }

  void actionStarted(const actasp::AspFluent &action) noexcept override {
  }


  void planChanged(const actasp::AnswerSet &newPlan) noexcept {
  }

  void planTerminated(const PlanStatus status, const actasp::AspFluent &final_action, const actasp::AnswerSet &plan_remainder) noexcept {
  }

  void policyChanged(actasp::PartialPolicy *policy) noexcept {}

};


}

#endif //BWI_KR_EXECUTION_OBSERVERS_H
