#pragma once

#include <actasp/AspRule.h>

#include <vector>

namespace actasp {

class AspFluent;

class AnswerSet;

class PartialPolicy;


struct ExecutionObserver {
  enum PlanStatus {
    SUCCEEDED,
    FAILED_TO_PLAN,
    TOO_MANY_ACTION_FAILURES

  };

  std::string planStatusToString(const PlanStatus &status) {
    switch (status) {
      case SUCCEEDED:
        return "Succeeded";
      case FAILED_TO_PLAN:
        return "Failed to plan";
      case TOO_MANY_ACTION_FAILURES:
        return "Too many action failures";
    }
  }

  virtual void actionStarted(const AspFluent &action) noexcept = 0;

  virtual void actionTerminated(const AspFluent &action, bool succeeded) noexcept = 0;

  virtual void planTerminated(const PlanStatus status, const actasp::AspFluent &final_action,
                              const actasp::AnswerSet &plan_remainder) noexcept = 0;

  virtual void goalChanged(const std::vector<actasp::AspRule> &newGoalRules) noexcept = 0;

  //TODO move this into a separate observer
  virtual void policyChanged(PartialPolicy *policy) noexcept = 0;

  virtual ~ExecutionObserver() = default;

  bool operator==(const ExecutionObserver &other) const {
    return this == &other;
  }
};

inline bool operator==(const std::reference_wrapper<ExecutionObserver> &first,
                       const std::reference_wrapper<ExecutionObserver> &second) {
  return first.get() == second.get();
}

}



