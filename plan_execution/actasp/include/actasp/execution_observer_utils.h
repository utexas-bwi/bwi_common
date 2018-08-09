#include <utility>

#ifndef actasp_execution_observer_utiles_h__guard
#define actasp_execution_observer_utiles_h__guard

#include <actasp/AspFluent.h>
#include <actasp/ExecutionObserver.h>

namespace actasp {


struct NotifyActionStart {

  NotifyActionStart(AspFluent action) : action(std::move(action)) {}

  void operator()(ExecutionObserver &observer) {
    observer.actionStarted(action);
  }

  AspFluent action;
};

struct NotifyGoalChanged {

  NotifyGoalChanged(const std::vector<actasp::AspRule> &goalRules) : goalRules(goalRules) {}

  void operator()(ExecutionObserver &observer) {
    observer.goalChanged(goalRules);
  }

  const std::vector<actasp::AspRule> &goalRules;
};


}

#endif
