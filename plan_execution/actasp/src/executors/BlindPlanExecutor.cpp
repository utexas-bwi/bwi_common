#include <utility>

#include <actasp/executors/BlindPlanExecutor.h>

#include <actasp/AspKR.h>
#include <actasp/ExecutionObserver.h>
#include <actasp/PlanningObserver.h>

#include <iostream>
#include <actasp/action_utils.h>
#include <actasp/execution_observer_utils.h>

using namespace std;

namespace actasp {


BlindPlanExecutor::BlindPlanExecutor(AspKR &reasoner,
                                     Planner &planner,
                                     const std::map<std::string, Action *> &actionMap
    ) noexcept(false) :
            goalRules(),
            isGoalReached(false),
            hasFailed(false),
            actionMap(),
            plan(),
            actionCounter(0),
            newAction(true),
            kr(reasoner),
            planner(planner),
            executionObservers() {
        transform(actionMap.begin(), actionMap.end(), inserter(this->actionMap, this->actionMap.begin()),
                  ActionMapDeepCopy());
    }

    BlindPlanExecutor::~BlindPlanExecutor() {
        for_each(actionMap.begin(), actionMap.end(), ActionMapDelete());
    }

    struct NotifyNewPlan {

        explicit NotifyNewPlan(AnswerSet plan) : plan(std::move(plan)) {}

      void operator()(PlanningObserver &observer) {
        observer.planChanged(plan);
        }

        AnswerSet plan;

    };

    void BlindPlanExecutor::computePlan() {

      isGoalReached = kr.currentStateQuery(goalRules).satisfied;

        if (!isGoalReached) {
          plan = planner.computePlan(goalRules).instantiateActions(actionMap);
          actionCounter = 0;
        }

        hasFailed = plan.empty();

        if (!hasFailed)
            for_each(planningObservers.begin(), planningObservers.end(), NotifyNewPlan(planToAnswerSet(plan)));

    }

    void BlindPlanExecutor::setGoal(const std::vector<actasp::AspRule> &goalRules) noexcept {
      this->goalRules.clear();
      for (const auto &rule: goalRules) { this->goalRules.push_back(rule);}


      for_each(executionObservers.begin(),executionObservers.end(),NotifyGoalChanged(goalRules));

        computePlan();
    }


    void BlindPlanExecutor::executeActionStep() {

        if (isGoalReached || hasFailed)
            return;


      auto &current = plan.front();

        if (newAction) {
            for_each(executionObservers.begin(), executionObservers.end(),
                     NotifyActionStart(current->toFluent(actionCounter)));
            newAction = false;
        }


        current->run();

        if (current->hasFinished()) {
            //destroy the action and pop a new one

            actionCounter += 1;
            auto as_fluent = current->toFluent(actionCounter);
            for_each(executionObservers.begin(), executionObservers.end(),
                     [as_fluent, &current](ExecutionObserver &observer) {
                       observer.actionTerminated(as_fluent, current->hasFailed());
                     });

          bool action_failed = current->hasFailed();
          // WARNING: Current may become undefined after this point, as it is a pointer to the front of the plan
            plan.pop_front();
          // Current now points to the next action in the plan
            newAction = true;

            std::cout << "Remaining plan size: " << plan.size() << std::endl;

            if (plan.empty()) {
              if (action_failed) {
                    hasFailed = true;
                } else {
                    // Note: Really, this just means the plan is done. The goal may not be reached, but we have executed the plan.
                    isGoalReached = true;
                }

            }

        }

    }

void BlindPlanExecutor::addExecutionObserver(ExecutionObserver &observer) noexcept {
  executionObservers.emplace_back(observer);
    }

void BlindPlanExecutor::removeExecutionObserver(ExecutionObserver &observer) noexcept {
        executionObservers.remove(observer);
    }

void BlindPlanExecutor::addPlanningObserver(PlanningObserver &observer) noexcept {
  planningObservers.emplace_back(observer);
    }

void BlindPlanExecutor::removePlanningObserver(PlanningObserver &observer) noexcept {
        planningObservers.remove(observer);
    }

}
