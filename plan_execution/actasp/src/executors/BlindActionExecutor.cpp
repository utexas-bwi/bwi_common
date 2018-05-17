

#include <actasp/executors/BlindActionExecutor.h>

#include <actasp/AspKR.h>
#include <actasp/AnswerSet.h>
#include <actasp/Planner.h>
#include <actasp/Action.h>
#include <actasp/action_utils.h>
#include <actasp/ExecutionObserver.h>
#include <actasp/PlanningObserver.h>
#include <actasp/execution_observer_utils.h>

#include <iostream>
#include <list>
#include <algorithm>
#include <iterator>

using namespace std;

namespace actasp {

//TODO: Rename these to plan executors
    BlindActionExecutor::BlindActionExecutor(actasp::AspKR *reasoner,
                                             actasp::Planner *planner,
                                             const std::map<std::string, Action *> &actionMap
    ) throw(std::invalid_argument) :
            goalRules(),
            isGoalReached(false),
            hasFailed(false),
            actionMap(),
            plan(),
            actionCounter(0),
            newAction(true),
            failedActionCount(0),
            kr(reasoner),
            planner(planner),
            executionObservers() {
        if (reasoner == NULL)
            throw invalid_argument("BlindActionExecutor: reasoner is NULL");

        if (planner == NULL)
            throw invalid_argument("BlindActionExecutor: planner is NULL");

        transform(actionMap.begin(), actionMap.end(), inserter(this->actionMap, this->actionMap.begin()),
                  ActionMapDeepCopy());
    }

    BlindActionExecutor::~BlindActionExecutor() {
        for_each(actionMap.begin(), actionMap.end(), ActionMapDelete());
    }

    struct NotifyNewPlan {

        explicit NotifyNewPlan(const AnswerSet &plan) : plan(plan) {}

        void operator()(PlanningObserver *observer) {
            observer->planChanged(plan);
        }

        AnswerSet plan;

    };

    void BlindActionExecutor::computePlan() {

        plan = planner->computePlan(goalRules).instantiateActions(actionMap);
        actionCounter = 0;

        hasFailed = plan.empty();

        if (!hasFailed)
            for_each(planningObservers.begin(), planningObservers.end(), NotifyNewPlan(planToAnswerSet(plan)));

    }

    void BlindActionExecutor::setGoal(const std::vector<actasp::AspRule> &goalRules) throw() {
        this->goalRules = goalRules;

        computePlan();

        failedActionCount = 0;
    }


    void BlindActionExecutor::executeActionStep() {

        if (isGoalReached || hasFailed)
            return;


        Action *current = plan.front();

        if (newAction) {
            for_each(executionObservers.begin(), executionObservers.end(),
                     NotifyActionStart(current->toFluent(actionCounter)));
            newAction = false;
        }


        current->run();

        if (current->hasFinished()) {
            //destroy the action and pop a new one

            for_each(executionObservers.begin(), executionObservers.end(),
                     NotifyActionTermination(current->toFluent(actionCounter++)));

            plan.pop_front();

            newAction = true;

            std::cout << "Remaining plan size: " << plan.size() << std::endl;

            if (plan.empty()) {

                if (current->hasFailed()) {
                    ++failedActionCount;
                } else {
                    // Note: Really, this just means the plan is done. The goal may not be reached, but we have executed the plan.
                    isGoalReached = true;
                }

                if (failedActionCount >= 3) {
                    std::cout << "FAILED TOO MANY TIMES. Aborting goal." << std::endl;
                    hasFailed = true;
                }
            } else {
                failedActionCount = 0;
            }

            delete current;

        }


    }

    void BlindActionExecutor::addExecutionObserver(ExecutionObserver *observer) throw() {
        executionObservers.push_back(observer);
    }

    void BlindActionExecutor::removeExecutionObserver(ExecutionObserver *observer) throw() {
        executionObservers.remove(observer);
    }

    void BlindActionExecutor::addPlanningObserver(PlanningObserver *observer) throw() {
        planningObservers.push_back(observer);
    }

    void BlindActionExecutor::removePlanningObserver(PlanningObserver *observer) throw() {
        planningObservers.remove(observer);
    }

}
