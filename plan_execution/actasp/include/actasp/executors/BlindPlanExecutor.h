#ifndef actasp_BlindActionExecutor_h__guard
#define actasp_BlindActionExecutor_h__guard


#include <actasp/PlanExecutor.h>

#include <stdexcept>
#include <list>
#include <map>
#include <actasp/Action.h>

namespace actasp {

    class AspKR;

    class Planner;

    class Action;

    class PlanningObserver;

    class BlindPlanExecutor : public PlanExecutor {

    public:

        BlindPlanExecutor(actasp::AspKR *reasoner,
                            actasp::Planner *planner,
                            const std::map<std::string, Action *> &actionMap
        ) noexcept(false);

        using PlanExecutor::setGoal;

        void setGoal(const std::vector<actasp::AspRule> &goalRules) noexcept;

        bool goalReached() const noexcept {
            return isGoalReached;
        }

        bool failed() const noexcept {
            return hasFailed;
        }

        void executeActionStep();

        void addExecutionObserver(ExecutionObserver *observer) noexcept;

        void removeExecutionObserver(ExecutionObserver *observer) noexcept;

        void addPlanningObserver(PlanningObserver *observer) noexcept;

        void removePlanningObserver(PlanningObserver *observer) noexcept;

        ~BlindPlanExecutor();


    private:
        std::vector<actasp::AspRule> goalRules;
        bool isGoalReached;
        bool hasFailed;
        std::map<std::string, Action *> actionMap;

        std::list<Action::Ptr> plan;
        unsigned int actionCounter;
        bool newAction;

        AspKR *kr;
        Planner *planner;

        std::list<ExecutionObserver *> executionObservers;
        std::list<PlanningObserver *> planningObservers;

        void computePlan();


    };


}
#endif
