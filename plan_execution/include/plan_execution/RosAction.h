#ifndef actexec_RosAction_h__guard
#define actexec_RosAction_h__guard

#include "actasp/Action.h"

#include <actionlib/client/simple_action_client.h>

#include <knowledge_representation/MemoryConduit.h>
namespace plan_exec {

	template <typename ROSAction, typename Goal, typename Result>
class RosAction : public actasp::Action {
public:

    typedef actionlib::SimpleActionClient<ROSAction> ActionClient;
    typedef typename boost::shared_ptr<const Result> ResultConstPtr;

    explicit RosAction(const std::string &action_topic_name) :
            done(false),
            failed(false),
            action_topic_name(action_topic_name),
            request_in_progress(false), ac() {}

    ~RosAction() {
        if (request_in_progress) {
            // The goal was sent but the action is being terminated before being allowed to finish. Cancel that command!
            ac->cancelGoal();
            delete ac;
        }
    }

    virtual void run() {
        if (!request_in_progress) {
            typename boost::optional<Goal> goal = prepareGoal();
            if (!goal) {
                onFinished(false, {});
                done = true;
            }
            ac = new ActionClient(action_topic_name, true);
            ac->waitForServer();

            ac->sendGoal(*goal);
            request_in_progress = true;
        }

        ResultConstPtr result = checkForResult();
        if (result) {
            onFinished(!failed, result);
            // Mark the request as completed.
            done = true;
        }

    }

    bool hasFinished() const { return done; }

    bool hasFailed() const { return failed; }

protected:

    virtual boost::optional <Goal> prepareGoal() = 0;

    virtual void onFinished(bool success, ResultConstPtr result) = 0;

    ResultConstPtr checkForResult() {
        bool finished_before_timeout = ac->waitForResult(ros::Duration(0.5f));

        // If the action finished, need to do some work here.
        if (finished_before_timeout) {
            boost::shared_ptr<const Result> result = ac->getResult();
            if (ac->getState() == actionlib::SimpleClientGoalState::ABORTED ||
                ac->getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
                failed = true;
            }

            // Cleanup the simple action client.
            request_in_progress = false;
            delete ac;
            return result;
        }

        return {};

    }



private:
    ActionClient* ac;
    const std::string action_topic_name;
    bool done;
    bool failed;
    bool request_in_progress;
};
}

#endif
