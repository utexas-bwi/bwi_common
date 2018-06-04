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

    explicit RosAction() :
            done(false),
            failed(false),
            request_in_progress(false), ac() {}

    ~RosAction() {
        if (request_in_progress) {
            // The goal was sent but the action is being terminated before being allowed to finish. Cancel that command!
            ac->cancelGoal();
            delete ac;
        }
    }

    virtual int paramNumber() const = 0;

    std::string getName() const = 0;


    virtual void run() {
        ROS_ERROR("Called run without action topic name");
    }

    typename boost::shared_ptr<const Result> run(const std::string &action_topic_name, Goal goal) {

        if (!request_in_progress) {
            //ac = std::unique_ptr<ActionClient>(new ActionClient(action_topic_name, true));
            ac = new ActionClient(action_topic_name, true);
            ac->waitForServer();

            ac->sendGoal(goal);
            request_in_progress = true;
        }

        bool finished_before_timeout = ac->waitForResult(ros::Duration(0.5f));

        // If the action finished, need to do some work here.
        if (finished_before_timeout) {
            boost::shared_ptr<const Result> result = ac->getResult();
            if (ac->getState() == actionlib::SimpleClientGoalState::ABORTED ||
                ac->getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
                failed = true;
            }
            //TODO: Call post-action perceivers to update

            // Mark the request as completed.
            done = true;

            // Cleanup the simple action client.
            request_in_progress = false;
            delete ac;
            return result;
        }

        return {};

    }
	
	bool hasFinished() const {return done;}

    bool hasFailed() const { return failed; }

    virtual Action *clone() const = 0;

    virtual Action *cloneAndInit(const actasp::AspFluent &fluent) const = 0;


protected:

    virtual std::vector <std::string> getParameters() const = 0;

	bool done;
    bool failed;


  bool request_in_progress;

private:
    ActionClient* ac;

};
}

#endif
