#ifndef actexec_RosAction_h__guard
#define actexec_RosAction_h__guard

#include "actasp/Action.h"

#include <actionlib/client/simple_action_client.h>

#include <knowledge_representation/MemoryConduit.h>
namespace plan_exec {

	template <typename T, typename U, typename V>
class RosAction : public actasp::Action {
public:

    typedef actionlib::SimpleActionClient<T> ActionClient;
    explicit RosAction(const std::string& logical_name, const std::vector<std::string>& parameters) :
            name(name),
            parameters(parameters),
            done(false),
            request_in_progress(false), ac(), mc() {}

    ~RosAction() {
        if (request_in_progress) {
            // The goal was sent but the action is being terminated before being allowed to finish. Cancel that command!
            ac->cancelGoal();
            delete ac;
        }
    }
	
	int paramNumber() const {return 1;}
	
	std::string getName() const {return name;}


    virtual void run() {
        ROS_ERROR("Called run without action topic name");
    }

    V run(const std::string &action_topic_name, U goal) {

        ROS_DEBUG_STREAM("Executing " << name);

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
            boost::shared_ptr<const V> result = ac->getResult();

            //TODO: Call post-action perceivers to update

            // Mark the request as completed.
            done = true;

            // Cleanup the simple action client.
            request_in_progress = false;
            delete ac;
        }

    }
	
	bool hasFinished() const {return done;}

	
	virtual Action *clone() const {return new RosAction(*this);}

    Action* cloneAndInit(const actasp::AspFluent & fluent) const {
        return new RosAction(fluent.getName(),fluent.getParameters());
    }


protected:
	
	virtual std::vector<std::string> getParameters() const {return parameters;}
	
	std::string name;
	std::vector<std::string> parameters;
	bool done;


  bool request_in_progress;
  //HACK: This is a pointer to avoid messing up the default copy constructor. Maybe if mc
    // were copyable, this would not be an issue.
  knowledge_rep::MemoryConduit* mc;
private:
    ActionClient* ac;

};
}

#endif
