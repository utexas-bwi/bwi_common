#ifndef actexec_LogicalAction_h__guard
#define actexec_LogicalAction_h__guard

#include "actasp/Action.h"

#include <actionlib/client/simple_action_client.h>
#include <bwi_msgs/LogicalActionAction.h>

namespace plan_exec {

	
class LogicalAction : public actasp::Action {
public:

	explicit LogicalAction(const std::string &name,
                             const std::vector<std::string>& parameters = std::vector<std::string>());
	
	int paramNumber() const {return 1;}
	
	std::string getName() const {return name;}
	
	virtual void run();
	
	bool hasFinished() const {return done;}
	
	virtual Action *cloneAndInit(const actasp::AspFluent & fluent) const;
	
	virtual Action *clone() const {return new LogicalAction(*this);}
	
  virtual ~LogicalAction();
	
protected:
	
	virtual std::vector<std::string> getParameters() const {return parameters;}
	
	std::string name;
	std::vector<std::string> parameters;
	bool done;

  actionlib::SimpleActionClient<bwi_msgs::LogicalActionAction>* lnac;
  bwi_msgs::LogicalActionGoal goal;
  bool request_in_progress;

};	
}

#endif
