#ifndef bwi_actexec_LogicalNavigation_h__guard
#define bwi_actexec_LogicalNavigation_h__guard

#include "actasp/Action.h"

#include <actionlib/client/simple_action_client.h>
#include <segbot_logical_translator/LogicalNavigationAction.h>

namespace bwi_krexec {

	
class LogicalNavigation : public actasp::Action {
public:

	explicit LogicalNavigation(const std::string &name,
                             const std::vector<std::string>& parameters = std::vector<std::string>());
	
	int paramNumber() const {return 1;}
	
	std::string getName() const {return name;}
	
	virtual void run();
	
	bool hasFinished() const {return done;}
	
	virtual Action *cloneAndInit(const actasp::AspFluent & fluent) const;
	
	virtual Action *clone() const {return new LogicalNavigation(*this);}
	
  virtual ~LogicalNavigation();
	
protected:
	
	virtual std::vector<std::string> getParameters() const {return parameters;}
	
	std::string name;
	std::vector<std::string> parameters;
	bool done;

  actionlib::SimpleActionClient<segbot_logical_translator::LogicalNavigationAction>* lnac;
  segbot_logical_translator::LogicalNavigationGoal goal;
  bool request_in_progress;

};	
}

#endif
