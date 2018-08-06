#ifndef actexec_LogicalAction_h__guard
#define actexec_LogicalAction_h__guard

#include "actasp/Action.h"
#include <plan_execution/RosAction.h>
#include <knowledge_representation/LongTermMemoryConduit.h>

#include <actionlib/client/simple_action_client.h>
#include <bwi_msgs/LogicalNavAction.h>

namespace bwi_krexec {
typedef plan_exec::RosAction<bwi_msgs::LogicalNavAction, bwi_msgs::LogicalNavGoal, bwi_msgs::LogicalNavResult> LogicalNavigationRosAction;
class LogicalNavigation : public LogicalNavigationRosAction {
public:

    explicit LogicalNavigation(const std::string &logical_name);

	std::string getName() const final {return name;}

	virtual std::vector<std::string> prepareGoalParameters() const = 0;

  void configureWithResources(actasp::ResourceManager *resource_manager);

	
protected:

    std::string name;
    std::reference_wrapper<knowledge_rep::LongTermMemoryConduit> ltmc;

	boost::optional<bwi_msgs::LogicalNavGoal> prepareGoal() final;

	void onFinished(bool succeeded, ResultConstPtr result) override;

};
}

#endif
