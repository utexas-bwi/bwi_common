#include <actionlib/server/simple_action_server.h>
#include <actasp_ros/observers.h>

namespace actasp_ros {
class PlanExecutorNode {

  typedef actionlib::SimpleActionServer<actasp_ros::ExecutePlanAction> Server;

public:
  PlanExecutorNode(const std::string &domain_directory,
                   std::map<std::string, actasp::ActionFactory> action_map,
                   actasp::ResourceManager &resourceManager,
                   std::vector<std::reference_wrapper<actasp::ExecutionObserver>> execution_observers,
                   std::vector<std::reference_wrapper<actasp::PlanningObserver>> planning_observers);

  ~PlanExecutorNode();

  std::string working_memory_path;
private:
  void executePlan(const actasp_ros::ExecutePlanGoalConstPtr &plan);

  std::unique_ptr<actasp::PlanExecutor> executor;
  std::unique_ptr<actasp::Solver> planningReasoner;
  Server server;

  std::unique_ptr<RosActionServerInterfaceObserver> ros_observer;

};
}
