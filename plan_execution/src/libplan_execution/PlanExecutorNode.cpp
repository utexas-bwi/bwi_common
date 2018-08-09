#include "plan_execution/msgs_utils.h"
#include "plan_execution/RemoteReasoner.h"

#include "actasp/executors/ReplanningPlanExecutor.h"
#include "actasp/executors/BlindPlanExecutor.h"
#include "actasp/ExecutionObserver.h"
#include "actasp/PlanningObserver.h"

#include "plan_execution/ExecutePlanAction.h"

#include <actionlib/server/simple_action_server.h>

#include <boost/filesystem.hpp>

#include <plan_execution/observers.h>
#include <plan_execution/PlanExecutorNode.h>
#include <actasp/action_utils.h>
#include <actasp/reasoners/Clingo.h>


using namespace std;
using namespace actasp;

namespace plan_exec {

const int MAX_N = 30;
const int PLANNER_TIMEOUT = 10; //seconds
const static std::string queryDirectory = string("/tmp/plan_execution/");
const static std::string memory_log_path = string("/tmp/plan_execution_logs/");


PlanExecutorNode::PlanExecutorNode(const string &domain_directory, map<string, ActionFactory> action_map,
                                   actasp::ResourceManager &resourceManager,
                                   vector<reference_wrapper<ExecutionObserver>> execution_observers,
                                   vector<reference_wrapper<PlanningObserver>> planning_observers) :
        server({"~"}, "execute_plan", boost::bind(&PlanExecutorNode::executePlan, this, _1), false),
        ros_observer(server), working_memory_path("/tmp/current.asp") {
  ros::NodeHandle n;

  ros::NodeHandle privateNode("~");


  boost::filesystem::create_directories(queryDirectory);
  boost::filesystem::create_directories(memory_log_path);

  FilteringQueryGenerator *generator = Clingo::getQueryGenerator("n", queryDirectory, domain_directory,
                                                                 actionMapToSet(action_map),
                                                                 working_memory_path,
                                                                 PLANNER_TIMEOUT);
  AspKR *reasoner = new RemoteReasoner(generator, MAX_N, actionMapToSet(action_map));
  //StaticFacts::retrieveStaticFacts(reasoner, domainDirectory);
  {
    //need a pointer to the specific type for the observer
    auto replanner = new ReplanningPlanExecutor(*reasoner, *reasoner, action_map, resourceManager);
    //BlindPlanExecutor *replanner = new BlindPlanExecutor(reasoner, reasoner, ActionFactory::actions());
    for (auto &observer: planning_observers) {
      replanner->addPlanningObserver(observer);
    }

    executor = std::unique_ptr<actasp::PlanExecutor>(replanner);

    executor->addExecutionObserver(ros_observer);
    replanner->addPlanningObserver(ros_observer);
  }

  for (auto &observer: execution_observers) {
    executor->addExecutionObserver(observer);
  }

  server.start();

}

PlanExecutorNode::~PlanExecutorNode() {

  boost::filesystem::remove_all(queryDirectory);

}

void PlanExecutorNode::executePlan(const plan_execution::ExecutePlanGoalConstPtr &plan) {
  plan_execution::ExecutePlanResult result;
  vector<AspRule> goalRules;


  transform(plan->aspGoal.begin(), plan->aspGoal.end(), back_inserter(goalRules), TranslateRule());

  try {
    executor->setGoal(goalRules);
  } catch (std::logic_error &e) {
    server.setAborted(ros_observer.result);
    return;
  }

  ros::Rate loop(10);

  while (!executor->goalReached() && !executor->failed() && ros::ok() && server.isActive()) {
    if (!server.isPreemptRequested()) {
      executor->executeActionStep();
    } else {

      server.setPreempted(ros_observer.result);

      if (executor->goalReached())
        ROS_INFO("Preempted, but execution succeeded");
      else
        ROS_INFO("Preempted, execution aborted");

      if (server.isNewGoalAvailable()) {
        goalRules.clear();
        const plan_execution::ExecutePlanGoalConstPtr &newGoal = server.acceptNewGoal();
        transform(newGoal->aspGoal.begin(), newGoal->aspGoal.end(), back_inserter(goalRules), TranslateRule());

        try {
          executor->setGoal(goalRules);
        } catch (std::logic_error &e) {
          server.setAborted(ros_observer.result);
          return;
        }
      }
    }
    ros::spinOnce();
    loop.sleep();
  }


  if (executor->goalReached()) {
    ROS_INFO("Execution succeeded");
    if (server.isActive())
      server.setSucceeded(ros_observer.result);
  } else {
    ROS_INFO("Execution failed");
    if (server.isActive()) {
      server.setAborted(ros_observer.result);
    }
  }

}

}
