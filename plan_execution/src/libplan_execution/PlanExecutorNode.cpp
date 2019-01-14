#include "plan_execution/msgs_utils.h"

#include <actasp/action_utils.h>
#include <actasp/reasoners/Clingo.h>
#include "actasp/executors/ReplanningPlanExecutor.h"
#include "actasp/executors/BlindPlanExecutor.h"
#include "actasp/ExecutionObserver.h"
#include "actasp/PlanningObserver.h"
#include <actasp/AspKR.h>

#include "plan_execution/ExecutePlanAction.h"

#include <actionlib/server/simple_action_server.h>

#include <boost/filesystem.hpp>

#include <plan_execution/observers.h>
#include <plan_execution/PlanExecutorNode.h>

#include <iostream>



using namespace std;
using namespace actasp;

namespace plan_exec {

const int MAX_N = 30;
const int PLANNER_TIMEOUT = 10; //seconds


PlanExecutorNode::PlanExecutorNode(const string &domain_directory, map<string, ActionFactory> action_map,
                                   actasp::ResourceManager &resourceManager,
                                   vector<reference_wrapper<ExecutionObserver>> execution_observers,
                                   vector<reference_wrapper<PlanningObserver>> planning_observers) :
    server({"~"}, "execute_plan", boost::bind(&PlanExecutorNode::executePlan, this, _1), false), working_memory_path("/tmp/current.asp") {
  ros::NodeHandle n;

  ros::NodeHandle privateNode("~");

  // Touch the working memory_path so the reasoner can verify that it exists
  fstream fs;
  fs.open(working_memory_path, ios::out);
  fs.close();

  planningReasoner = unique_ptr<actasp::FilteringQueryGenerator>(Clingo::getQueryGenerator( domain_directory));
  auto diagnosticsPath = boost::filesystem::path(domain_directory) / "diagnostics";
  if (boost::filesystem::is_directory(diagnosticsPath)) {
    auto diagnosticReasoner = std::unique_ptr<actasp::QueryGenerator>(actasp::Clingo::getQueryGenerator(diagnosticsPath.string()));
    ros_observer = std::unique_ptr<RosActionServerInterfaceObserver>(new ExplainingRosActionServerInterfaceObserver(server, std::move(diagnosticReasoner)));
  } else {
    ros_observer = std::unique_ptr<RosActionServerInterfaceObserver>(new RosActionServerInterfaceObserver(server));
  }
  {
    //need a pointer to the specific type for the observer
    auto replanner = new ReplanningPlanExecutor(*dynamic_cast<actasp::AspKR*>(planningReasoner.get()), *dynamic_cast<actasp::Planner*>(planningReasoner.get()), action_map, resourceManager);
    //BlindPlanExecutor *replanner = new BlindPlanExecutor(reasoner, reasoner, ActionFactory::actions());
    for (auto &observer: planning_observers) {
      replanner->addPlanningObserver(observer);
    }

    executor = std::unique_ptr<actasp::PlanExecutor>(replanner);

    executor->addExecutionObserver(*ros_observer);
    replanner->addPlanningObserver(*ros_observer);
  }

  for (auto &observer: execution_observers) {
    executor->addExecutionObserver(observer);
  }

  server.start();

}

PlanExecutorNode::~PlanExecutorNode() = default;

void PlanExecutorNode::executePlan(const plan_execution::ExecutePlanGoalConstPtr &plan) {
  plan_execution::ExecutePlanResult result;
  vector<AspFluentRule> goalRules;


  transform(plan->aspGoal.begin(), plan->aspGoal.end(), back_inserter(goalRules), TranslateRule());

  try {
    executor->setGoal(goalRules);
  } catch (std::logic_error &e) {
    server.setAborted(ros_observer->result);
    return;
  }

  ros::Rate loop(10);

  while (!executor->goalReached() && !executor->failed() && ros::ok() && server.isActive()) {
    if (!server.isPreemptRequested()) {
      executor->executeActionStep();
    } else {

      server.setPreempted(ros_observer->result);

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
          server.setAborted(ros_observer->result);
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
      server.setSucceeded(ros_observer->result);
  } else {
    ROS_INFO("Execution failed");
    if (server.isActive()) {
      server.setAborted(ros_observer->result);
    }
  }

}

}
