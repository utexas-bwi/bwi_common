
#include "plan_execution/msgs_utils.h"
#include "plan_execution/RemoteReasoner.h"

#include "actasp/action_utils.h"
#include "actasp/executors/ReplanningPlanExecutor.h"
#include "actasp/ExecutionObserver.h"
#include "actasp/PlanningObserver.h"
#include "actasp/AnswerSet.h"
#include <actasp/reasoners/Clingo.h>

#include "plan_execution/ExecutePlanAction.h"
#include <plan_execution/PlanExecutorNode.h>
#include <plan_execution/observers.h>

#include "observers.h"
#include "utils.h"
#include "BwiResourceManager.h"
#include "actions/ActionFactory.h"

#include <actionlib/server/simple_action_server.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

#include <boost/filesystem.hpp>

#include <string>

#include <iostream>
#include <fstream>

using namespace plan_exec;
using namespace bwi_krexec;
using namespace std;

const static std::string memory_log_path = string("/tmp/villa_action_execution_logs/");

std::string working_memory_path;
void updateFacts() {
  string as_string = memoryConduitToAsp();
  std::ofstream working_memory(working_memory_path.c_str());
  working_memory << as_string;
  working_memory.close();

  {
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    stringstream stampstream;
    stampstream << std::put_time(&tm, "%d-%m-%Y_%H-%M-%S");
    string log_name = stampstream.str() + ".asp";

    ofstream log((memory_log_path + log_name).c_str());
    log << as_string;
    log.close();
  }
}


int main(int argc, char**argv) {
  ros::init(argc, argv, "action_executor");
  ros::NodeHandle n;

//   if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
//     ros::console::notifyLoggerLevelsChanged();
//   }
  
  ros::NodeHandle privateNode("~");
  string domainDirectory;
  n.param<std::string>("plan_execution/domain_directory", domainDirectory, ros::package::getPath("bwi_kr_execution")+"/domain/");
  
  if(domainDirectory.at(domainDirectory.size()-1) != '/')
    domainDirectory += '/';


  bool simulating;
  privateNode.param<bool>("simulation",simulating,false);
  ActionFactory::setSimulation(simulating);

  unique_ptr<BwiResourceManager> resourceManager = unique_ptr<BwiResourceManager>(new BwiResourceManager());

  vector<std::reference_wrapper<actasp::ExecutionObserver>> execution_observers;
  vector<std::reference_wrapper<actasp::PlanningObserver>> planning_observers;
  ConsoleObserver observer;
  std::function<void()> function = std::function<void()>(updateFacts);
  KnowledgeUpdater updating_observer(function, *resourceManager);
  execution_observers.emplace_back(observer);
  planning_observers.emplace_back(observer);

  execution_observers.emplace_back(updating_observer);
  planning_observers.emplace_back(updating_observer);

  std::map<std::string, actasp::ActionFactory> actions = {};

  PlanExecutorNode node(domainDirectory, actions, *resourceManager, execution_observers, planning_observers);
  working_memory_path = node.working_memory_path;
  ros::spin();

  
  return 0;
}
