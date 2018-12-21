
#include "plan_execution/msgs_utils.h"
#include "actasp/action_utils.h"
#include "actasp/executors/ReplanningPlanExecutor.h"
#include "actasp/planners/AnyPlan.h"
#include "actasp/ExecutionObserver.h"
#include "actasp/PlanningObserver.h"
#include "actasp/AnswerSet.h"
#include <actasp/reasoners/Clingo.h>


#include "plan_execution/ExecutePlanAction.h"

#include "actions/ActionFactory.h"


#include <actionlib/server/simple_action_server.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

#include <boost/filesystem.hpp>

#include <string>

const int MAX_N = 20;
const std::string queryDirectory("/tmp/bwi_action_execution/");


using namespace std;
using namespace bwi_krexec;
using namespace actasp;
using namespace plan_exec;

typedef actionlib::SimpleActionServer<plan_execution::ExecutePlanAction> Server;


PlanExecutor *executor;

struct PrintFluent {
  
  PrintFluent(ostream& stream) : stream(stream) {}
  
  string operator()(const AspFluent& fluent) {
    stream << fluent.toString() << " ";
  }
  
  ostream &stream;
  
};

struct Observer : public ExecutionObserver, public PlanningObserver {
  
  void actionStarted(const AspFluent& action) throw() {
    ROS_INFO_STREAM("Starting execution: " << action.toString());
  }
  
  void actionTerminated(const AspFluent& action) throw() {
    ROS_INFO_STREAM("Terminating execution: " << action.toString());
  }
  
  
  void planChanged(const AnswerSet& newPlan) throw() {
   stringstream planStream;
   
   ROS_INFO_STREAM("plan size: " << newPlan.getFluents().size());
   
   copy(newPlan.getFluents().begin(),newPlan.getFluents().end(),ostream_iterator<string>(planStream," "));
   
   ROS_INFO_STREAM(planStream.str());
  }
  
  void goalChanged(std::vector<actasp::AspRule> newGoalRules) throw() {}
  
  void policyChanged(PartialPolicy* policy) throw() {}
  
  
};

void executePlan(const plan_execution::ExecutePlanGoalConstPtr& plan, Server* as) {

  vector<AspRule> goalRules;

  transform(plan->aspGoal.begin(),plan->aspGoal.end(),back_inserter(goalRules),TranslateRule());

  executor->setGoal(goalRules);

  ros::Rate loop(10);

  while (!executor->goalReached() && !executor->failed() && ros::ok()) {

    if (!as->isPreemptRequested()) {
      executor->executeActionStep();
    }
    else {
      if(as->isNewGoalAvailable()) {
        goalRules.clear();
        const plan_execution::ExecutePlanGoalConstPtr& newGoal = as->acceptNewGoal();
        transform(newGoal->aspGoal.begin(),newGoal->aspGoal.end(),back_inserter(goalRules),TranslateRule());
        executor->setGoal(goalRules);
      }
    }
         loop.sleep();
  }


  if (executor->goalReached()) {
    ROS_INFO("Execution succeded");
    as->setSucceeded();
  }
  else {
    ROS_INFO("Execution failed");
    as->setAborted();
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

//  create initial state



  bool simulating;
  privateNode.param<bool>("simulation",simulating,false);
  ActionFactory::setSimulation(simulating); 
  
  boost::filesystem::create_directories(queryDirectory);

  FilteringQueryGenerator *reasoner = Clingo::getQueryGenerator("n",queryDirectory,domainDirectory,actionMapToSet(ActionFactory::actions()));

  Planner *planner = new AnyPlan(reasoner,1.);
  
  //need a pointer to the specific type for the observer
  ReplanningPlanExecutor *replanner = new ReplanningPlanExecutor(reasoner,planner,ActionFactory::actions());
  executor = replanner;
  
  Observer observer;
  executor->addExecutionObserver(&observer);
  replanner->addPlanningObserver(&observer);

  Server server(privateNode, "execute_plan", boost::bind(&executePlan, _1, &server), false);
  server.start();

  ros::spin();

  
  return 0;
}
