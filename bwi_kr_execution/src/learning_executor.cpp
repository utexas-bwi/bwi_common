
#include "msgs_utils.h"
#include "RemoteReasoner.h"
#include "StaticFacts.h"

#include "learning/QLearningActionSelector.h"
#include "learning/TimeReward.h"
#include "learning/DefaultTimes.h"

#include "bwi_kr_execution/ExecutePlanAction.h"

#include "actasp/action_utils.h"
#include "actasp/executors/MultiPolicyExecutor.h"

#include "actions/ActionFactory.h"
#include "actions/LogicalNavigation.h"


#include <actionlib/server/simple_action_server.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

#include <boost/filesystem.hpp>

#include <string>
#include <fstream>

const int MAX_N = 20;
const std::string queryDirectory("/tmp/bwi_action_execution/");
std::string valueDirectory;


using namespace std;
using namespace bwi_krexec;
using namespace actasp;

typedef actionlib::SimpleActionServer<bwi_kr_execution::ExecutePlanAction> Server;


ActionExecutor *executor;
QLearningActionSelector *selector;

struct PrintFluent {

  PrintFluent(ostream& stream) : stream(stream) {}

  string operator()(const AspFluent& fluent) {
    stream << fluent.toString() << " ";
  }

  ostream &stream;

};

struct Observer : public ExecutionObserver {

  void actionStarted(const AspFluent& action) throw() {
    ROS_INFO_STREAM("Starting execution: " << action.toString());
  }

  void actionTerminated(const AspFluent& action) throw() {
    ROS_INFO_STREAM("Terminating execution: " << action.toString());
  }

};

std::string rulesToFileName(const vector<AspRule> &rules) {
  
  stringstream ruleString;
  vector<string> ruleStringVector;
  
    
  
  vector<AspRule>::const_iterator ruleIt = rules.begin();
  
  for(; ruleIt != rules.end(); ++ruleIt) {
    
    vector<AspFluent>::const_iterator headIt = ruleIt->head.begin();
    
    for(; headIt != ruleIt->head.end(); ++headIt)
      ruleString << headIt->toString(0);
    
    ruleString << ":-";
    
    vector<AspFluent>::const_iterator bodyIt = ruleIt->body.begin();
    
    for(; bodyIt != ruleIt->body.end(); ++bodyIt)
      ruleString << bodyIt->toString(0);
    
    ruleString << "--";
    
      ruleStringVector.push_back(ruleString.str());
      ruleString.str(string());
  }
  
  sort(ruleStringVector.begin(), ruleStringVector.end()); //the order of the rules must not matter
  
  vector<string>::const_iterator stringIt = ruleStringVector.begin();
  for(; stringIt != ruleStringVector.end(); ++stringIt)
    ruleString  << *stringIt;
  
  return valueDirectory + ruleString.str();
}

void executePlan(const bwi_kr_execution::ExecutePlanGoalConstPtr& plan, Server* as) {

  vector<AspRule> goalRules;

  transform(plan->aspGoal.begin(),plan->aspGoal.end(),back_inserter(goalRules),TranslateRule());

  string valueFileName = rulesToFileName(goalRules);
  ifstream valueFileIn(valueFileName.c_str());
  selector->readFrom(valueFileIn);
  valueFileIn.close();

  executor->setGoal(goalRules);

  ros::Rate loop(10);

  while (!executor->goalReached() && !executor->failed() && ros::ok()) {

    if (!as->isPreemptRequested()) {
      executor->executeActionStep();
    } else {
      if (as->isNewGoalAvailable()) {

        ofstream newValueFileOut(valueFileName.c_str());
        selector->writeTo(newValueFileOut);
        newValueFileOut.close();

        goalRules.clear();
        const bwi_kr_execution::ExecutePlanGoalConstPtr& newGoal = as->acceptNewGoal();
        transform(newGoal->aspGoal.begin(),newGoal->aspGoal.end(),back_inserter(goalRules),TranslateRule());

        valueFileName = rulesToFileName(goalRules);
        ifstream newValueFileIn(valueFileName.c_str());
        selector->readFrom(newValueFileIn);
        newValueFileIn.close();

        executor->setGoal(goalRules);
        selector->episodeEnded();
      }
    }
    loop.sleep();
  }

  selector->episodeEnded();

  ofstream valueFileOut(valueFileName.c_str());
  selector->writeTo(valueFileOut);
  valueFileOut.close();

  if (executor->goalReached()) {
    ROS_INFO("Execution succeded");
    as->setSucceeded();
  } else {
    ROS_INFO("Execution failed");
    as->setAborted();
  }
}





int main(int argc, char**argv) {
  ros::init(argc, argv, "action_executor");
  ros::NodeHandle n;

  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }

  ros::NodeHandle privateNode("~");
  string domainDirectory;
  n.param<std::string>("bwi_kr_execution/domain_directory", domainDirectory, ros::package::getPath("bwi_kr_execution")+"/domain/");

  if (domainDirectory.at(domainDirectory.size()-1) != '/')
    domainDirectory += '/';

//  create initial state
  LogicalNavigation setInitialState("noop");
  setInitialState.run();

 

  bool simulating;
  privateNode.param<bool>("simulation",simulating,false);
  
  valueDirectory = ros::package::getPath("bwi_kr_execution") +((simulating)? "/values_simulation/" : "/values/" ) ;
  boost::filesystem::create_directories(valueDirectory);
  
  ActionFactory::setSimulation(simulating);

  boost::filesystem::create_directories(queryDirectory);
  
  AspKR *reasoner = new RemoteReasoner(MAX_N,queryDirectory,domainDirectory,actionMapToSet(ActionFactory::actions()),5);
  StaticFacts::retrieveStaticFacts(reasoner, domainDirectory);

  TimeReward<QLearningActionSelector::State> *reward = new TimeReward<QLearningActionSelector::State>();
  DefaultActionValue *timeValue = new DefaultTimes();

  selector = new QLearningActionSelector(0.3, reward , reasoner,timeValue);

  //need a pointer to the specific type for the observer
  executor = new MultiPolicyExecutor(reasoner, reasoner,selector , ActionFactory::actions(),1.5);

  executor->addExecutionObserver(selector);
  executor->addExecutionObserver(reward);

  Observer observer;
  executor->addExecutionObserver(&observer);

  Server server(privateNode, "execute_plan", boost::bind(&executePlan, _1, &server), false);
  server.start();

  ros::spin();

  server.shutdown();

  delete executor;
  delete selector;
  delete timeValue;
  delete reward;
  delete reasoner;

  return 0;
}
