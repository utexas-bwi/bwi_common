
#include "plan_execution/msgs_utils.h"

#include "learning/SarsaActionSelector.h"
#include "learning/TimeReward.h"
#include "learning/DefaultTimes.h"
#include "learning/ActionLogger.h"

#include "plan_execution/ExecutePlanAction.h"

#include "actasp/action_utils.h"
#include "actasp/executors/PartialPolicyExecutor.h"
#include <actasp/reasoners/Clingo.h>

#include "actions/ActionFactory.h"



#include <actionlib/server/simple_action_server.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>

#include <boost/filesystem.hpp>

#include <string>
#include <fstream>
#include <ctime>

const int MAX_N = 20;
const std::string queryDirectory("/tmp/bwi_action_execution/");
const std::string value_directory_base("/var/tmp/my_bwi_action_execution/");
std::string valueDirectory;


using namespace std;
using namespace bwi_krexec;
using namespace actasp;
using namespace plan_exec;

typedef actionlib::SimpleActionServer<plan_execution::ExecutePlanAction> Server;

PlanExecutor *executor;
SarsaActionSelector *selector;
ActionLogger *action_logger;

struct PrintFluent {

  PrintFluent(ostream& stream) : stream(stream) {}

  string operator()(const AspFluent& fluent) {
    stream << fluent.toString() << " ";
  }

  ostream &stream;

};

struct Observer : public ExecutionObserver {

  void actionStarted(const AspFluent& action) noexcept {
    ROS_INFO_STREAM("Starting execution: " << action.toString());
  }

  void actionTerminated(const AspFluent& action) noexcept {
    ROS_INFO_STREAM("Terminating execution: " << action.toString());
  }
  
    
  void goalChanged(std::vector<actasp::AspRule> newGoalRules) noexcept {}
  
  void policyChanged(PartialPolicy* policy) noexcept {}

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


void completeTask(const string& valueFileName, const ros::Time& begin, const string& time_string) {
    
  ros::Time end = ros::Time::now();
  
  selector->episodeEnded();
  
  ofstream time_file((valueFileName+"_time").c_str(), ofstream::app);
  if(executor->goalReached()) 
    time_file << 1 << " " ;
  else {
    time_file << 0 << " " ;
  }
  time_file << (end - begin).toSec() << " " << time_string << endl;
  time_file.close();
  
  
  action_logger->taskCompleted();

  ofstream valueFileOut(valueFileName.c_str());
  selector->writeTo(valueFileOut);
  valueFileOut.close();
  
}

void initiateTask(const plan_execution::ExecutePlanGoalConstPtr& plan, string &valueFileName, ros::Time& begin, char* time_string) {
  
  begin = ros::Time::now();
  
  vector<AspRule> goalRules;

  transform(plan->aspGoal.begin(),plan->aspGoal.end(),back_inserter(goalRules),TranslateRule());

  executor->setGoal(goalRules); //this has to be before selector->savevalueinitial

  valueFileName = rulesToFileName(goalRules);
  ifstream valueFileIn(valueFileName.c_str());
  selector->readFrom(valueFileIn);
  valueFileIn.close();
  selector->saveValueInitialState(valueFileName + "_initial"); 
  
  action_logger->setFile((valueFileName+"_actions"));
  
  //very practical C way of getting the current hour of day
  time_t rawtime;
  struct tm * timeinfo;
  time (&rawtime);
  timeinfo = localtime (&rawtime);
  strftime (time_string,10,"%R",timeinfo);
  
}

void executePlan(const plan_execution::ExecutePlanGoalConstPtr& plan, Server* as) {
  
  string valueFileName;
  char time_string[10];
  ros::Time begin;
  
  initiateTask(plan,valueFileName,begin,time_string); //does side effect on variables

  ros::Rate loop(10);

  while (!executor->goalReached() && !executor->failed() && as->isActive() && ros::ok()) {
    
    if (!as->isPreemptRequested()) {
      executor->executeActionStep();
    } else {
      
      as->setPreempted();
      
      if (executor->goalReached()) 
        ROS_INFO("Preempted, but execution succeded");
      else 
        ROS_INFO("Preempted, execution aborted");
      
      //if there is no new goal completeTask will be called for this task at the end of this function
      
      if (as->isNewGoalAvailable()) {
  
        completeTask(valueFileName,begin,time_string);
        
        const plan_execution::ExecutePlanGoalConstPtr& newGoal = as->acceptNewGoal();

        initiateTask(newGoal,valueFileName,begin,time_string);
                
      }

    }
    
    loop.sleep();
  }
  
  if (executor->goalReached()) {
    ROS_INFO("Execution succeded");
    if(as->isActive())
      as->setSucceeded();
  } else {
    ROS_INFO("Execution failed");
   if(as->isActive())
    as->setAborted();
  }
  
  completeTask(valueFileName,begin,time_string);

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

  if (domainDirectory.at(domainDirectory.size()-1) != '/')
    domainDirectory += '/';


 

  bool simulating;
  privateNode.param<bool>("simulation",simulating,false);
  
//  valueDirectory = ros::package::getPath("bwi_kr_execution") +((simulating)? "/values_simulation/" : "/values/" ) ;
  valueDirectory = value_directory_base + ((simulating)? "values_simulation/" : "values/" ); 
  boost::filesystem::create_directories(valueDirectory);
  
  ActionFactory::setSimulation(simulating);

  boost::filesystem::create_directories(queryDirectory);
  
  FilteringQueryGenerator *reasoner = Clingo::getQueryGenerator("n",queryDirectory,domainDirectory,actionMapToSet(ActionFactory::actions()),20);

  TimeReward<SarsaActionSelector::State> *reward = new TimeReward<SarsaActionSelector::State>();
  DefaultActionValue *timeValue = new DefaultTimes();

  SarsaParams params;
  params.alpha = 0.2;
  params.gamma = 0.9999;
  params.lambda = 0.9;
  params.epsilon = 0.2;
  
  selector = new SarsaActionSelector(reasoner,timeValue,reward,params);
  
  executor = new PartialPolicyExecutor(reasoner, reasoner,selector,ActionFactory::actions(),1.5);
  executor->addExecutionObserver(selector);
  executor->addExecutionObserver(reward);

  //need a pointer to the specific type for the observer
//   executor = new MultiPolicyExecutor(reasoner, reasoner,selector , ActionFactory::actions(),1.5);
// 
//   executor->addExecutionObserver(selector);
//   executor->addExecutionObserver(reward);

  Observer observer;
  executor->addExecutionObserver(&observer);
  
  action_logger = new ActionLogger();
  executor->addExecutionObserver(action_logger);

  Server server(privateNode, "execute_plan", boost::bind(&executePlan, _1, &server), false);
  server.start();

  ros::spin();

  server.shutdown();

  delete executor;
  delete action_logger;
  delete selector;
  delete timeValue;
  delete reward;
  delete reasoner;
  delete generator;

  return 0;
}
