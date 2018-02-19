#include "ApproachDoor.h"

#include "ActionFactory.h"
#include "plan_execution/msgs_utils.h"

#include "actasp/AnswerSet.h"

#include "plan_execution/CurrentStateQuery.h"

#include <ros/ros.h>

#include <algorithm>




using namespace std;
using namespace actasp;

namespace bwi_krexec {
  
  static vector<string> createVector(const std::string& doorName) {
    vector<string> paramVector(1);
    paramVector[0] = doorName;
    
    return paramVector;
  }
    
  
ApproachDoor::ApproachDoor(const std::string& doorName): 
              plan_exec::LogicalAction("approach",createVector(doorName)),
              failed(false){}
 
 
 struct IsFluentAt {
   
   bool operator()(const plan_execution::AspFluent& fluent) {
     return fluent.name == "at";
   }
   
 };
 
void ApproachDoor::run()  {
  
  plan_exec::LogicalAction::run();

  ros::NodeHandle n;
  ros::ServiceClient krClient = n.serviceClient<plan_execution::CurrentStateQuery> ( "current_state_query" );
  krClient.waitForExistence();
  
  plan_execution::CurrentStateQuery csq;
  
  krClient.call(csq);
  
  AnswerSet answer = plan_exec::TranslateAnswerSet()(csq.response.answer);
  
  failed = !answer.contains(AspFluent("facing",this->getParameters(),0));
  
  
}

ActionFactory approachFactory(new ApproachDoor(""));
  
  
}