#include "GoThrough.h"

#include "ActionFactory.h"

#include "plan_execution/CurrentStateQuery.h"

#include <ros/console.h>
#include <ros/ros.h>

#include <algorithm>

using namespace std;

namespace bwi_krexec {
  
  static vector<string> createVector(const std::string& doorName) {
    vector<string> paramVector(1);
    paramVector[0] = doorName;
    
    return paramVector;
  }
    
  
GoThrough::GoThrough(const std::string& doorName): 
              plan_exec::LogicalAction("gothrough",createVector(doorName)),
              failed(false){}
 
 
 struct IsFluentAt {
   
   bool operator()(const plan_execution::AspFluent& fluent) {
     return fluent.name == "at";
   }
   
 };
 
void GoThrough::run() {
  
  ros::NodeHandle n;
  ros::ServiceClient krClient = n.serviceClient<plan_execution::CurrentStateQuery> ( "current_state_query" );
  krClient.waitForExistence();
  
  plan_execution::CurrentStateQuery csq;
  
  krClient.call(csq);
  
  vector<plan_execution::AspFluent>::const_iterator atIt = 
                    find_if(csq.response.answer.fluents.begin(), csq.response.answer.fluents.end(), IsFluentAt());
  
   bool error = false;
   string initialPosition = "";
                    
  if(atIt == csq.response.answer.fluents.end()) {
    ROS_ERROR("ApproachDoor: fluent \"at\" missing ");
    error = true;
  }
  else 
    initialPosition = atIt->variables[0];
  
  plan_exec::LogicalAction::run();
  
   krClient.call(csq);
  
  atIt = find_if(csq.response.answer.fluents.begin(), csq.response.answer.fluents.end(), IsFluentAt());
                    
  if(!error && atIt != csq.response.answer.fluents.end()) 
    failed = initialPosition == atIt->variables[0];
  
}

ActionFactory gothroughFactory(new GoThrough(""));
  
  
}
