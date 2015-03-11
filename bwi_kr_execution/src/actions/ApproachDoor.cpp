#include "ApproachDoor.h"

#include "ActionFactory.h"
#include "../msgs_utils.h"

#include "actasp/AnswerSet.h"

#include "bwi_kr_execution/CurrentStateQuery.h"

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
              LogicalNavigation("approach",createVector(doorName)),
              failed(false){}
 
 
 struct IsFluentAt {
   
   bool operator()(const bwi_kr_execution::AspFluent& fluent) {
     return fluent.name == "at";
   }
   
 };
 
void ApproachDoor::run()  {
  
  bwi_krexec::LogicalNavigation::run();

  ros::NodeHandle n;
  ros::ServiceClient krClient = n.serviceClient<bwi_kr_execution::CurrentStateQuery> ( "current_state_query" );
  krClient.waitForExistence();
  
  bwi_kr_execution::CurrentStateQuery csq;
  
  krClient.call(csq);
  
  AnswerSet answer = TranslateAnswerSet()(csq.response.answer);
  
  failed = !answer.contains(AspFluent("facing",this->getParameters(),0));
  
  
}

static ActionFactory approachFactory(new ApproachDoor(""));
  
  
}