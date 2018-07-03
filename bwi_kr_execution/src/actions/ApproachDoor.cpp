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
    
  
ApproachDoor::ApproachDoor():
              LogicalNavigation("approach"),
              failed(false){}
 

 
void ApproachDoor::run()  {
  
  LogicalNavigation::run();

  ros::NodeHandle n;
  ros::ServiceClient krClient = n.serviceClient<plan_execution::CurrentStateQuery> ( "current_state_query" );
  krClient.waitForExistence();
  
  plan_execution::CurrentStateQuery csq;
  
  krClient.call(csq);
  
  AnswerSet answer = plan_exec::TranslateAnswerSet()(csq.response.answer);
  
  failed = !answer.contains(AspFluent("facing",this->getParameters(),0));
  
  
}

std::vector<std::string> ApproachDoor::prepareGoalParameters() const {
  vector<string> params;
  params.push_back(door_name);
  return params;
}

ActionFactory approachFactory(new ApproachDoor());
  
  
}