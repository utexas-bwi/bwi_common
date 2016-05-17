#include "GoThrough.h"

#include "ActionFactory.h"

#include "bwi_kr_execution/CurrentStateQuery.h"

#include <dynamic_reconfigure/Reconfigure.h>
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
              LogicalNavigation("gothrough",createVector(doorName)),
              failed(false){}
 
 
 struct IsFluentAt {
   
   bool operator()(const bwi_kr_execution::AspFluent& fluent) {
     return fluent.name == "at";
   }
   
 };
 
void GoThrough::run() {
  
  ros::NodeHandle n;
  ros::ServiceClient krClient = n.serviceClient<bwi_kr_execution::CurrentStateQuery> ( "current_state_query" );
  krClient.waitForExistence();
  
  bwi_kr_execution::CurrentStateQuery csq;
  
  krClient.call(csq);
  
  vector<bwi_kr_execution::AspFluent>::const_iterator atIt = 
                    find_if(csq.response.answer.fluents.begin(), csq.response.answer.fluents.end(), IsFluentAt());
  
   bool error = false;
   string initialPosition = "";
                    
  if(atIt == csq.response.answer.fluents.end()) {
    ROS_ERROR("ApproachDoor: fluent \"at\" missing ");
    error = true;
  }
  else 
    initialPosition = atIt->variables[0];
  
  ros::ServiceClient static_costmap_service = 
    n.serviceClient<dynamic_reconfigure::Reconfigure> ( "move_base/global_costmap/static_layer/set_parameters" );
  dynamic_reconfigure::Reconfigure static_costmap_toggle_call;
  static_costmap_toggle_call.request.config.bools.resize(1);
  static_costmap_toggle_call.request.config.bools[0].name = "enabled";
  if (!request_in_progress && !done) {
    static_costmap_toggle_call.request.config.bools[0].value = false;
    static_costmap_service.call(static_costmap_toggle_call);
  }

  bwi_krexec::LogicalNavigation::run();
  
  if (done) {
    static_costmap_toggle_call.request.config.bools[0].value = true;
    static_costmap_service.call(static_costmap_toggle_call);
  }

   krClient.call(csq);
  
  atIt = find_if(csq.response.answer.fluents.begin(), csq.response.answer.fluents.end(), IsFluentAt());
                    
  if(!error && atIt != csq.response.answer.fluents.end()) 
    failed = initialPosition == atIt->variables[0];
  
}

static ActionFactory gothroughFactory(new GoThrough(""));
  
  
}
