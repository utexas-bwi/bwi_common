#include "OpenSimulatedDoor.h"

#include "bwi_msgs/DoorHandlerInterface.h"
#include <knowledge_representation/Entity.h>

using namespace std;
using namespace ros;

namespace bwi_krexec {


OpenSimulatedDoor::OpenSimulatedDoor(const int door_id, knowledge_rep::LongTermMemoryConduit &ltmc) : 
  ltmc(ltmc), door_id(door_id), door_name(), done(false), failed(false), requestSent(false) {}

void OpenSimulatedDoor::run() {
  NodeHandle n;

  if (!requestSent) {
    knowledge_rep::Entity location(door_id, ltmc);
    if (!location.is_valid()) {
      failed = true;
      return;
    }
    auto attrs = location.get_attributes("name");
    
    if (attrs.size() != 1) {
      failed = true;
      return;
    }

    door_name = attrs.at(0).get_string_value();

    ServiceClient doorClient = n.serviceClient<bwi_msgs::DoorHandlerInterface> ("/update_doors");
    doorClient.waitForExistence();

    bwi_msgs::DoorHandlerInterface dhi;

    dhi.request.all_doors = false;
    dhi.request.door = door_name;
    dhi.request.open = true;

    doorClient.call(dhi);

    requestSent = true;

    doorStateClient = n.serviceClient<bwi_msgs::CheckBool>("/sense_door_state");
  }

  
  bwi_msgs::CheckBool open_srv;
  doorStateClient.call(open_srv);

  done = open_srv.response.value;

  if (done) {
    ROS_INFO_STREAM("Door " << door_name << " is open");
  }

}


actasp::Action* OpenSimulatedDoor::cloneAndInit(const actasp::AspFluent& fluent) const {
  return nullptr;
}

std::vector<std::string> OpenSimulatedDoor::getParameters() const {
  return {to_string(door_id)};
}

}
