#include "OpenSimulatedDoor.h"

#include "bwi_msgs/DoorHandlerInterface.h"

using namespace std;
using namespace ros;

namespace bwi_krexec {


OpenSimulatedDoor::OpenSimulatedDoor(const int door_id, knowledge_rep::LongTermMemoryConduit &ltmc) : 
  ltmc(ltmc), door_id(door_id), door_entity(door_id, ltmc), door_name(), done(false), failed(false), requestSent(false) {}

bool OpenSimulatedDoor::checkDoorOpen() {
  bwi_msgs::CheckBool open_srv;
  doorStateClient.call(open_srv);

  if (open_srv.response.value) {
    ROS_INFO_STREAM("Door " << door_name << " is open");
    door_entity.addAttribute("is_open", true);

    return true;
  }

  return false;
}

void OpenSimulatedDoor::run() {
  NodeHandle n;

  if (!requestSent) {

    if (!door_entity.isValid()) {
      failed = true;
      return;
    }
    auto attrs = door_entity.getAttributes("name");
    
    if (attrs.size() != 1) {
      failed = true;
      return;
    }

    door_name = attrs.at(0).getStringValue();

    doorStateClient = n.serviceClient<bwi_msgs::CheckBool>("/sense_door_state");

    if (checkDoorOpen()) {
      done = true;
      return;
    }

    ServiceClient doorClient = n.serviceClient<bwi_msgs::DoorHandlerInterface> ("/update_doors");
    doorClient.waitForExistence();

    bwi_msgs::DoorHandlerInterface dhi;

    dhi.request.all_doors = false;
    dhi.request.door = door_name;
    dhi.request.open = true;

    doorClient.call(dhi);

    requestSent = true;
  }

  done = checkDoorOpen();

}


actasp::Action* OpenSimulatedDoor::cloneAndInit(const actasp::AspFluent& fluent) const {
  return nullptr;
}

std::vector<std::string> OpenSimulatedDoor::getParameters() const {
  return {to_string(door_id)};
}

}
