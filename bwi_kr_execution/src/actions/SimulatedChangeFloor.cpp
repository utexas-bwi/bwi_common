#include "SimulatedChangeFloor.h"

#include <boost/foreach.hpp>

#include "ActionFactory.h"
#include "plan_execution/StaticFacts.h"

#include "plan_execution/UpdateFluents.h"
#include "ros/console.h"
#include "ros/ros.h"

#include <actionlib/client/simple_action_client.h>
#include <bwi_msgs/DoorHandlerInterface.h>
#include <bwi_msgs/LogicalActionAction.h>
#include <bwi_msgs/ResolveChangeFloor.h>
#include <bwi_msgs/RobotTeleporterInterface.h>

namespace bwi_krexec {

SimulatedChangeFloor::SimulatedChangeFloor() :
             robot_teleported(false),
             done(false),
             failed(false) {}

void SimulatedChangeFloor::run() {

  if (!done) {
  
    // Get the doors for this elevator.
    std::string facing_door;
    std::list<actasp::AspAtom> static_facts = plan_exec::StaticFacts::staticFacts(); 
    BOOST_FOREACH(const actasp::AspAtom fact, static_facts) {
      if (fact.getName() == "hasdoor") {
        std::vector<std::string> params = fact.getParameters();
        if (params[0] == dest_room) {
          // NOTE: This makes the assumption that an elevator room only has a single door, which is true for GDC.
          facing_door = params[1];
          break;
        }
      }
    }

    if (facing_door.empty()) {
      ROS_ERROR_STREAM("Unable to retrieve door we're facing for destination " << dest_room << ". Cannot complete action!");
      failed = true;
      done = true;
    } else {

      if (!robot_teleported) {

        // Resolve the location to which the robot needs to be teleported to.
        ros::NodeHandle n;
        ros::ServiceClient change_floor_resolution_client = 
          n.serviceClient<bwi_msgs::ResolveChangeFloor>("resolve_change_floor");
        change_floor_resolution_client.waitForExistence();

        bwi_msgs::ResolveChangeFloor rcf;
        rcf.request.new_room = dest_room;
        rcf.request.facing_door = facing_door;
        if (change_floor_resolution_client.call(rcf) && rcf.response.success) {

          // Teleport robot to resolved location.
          ros::ServiceClient robot_teleporter_client = 
            n.serviceClient<bwi_msgs::RobotTeleporterInterface>("teleport_robot");
          robot_teleporter_client.waitForExistence();

          bwi_msgs::RobotTeleporterInterface rti;
          rti.request.pose = rcf.response.pose.pose.pose;

          if (robot_teleporter_client.call(rti) && rti.response.success) {
            robot_teleported = true;
            robot_teleportation_start_time = ros::Time::now();
          } else {
            ROS_ERROR_STREAM("Failed robot teleportation to pose " << rti.request.pose);
            failed = true;
            done = true;
          }
        } else {
          ROS_ERROR_STREAM("Change floor resolution failed for room " << dest_room << " and door " << facing_door);
          failed = true;
          done = true;
        }

      } else {

        if (ros::Time::now() - robot_teleportation_start_time > ros::Duration(10.0f)) {

          // Open Elevator door.
          ros::NodeHandle n;
          ros::ServiceClient door_client = n.serviceClient<bwi_msgs::DoorHandlerInterface> ("/update_doors");
          door_client.waitForExistence();

          bwi_msgs::DoorHandlerInterface dhi;

          dhi.request.all_doors = false;
          dhi.request.door = facing_door;
          dhi.request.open = true;
          dhi.request.open_timeout = 30.0f;

          door_client.call(dhi);

          // Attempt to change the robot's location to this floor and location.
          boost::shared_ptr<actionlib::SimpleActionClient<bwi_msgs::LogicalActionAction> > lnac;
          lnac.reset(new actionlib::SimpleActionClient<bwi_msgs::LogicalActionAction>("execute_logical_goal",
                                                                                                           true));
          lnac->waitForServer();
          bwi_msgs::LogicalActionGoal goal;
          goal.command.name = "changefloor";
          goal.command.value.push_back(dest_room);
          goal.command.value.push_back(facing_door);
          lnac->sendGoal(goal);
          lnac->waitForResult();

          // Update knowledge base to reflect change in position.
          ros::ServiceClient krClient = n.serviceClient<plan_execution::UpdateFluents> ( "update_fluents" );
          krClient.waitForExistence();
          plan_execution::UpdateFluents uf;

          plan_execution::AspFluent open_door;
          open_door.name = "open";
          open_door.variables.push_back(facing_door);

          plan_execution::AspFluent face_door;
          face_door.name = "facing";
          face_door.variables.push_back(facing_door);

          plan_execution::AspFluent beside_door;
          beside_door.name = "beside";
          beside_door.variables.push_back(facing_door);

          plan_execution::AspFluent at_loc;
          at_loc.name = "at";
          at_loc.variables.push_back(dest_room);

          uf.request.fluents.push_back(open_door);
          uf.request.fluents.push_back(face_door);
          uf.request.fluents.push_back(beside_door);
          uf.request.fluents.push_back(at_loc);
          krClient.call(uf);

          done = true;
        }
      }
    }
  }

}

bool SimulatedChangeFloor::hasFinished() const {
  return done;
}

bool SimulatedChangeFloor::hasFailed() const {
  return failed;
}

actasp::Action *SimulatedChangeFloor::cloneAndInit(const actasp::AspFluent & fluent) const {
  SimulatedChangeFloor *other = new SimulatedChangeFloor();
  other->dest_room = fluent.getParameters().at(0);
  return other;
}

std::vector<std::string> SimulatedChangeFloor::getParameters() const {
  return std::vector<std::string>(1, dest_room);
}

//if you want the action to be available only in simulation, or only
//on the robot, use the constructor that also takes a boolean.
ActionFactory simulatedChangeFloor(new SimulatedChangeFloor(), true);

}
