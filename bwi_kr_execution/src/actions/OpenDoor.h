
#ifndef bwi_krexec_OpenDoor_h__guard
#define bwi_krexec_OpenDoor_h__guard

#include "actasp/Action.h"
#include "LogicalNavigation.h"
#include "../BwiResourceManager.h"
#include <ros/ros.h>

#include <string>

namespace bwi_krexec {

class OpenDoor : public actasp::Action{
public:
  OpenDoor(const std::string &door_name, knowledge_rep::LongTermMemoryConduit &ltmc);

  int paramNumber() const override {return 1;}

  std::string getName() const override {return "opendoor";}
  
  void run();
  
  bool hasFinished() const {return done;}
  
  virtual bool hasFailed() const {return failed;}
  
  actasp::Action *cloneAndInit(const actasp::AspFluent & fluent) const;
  
  virtual actasp::Action *clone() const {return nullptr;}

  static std::unique_ptr<actasp::Action> create(const actasp::AspFluent & fluent, actasp::ResourceManager &resource_manager) {
    auto door_name = fluent.getParameters().at(0);
    auto& resource_manager_cast = dynamic_cast<BwiResourceManager&>(resource_manager);
    return std::unique_ptr<actasp::Action>(new bwi_krexec::OpenDoor(door_name, resource_manager_cast.ltmc));
  }


private:
  
 std::vector<std::string> getParameters() const;
 
 std::string door;
 bool done;
 bool asked;
 bool open;
 bool failed;
 ros::Time startTime;
 knowledge_rep::LongTermMemoryConduit &ltmc;

 std::unique_ptr<LogicalNavigation> senseDoor;
 
};

}
 
#endif
