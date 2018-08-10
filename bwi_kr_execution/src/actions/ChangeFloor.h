#ifndef bwi_krexec_ChangeFloor_h__guard
#define bwi_krexec_ChangeFloor_h__guard

#include <boost/shared_ptr.hpp>
#include "LogicalNavigation.h"
#include "actasp/Action.h"
#include "CallGUI.h"
#include "../BwiResourceManager.h"

namespace bwi_krexec {
  
struct ChangeFloor : public LogicalNavigation {

  ChangeFloor(const std::string &dest_room, knowledge_rep::LongTermMemoryConduit &ltmc);

  int paramNumber() const {return 1;}

  actasp::Action *cloneAndInit(const actasp::AspFluent & fluent) const;
  
  actasp::Action *clone() const {return nullptr;}

  std::vector<std::string> prepareGoalParameters() const override;

  static std::unique_ptr<actasp::Action> create(const actasp::AspFluent & fluent, actasp::ResourceManager &resource_manager) {
    auto dest_room = fluent.getParameters().at(0);
    auto& resource_manager_cast = dynamic_cast<BwiResourceManager&>(resource_manager);
    return std::unique_ptr<actasp::Action>(new bwi_krexec::ChangeFloor(dest_room, resource_manager_cast.ltmc));
  }

private:
 
std::vector<std::string> getParameters() const;

const std::string &dest_room;

bool asked;
bool done;
bool failed;

boost::shared_ptr<CallGUI> askToChangeFloor;

};
  
  
}

#endif
