#ifndef bwi_actexec_GoThrough_h__guard
#define bwi_actexec_GoThrough_h__guard

#include "LogicalNavigation.h"
#include "../BwiResourceManager.h"

namespace bwi_krexec {


class GoThrough : public LogicalNavigation {
public:
  GoThrough(const uint door_id, knowledge_rep::LongTermMemoryConduit &ltmc);

  Action *cloneAndInit(const actasp::AspFluent &fluent) const override {
    return nullptr;
  }

  Action *clone() const override { return nullptr; }

  int paramNumber() const override {
    return 1;
  }

  std::vector<std::string> getParameters() const override;

  boost::optional<std::vector<std::string> > prepareGoalParameters() const override;

  static std::unique_ptr<actasp::Action> create(const actasp::AspFluent & fluent, actasp::ResourceManager &resource_manager) {
    auto door_id = std::atoi(fluent.getParameters().at(0).c_str());
    auto& resource_manager_cast = dynamic_cast<BwiResourceManager&>(resource_manager);
    return std::unique_ptr<actasp::Action>(new bwi_krexec::GoThrough(door_id, resource_manager_cast.ltmc));
  }

private:
  uint door_id;

  void onFinished(bool success, const bwi_msgs::LogicalNavResult &result) override;
};
}

#endif
