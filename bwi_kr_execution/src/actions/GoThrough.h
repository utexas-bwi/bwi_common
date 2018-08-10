#ifndef bwi_actexec_GoThrough_h__guard
#define bwi_actexec_GoThrough_h__guard

#include "LogicalNavigation.h"
#include "../BwiResourceManager.h"

namespace bwi_krexec {


class GoThrough : public LogicalNavigation {
public:
  GoThrough(const std::string &door_name, knowledge_rep::LongTermMemoryConduit &ltmc);

  Action *cloneAndInit(const actasp::AspFluent &fluent) const override {
    return nullptr;
  }

  Action *clone() const override { return nullptr; }

  int paramNumber() const override {
    return 1;
  }

  std::vector<std::string> getParameters() const override;

  std::vector<std::string> prepareGoalParameters() const override;

  static std::unique_ptr<actasp::Action> create(const actasp::AspFluent & fluent, actasp::ResourceManager &resource_manager) {
    auto door_name = fluent.getParameters().at(0);
    auto& resource_manager_cast = dynamic_cast<BwiResourceManager&>(resource_manager);
    return std::unique_ptr<actasp::Action>(new bwi_krexec::GoThrough(door_name, resource_manager_cast.ltmc));
  }

private:
  const std::string &door_name;

  void onFinished(bool success, ResultConstPtr result) override;
};
}

#endif
