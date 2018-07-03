#ifndef bwi_actexec_GoThrough_h__guard
#define bwi_actexec_GoThrough_h__guard

#include "LogicalNavigation.h"

namespace bwi_krexec {


class GoThrough : public LogicalNavigation {
public:
  explicit GoThrough();

  bool hasFailed() const override;

  Action *cloneAndInit(const actasp::AspFluent &fluent) const override {
    auto action = new GoThrough();
    action->door_name = fluent.getParameters().at(0);
    return new GoThrough();
  }

  Action *clone() const override { return new GoThrough(*this); }

  int paramNumber() const override {
    return 1;
  }

  std::vector<std::string> getParameters() const override;

  std::vector<std::string> prepareGoalParameters() const override;


private:
  bool failed;
  std::string door_name;

  void onFinished(bool success, ResultConstPtr result) override;
};
}

#endif
