#pragma once

#include <actasp/Planner.h>
#include <actasp/asp/AspRule.h>
#include <actasp/MultiPolicy.h>
#include <actasp/GraphPolicy.h>

#include <vector>
#include <list>

namespace actasp {

class Action;

struct MultiPlanner : public actasp::Planner {

  virtual std::vector<Plan>
  computeAllPlans(const std::vector<actasp::AspRule> &goal, double suboptimality) const noexcept(false) = 0;

virtual PartialPolicy* computePolicy(const std::vector<actasp::AspRule>& goal, double suboptimality) const noexcept(false)=0;

  ~MultiPlanner() override = default;

};

}


