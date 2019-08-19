#pragma once

#include <actasp/AspKR.h>


namespace actasp {


struct FilteringKR : public actasp::AspKR {

  virtual GraphPolicy* computePolicy(const std::vector<actasp::AspRule>& goal, double suboptimality) const noexcept(false) = 0;

  virtual AnswerSet filterState(const std::vector<Plan> &plans, const std::vector<AspRule> &goals) = 0;

  ~FilteringKR() override = default;
};
  
}

