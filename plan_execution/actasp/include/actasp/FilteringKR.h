#pragma once

#include <actasp/AspKR.h>


namespace actasp {


struct FilteringKR : public actasp::AspKR {

  virtual GraphPolicy* computePolicy(const std::vector<actasp::AspFluentRule>& goal, double suboptimality) const noexcept(false) = 0;
  
  virtual AnswerSet filterState(const std::vector<actasp::AnswerSet>& plans, const std::vector<actasp::AspFluentRule>& goals) = 0;

  ~FilteringKR() override {}
};
  
}

