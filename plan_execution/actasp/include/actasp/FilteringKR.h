#pragma once

#include <actasp/AspKR.h>


namespace actasp {


struct FilteringKR : public actasp::AspKR {

  virtual GraphPolicy* computePolicy(const std::vector<actasp::AspRule>& goal, double suboptimality) const noexcept(false) = 0;
  
  virtual AnswerSet filterState(const std::vector<actasp::AnswerSet>& plans, const std::vector<actasp::AspRule>& goals) = 0;

  ~FilteringKR() override {}
};
  
}

