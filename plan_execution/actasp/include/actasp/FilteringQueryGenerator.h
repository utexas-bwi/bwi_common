#pragma once

#include <actasp/QueryGenerator.h>

#include <actasp/AnswerSet.h>

namespace actasp {
  

struct FilteringQueryGenerator : public QueryGenerator{

  virtual std::vector<Plan>
  filteringQuery(const AnswerSet &currentState, const Plan &plan, const std::vector<actasp::AspRule> &goals,
                 const std::vector<AspFact> *knowledge) const noexcept = 0;
  
};



}



  
  