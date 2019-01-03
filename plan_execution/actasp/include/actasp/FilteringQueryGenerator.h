#pragma once

#include <actasp/QueryGenerator.h>

#include <actasp/AnswerSet.h>

namespace actasp {
  

struct FilteringQueryGenerator : public QueryGenerator{
  
  virtual std::list<actasp::AnswerSet> filteringQuery(const AnswerSet& currentState, const AnswerSet& plan, const std::vector<actasp::AspFluentRule>& goals)=0;
  
};



}



  
  