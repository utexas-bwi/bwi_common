#ifndef actasp_FilteringQueryGenerator_h__guard
#define actasp_FilteringQueryGenerator_h__guard

#include <actasp/QueryGenerator.h>

#include <actasp/AnswerSet.h>

namespace actasp {
  

struct FilteringQueryGenerator : public QueryGenerator{
  
  virtual std::list<actasp::AnswerSet> filteringQuery(const AnswerSet& currentState, const AnswerSet& plan, const std::vector<actasp::AspRule>& goals)=0;
  
};



}


#endif
  
  