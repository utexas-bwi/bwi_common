#ifndef actasp_QueryGenerator_h__guard
#define actasp_QueryGenerator_h__guard

#include <actasp/AspRule.h>

#include <list>
#include <vector>
#include <string>
#include <set>

namespace actasp {
  
  class AnswerSet;
  class AspFluent;
  class AspAtom;

struct QueryGenerator {
  
  virtual std::list<actasp::AnswerSet> minimalPlanQuery(const std::vector<actasp::AspRule>& goalRules,
                                         bool filterActions, 
                                         unsigned int  max_plan_length,
                                         unsigned int answerset_number) const noexcept = 0;
                                         
  virtual std::list<actasp::AnswerSet> lengthRangePlanQuery(const std::vector<actasp::AspRule>& goalRules,
                                         bool filterActions, 
                                         unsigned int min_plan_length,
                                         unsigned int  max_plan_length,
                                         unsigned int answerset_number) const noexcept = 0;

  virtual actasp::AnswerSet optimalPlanQuery(const std::vector<actasp::AspRule>& goalRules,
                                         bool filterActions,
                                         unsigned int  max_plan_length,
                                         unsigned int answerset_number,
                                         bool minimum) const noexcept = 0;
  
  virtual std::list<actasp::AnswerSet> monitorQuery(const std::vector<actasp::AspRule>& goalRules,
                                         const AnswerSet& plan) const noexcept = 0;


  virtual AnswerSet currentStateQuery(const std::vector<actasp::AspRule>& query) const noexcept = 0;
  
  virtual void setCurrentState(const std::set<actasp::AspFluent>& newState) = 0;


  virtual std::list<actasp::AnswerSet> genericQuery(const std::vector<actasp::AspRule>& query,
      unsigned int timestep,
      const std::string& fileName,
      unsigned int answerSetsNumber) const noexcept = 0;
      
  virtual std::list< std::list<AspAtom> > genericQuery(const std::string& query,
      unsigned int timestep,
      const std::string& fileName,
      unsigned int answerSetsNumber) const noexcept = 0;

};

}


#endif
