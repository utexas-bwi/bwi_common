#pragma once

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
  
  virtual std::list<actasp::AnswerSet> minimalPlanQuery(const std::vector<actasp::AspFluentRule>& goalRules,
                                         unsigned int  max_plan_length,
                                         unsigned int answerset_number, bool actions_only=true
  ) const noexcept = 0;
                                         
  virtual std::list<actasp::AnswerSet> lengthRangePlanQuery(const std::vector<actasp::AspFluentRule>& goalRules,
                                         unsigned int min_plan_length,
                                         unsigned int  max_plan_length,
                                         unsigned int answerset_number, bool actions_only=true
  ) const noexcept = 0;

  virtual actasp::AnswerSet optimalPlanQuery(const std::vector<actasp::AspFluentRule>& goalRules,
                                         unsigned int  max_plan_length,
                                         unsigned int answerset_number, bool actions_only=true) const noexcept = 0;
  
  virtual std::list<actasp::AnswerSet> monitorQuery(const std::vector<actasp::AspFluentRule>& goalRules,
                                         const AnswerSet& plan) const noexcept = 0;


  virtual AnswerSet currentStateQuery(const std::vector<actasp::AspFluentRule>& query) const noexcept = 0;

  virtual std::list<actasp::AnswerSet> genericQuery(const std::vector<actasp::AspFluentRule>& query,
      unsigned int timestep,
      const std::string& fileName,
      unsigned int answerSetsNumber) const noexcept = 0;

  virtual std::set<std::string> get_all_actions() = 0;

};

}



