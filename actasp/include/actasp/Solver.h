#pragma once

#include <actasp/asp/AspRule.h>
#include <actasp/asp/AspProgram.h>

#include <list>
#include <vector>
#include <string>
#include <set>

namespace Clingo {
class Model;
}

namespace actasp {
  
  class AnswerSet;
  class AspFluent;
  class AspFunction;

class Plan;

struct Solver {

  virtual std::vector<Plan> minimalPlanQuery(const std::vector<AspRule> &goalRules,
                                             unsigned int max_plan_length,
                                             unsigned int answerset_number, const std::vector<AspFact> *knowledge
  ) const noexcept = 0;

  virtual std::vector<Plan> lengthRangePlanQuery(const std::vector<AspRule> &goalRules,
                                                 unsigned int min_plan_length,
                                                 unsigned int max_plan_length,
                                                 unsigned int answerset_number, const std::vector<AspFact> *knowledge
  ) const noexcept = 0;

  virtual Plan optimalPlanQuery(const std::vector<AspRule> &goalRules,
                                unsigned int max_plan_length,
                                unsigned int answerset_number,
                                const std::vector<AspFact> *knowledge) const noexcept = 0;

  virtual std::vector<Plan> monitorQuery(const std::vector<AspRule> &goalRules,
                                         const Plan &plan, const std::vector<AspFact> *knowledge) const noexcept = 0;


  virtual AnswerSet currentStateQuery(const std::vector<AspRule> &query) const noexcept = 0;

  virtual std::vector<AnswerSet> genericQuery(const std::vector<AspProgram> &programs,
                                              unsigned int min_plan_length,
                                              unsigned int max_plan_length,
                                              unsigned int max_num_plans,
                                              bool shortest_only) const noexcept = 0;

  virtual std::set<std::string> get_all_actions() = 0;

  virtual std::vector<Plan>
  filteringQuery(const AnswerSet &currentState, const Plan &plan, const std::vector<actasp::AspRule> &goals,
                 const std::vector<AspFact> *knowledge) const noexcept = 0;

};

}




