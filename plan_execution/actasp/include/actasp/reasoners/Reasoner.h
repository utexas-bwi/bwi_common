#pragma once

#include <actasp/AspKR.h>

#include <actasp/AspFluent.h>

namespace actasp {

class QueryGenerator;
class PartialPolicy;
  
struct Reasoner : public AspKR {
  
  Reasoner(QueryGenerator *queryGenerator,unsigned int max_n,const std::set<std::string>& allActions);
  
  ActionSet availableActions() const noexcept override;
  
  AnswerSet currentStateQuery(const std::vector<actasp::AspFluentRule>& query) const noexcept override;

  bool isPlanValid(const AnswerSet& plan, const std::vector<actasp::AspFluentRule>& goal)  const noexcept override;

  AnswerSet computePlan(const std::vector<actasp::AspFluentRule>& goal) const noexcept(false) override;
  
  std::vector< AnswerSet > computeAllPlans(const std::vector<actasp::AspFluentRule>& goal, double suboptimality) const noexcept(false) override;

  AnswerSet computeOptimalPlan(const std::vector<actasp::AspFluentRule>& goal, bool filterActions, double suboptimality, bool minimum) const noexcept(false);

  PartialPolicy* computePolicy(const std::vector<actasp::AspFluentRule>& goal, double suboptimality) const noexcept(false) override;
  
  std::list<actasp::AnswerSet> query(const std::vector<actasp::AspFluentRule> &query, unsigned int timestep) const noexcept override;
  
  void setMaxTimeStep(unsigned int max_n) noexcept {
    this->max_n = max_n;
  }

  ~Reasoner() override = default;

protected:
  QueryGenerator *clingo;
  unsigned int max_n;
  std::set<std::string> allActions;
  
  void computePolicyHelper(const std::vector<actasp::AspFluentRule>& goal, double suboptimality, PartialPolicy* p) const noexcept(false);
  
};
  
}

