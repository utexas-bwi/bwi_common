#pragma once

#include <actasp/AspKR.h>

#include <actasp/asp/AspFluent.h>

namespace actasp {

class Solver;
class PartialPolicy;
  
struct Reasoner : public AspKR {
  
  Reasoner(Solver *queryGenerator, unsigned int max_n, const std::set<std::string>& allActions);
  
  ActionSet availableActions() const noexcept override;

  AnswerSet currentStateQuery(const std::vector<AspRule> &query) const noexcept override;

  bool isPlanValid(const Plan &plan, const std::vector<AspRule> &goal) const noexcept override;

  Plan computePlan(const std::vector<AspRule> &goal) const noexcept(false) override;

  std::vector<Plan>
  computeAllPlans(const std::vector<AspRule> &goal, double suboptimality) const noexcept(false) override;

  AnswerSet computeOptimalPlan(const std::vector<AspRule> &goal, bool filterActions, double suboptimality,
                               bool minimum) const noexcept(false);

  PartialPolicy *computePolicy(const std::vector<AspRule> &goal, double suboptimality) const noexcept(false) override;

  std::vector<AnswerSet> query(const std::vector<AspRule> &query, unsigned int timestep) const noexcept override;
  
  void setMaxTimeStep(unsigned int max_n) noexcept {
    this->max_n = max_n;
  }

  ~Reasoner() override = default;

protected:
  Solver *clingo;
  unsigned int max_n;
  std::set<std::string> allActions;

  void
  computePolicyHelper(const std::vector<AspRule> &goal, double suboptimality, PartialPolicy *p) const noexcept(false);
  
};
  
}

