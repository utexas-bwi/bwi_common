#pragma once

#include <actasp/FilteringKR.h>
#include <actasp/reasoners/Reasoner.h>

namespace actasp {

class FilteringQueryGenerator;

struct FilteringReasoner : public FilteringKR, public Reasoner {

  FilteringReasoner(FilteringQueryGenerator *queryGenerator,unsigned int max_n,const std::set<std::string>& allActions);


  Plan computePlan(const std::vector<AspRule> &goal) const noexcept(false) override {
    return this->Reasoner::computePlan(goal);
  }

  std::vector<Plan>
  computeAllPlans(const std::vector<AspRule> &goal, double suboptimality) const noexcept(false) override {
    return this->Reasoner::computeAllPlans(goal,suboptimality);
  }

  ActionSet availableActions() const noexcept override {
    return this->Reasoner::availableActions();
  }

  AnswerSet currentStateQuery(const std::vector<AspRule> &query) const noexcept override {
    return  this->Reasoner::currentStateQuery(query);
  }


  bool isPlanValid(const Plan &plan, const std::vector<AspRule> &goal) const noexcept override {
    return this->Reasoner::isPlanValid(plan,goal);
  }

  std::vector<AnswerSet> query(const std::vector<AspRule> &query, unsigned int timestep) const noexcept {
    return this->Reasoner::query(query,timestep);
  }

  GraphPolicy *computePolicy(const std::vector<AspRule> &goal, double suboptimality) const noexcept(false) override;

  AnswerSet filterState(const std::vector<Plan> &plans, const std::vector<AspRule> &goals) override;

private:
  FilteringQueryGenerator *clingo;
};



}



