#pragma once

#include <actasp/FilteringKR.h>
#include <actasp/reasoners/Reasoner.h>

namespace actasp {

class FilteringQueryGenerator;

struct FilteringReasoner : public FilteringKR, public Reasoner {

  FilteringReasoner(FilteringQueryGenerator *queryGenerator,unsigned int max_n,const std::set<std::string>& allActions);


  AnswerSet computePlan(const std::vector<actasp::AspFluentRule>& goal) const noexcept(false) override {
    return this->Reasoner::computePlan(goal);
  }

  std::vector< AnswerSet > computeAllPlans(const std::vector<actasp::AspFluentRule>& goal, double suboptimality) const noexcept(false) override {
    return this->Reasoner::computeAllPlans(goal,suboptimality);
  }

  ActionSet availableActions() const noexcept override {
    return this->Reasoner::availableActions();
  }

  AnswerSet currentStateQuery(const std::vector<actasp::AspFluentRule>& query) const noexcept override {
    return  this->Reasoner::currentStateQuery(query);
  }


  bool isPlanValid(const AnswerSet& plan, const std::vector<actasp::AspFluentRule>& goal)  const noexcept override {
    return this->Reasoner::isPlanValid(plan,goal);
  }

  std::list<actasp::AnswerSet> query(const std::vector<actasp::AspFluentRule> &query, unsigned int timestep) const noexcept {
    return this->Reasoner::query(query,timestep);
  }
  
  GraphPolicy* computePolicy(const std::vector<actasp::AspFluentRule>& goal, double suboptimality) const noexcept(false) override;

  AnswerSet filterState(const std::vector<actasp::AnswerSet>& plans, const std::vector<actasp::AspFluentRule>& goals) override;

private:
  FilteringQueryGenerator *clingo;
};



}



