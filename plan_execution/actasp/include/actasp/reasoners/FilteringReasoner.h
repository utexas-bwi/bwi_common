#pragma once

#include <actasp/FilteringKR.h>
#include <actasp/reasoners/Reasoner.h>

namespace actasp {

class FilteringQueryGenerator;

struct FilteringReasoner : public FilteringKR, public Reasoner {

  FilteringReasoner(FilteringQueryGenerator *queryGenerator,unsigned int max_n,const ActionSet& allActions);


  AnswerSet computePlan(const std::vector<actasp::AspRule>& goal) const noexcept(false) override {
    return this->Reasoner::computePlan(goal);
  }

  std::vector< AnswerSet > computeAllPlans(const std::vector<actasp::AspRule>& goal, double suboptimality) const noexcept(false) override {
    return this->Reasoner::computeAllPlans(goal,suboptimality);
  }

  ActionSet availableActions() const noexcept override {
    return this->Reasoner::availableActions();
  }

  AnswerSet currentStateQuery(const std::vector<actasp::AspRule>& query) const noexcept override {
    return  this->Reasoner::currentStateQuery(query);
  }


  bool isPlanValid(const AnswerSet& plan, const std::vector<actasp::AspRule>& goal)  const noexcept override {
    return this->Reasoner::isPlanValid(plan,goal);
  }

  std::list< std::list<AspAtom> > query(const std::string &queryString, unsigned int timestep) const noexcept {
    return this->Reasoner::query(queryString,timestep);
  }
  
  GraphPolicy* computePolicy(const std::vector<actasp::AspRule>& goal, double suboptimality) const noexcept(false) override;

  AnswerSet filterState(const std::vector<actasp::AnswerSet>& plans, const std::vector<actasp::AspRule>& goals) override;

private:
  FilteringQueryGenerator *clingo;
};



}



