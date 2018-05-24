#ifndef actasp_FilteringReasoner_h__guard
#define actasp_FilteringReasoner_h__guard

#include <actasp/FilteringKR.h>
#include <actasp/reasoners/Reasoner.h>

namespace actasp {

class FilteringQueryGenerator;

struct FilteringReasoner : public FilteringKR, public Reasoner {

  FilteringReasoner(FilteringQueryGenerator *actualReasoner,unsigned int max_n,const ActionSet& allActions);


  AnswerSet computePlan(const std::vector<actasp::AspRule>& goal) const throw (std::logic_error) {
    return this->Reasoner::computePlan(goal);
  }

  std::vector< AnswerSet > computeAllPlans(const std::vector<actasp::AspRule>& goal, double suboptimality) const throw (std::logic_error) {
    return this->Reasoner::computeAllPlans(goal,suboptimality);
  }

  ActionSet availableActions() const noexcept {
    return this->Reasoner::availableActions();
  }

  AnswerSet currentStateQuery(const std::vector<actasp::AspRule>& query) const noexcept {
    return  this->Reasoner::currentStateQuery(query);
  }

  bool updateFluents(const std::vector<actasp::AspFluent> &observations) noexcept {
    return this->Reasoner::updateFluents(observations);
  }

  bool isPlanValid(const AnswerSet& plan, const std::vector<actasp::AspRule>& goal)  const noexcept {
    return this->Reasoner::isPlanValid(plan,goal);
  }

  void resetCurrentState() noexcept {
    this->Reasoner::resetCurrentState();
  }

  std::list< std::list<AspAtom> > query(const std::string &queryString, unsigned int timestep) const noexcept {
    return this->Reasoner::query(queryString,timestep);
  }
  
  GraphPolicy* computePolicy(const std::vector<actasp::AspRule>& goal, double suboptimality) const throw (std::logic_error);

  AnswerSet filterState(const std::vector<actasp::AnswerSet>& plans, const std::vector<actasp::AspRule>& goals);

private:
  FilteringQueryGenerator *clingo;
};



}


#endif
