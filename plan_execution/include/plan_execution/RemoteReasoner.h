#pragma once

#include "actasp/AspAtom.h"
#include "actasp/AspKR.h"
#include "actasp/reasoners/Reasoner.h"
#include <actasp/reasoners/FilteringReasoner.h>
#include <actasp/FilteringQueryGenerator.h>

namespace plan_exec {
  
  class RemoteReasoner : public actasp::FilteringKR {
  public:
    
  RemoteReasoner(actasp::FilteringQueryGenerator *actualReasoner,unsigned int max_n,const actasp::ActionSet& allActions);
    
  actasp::ActionSet availableActions() const noexcept;
  
  actasp::AnswerSet currentStateQuery(const std::vector<actasp::AspRule>& query) const noexcept;
  
  bool updateFluents(const std::vector<actasp::AspFluent> &observations) noexcept;
  
  bool isPlanValid(const actasp::AnswerSet& plan, const std::vector<actasp::AspRule>& goal) const noexcept;
  
  std::vector< actasp::AnswerSet > computeAllPlans(const std::vector<actasp::AspRule>& goal, double suboptimality) const;
  
  void resetCurrentState() noexcept;
  
  actasp::AnswerSet computePlan(const std::vector<actasp::AspRule>& goal) const;
  
  actasp::GraphPolicy* computePolicy(const std::vector<actasp::AspRule>& goal, double suboptimality) const;
  
  actasp::AnswerSet filterState(const std::vector<actasp::AnswerSet>& plans, const std::vector<actasp::AspRule>& goals);
  
  std::list< std::list<actasp::AspAtom> > query(const std::string &queryString, unsigned int timeStep) const noexcept;
  
  private:
    actasp::FilteringReasoner local;
  
  };
}
