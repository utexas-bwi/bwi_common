
#ifndef bwi_krexec_RemoteReasoner_h__guard
#define bwi_krexec_RemoteReasoner_h__guard

#include "actasp/AspAtom.h"
#include "actasp/AspKR.h"
#include "actasp/reasoners/Reasoner.h"
#include <actasp/reasoners/FilteringReasoner.h>
#include <actasp/FilteringQueryGenerator.h>

namespace bwi_krexec {
  
  class RemoteReasoner : public actasp::FilteringKR {
  public:
    
  RemoteReasoner(actasp::FilteringQueryGenerator *actualReasoner,unsigned int max_n,const actasp::ActionSet& allActions);
    
  actasp::ActionSet availableActions() const throw();
  
  actasp::AnswerSet currentStateQuery(const std::vector<actasp::AspRule>& query) const throw();
  
  bool updateFluents(const std::vector<actasp::AspFluent> &observations) throw();
  
  bool isPlanValid(const actasp::AnswerSet& plan, const std::vector<actasp::AspRule>& goal)  const throw();
  
  std::vector< actasp::AnswerSet > computeAllPlans(const std::vector<actasp::AspRule>& goal, double suboptimality) const throw (std::logic_error);
  
  void resetCurrentState() throw();
  
  actasp::AnswerSet computePlan(const std::vector<actasp::AspRule>& goal) const throw (std::logic_error);
  
  actasp::GraphPolicy* computePolicy(const std::vector<actasp::AspRule>& goal, double suboptimality) const throw (std::logic_error);
  
  actasp::AnswerSet filterState(const std::vector<actasp::AnswerSet>& plans, const std::vector<actasp::AspRule>& goals);
  
  std::list< std::list<actasp::AspAtom> > query(const std::string &queryString, unsigned int timeStep) const throw();
  
  private:
    actasp::FilteringReasoner local;
  
  };
}

#endif
