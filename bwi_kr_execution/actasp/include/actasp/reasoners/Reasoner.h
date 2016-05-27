
#ifndef actasp_Reasoner_h__guard
#define actasp_Reasoner_h__guard

#include <actasp/AspKR.h>

#include <actasp/AspFluent.h>

namespace actasp {

class QueryGenerator;
class PartialPolicy;
  
struct Reasoner : public AspKR {
  
  Reasoner(QueryGenerator *actualReasoner,unsigned int max_n,const ActionSet& allActions);
  
  ActionSet availableActions() const throw();
  
  AnswerSet currentStateQuery(const std::vector<actasp::AspRule>& query) const throw();
  
  bool updateFluents(const std::vector<actasp::AspFluent> &observations) throw();
  
  bool isPlanValid(const AnswerSet& plan, const std::vector<actasp::AspRule>& goal)  const throw();
  
  void resetCurrentState() throw();

  void setCurrentState(const std::set<actasp::AspFluent>& newState) throw();
  
  AnswerSet computePlan(const std::vector<actasp::AspRule>& goal) const throw (std::logic_error) ;
  
  std::vector< AnswerSet > computeAllPlans(const std::vector<actasp::AspRule>& goal, double suboptimality) const throw (std::logic_error);

  AnswerSet computeOptimalPlan(const std::vector<actasp::AspRule>& goal, bool filterActions, double suboptimality, bool minimum) const throw (std::logic_error);
  
  virtual PartialPolicy* computePolicy(const std::vector<actasp::AspRule>& goal, double suboptimality) const throw (std::logic_error);
  
  std::list< std::list<AspAtom> > query(const std::string &queryString, unsigned int timestep) const throw();
  
  void setMaxTimeStep(unsigned int max_n) throw() {
    this->max_n = max_n;
  }
  
  virtual ~Reasoner() {}
  
protected:
  QueryGenerator *clingo;
  unsigned int max_n;
  ActionSet allActions;
  
  void computePolicyHelper(const std::vector<actasp::AspRule>& goal, double suboptimality, PartialPolicy* p) const throw (std::logic_error);
  
};
  
}

#endif