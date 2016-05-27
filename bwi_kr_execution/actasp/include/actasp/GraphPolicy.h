#ifndef actasp_GraphPolicy_h__guard
#define actasp_GraphPolicy_h__guard

#include <actasp/PartialPolicy.h>

#include <actasp/AspFluent.h>
#include <actasp/state_utils.h>

#include <map>
#include <set>
#include <list>

namespace actasp {
  
struct GraphPolicy : public PartialPolicy {
public:
  
  GraphPolicy(const ActionSet& actions);
  
  ActionSet actions(const std::set<AspFluent>& state) const throw();
  
  void merge(const AnswerSet& plan) throw(std::logic_error);
  
  void merge(const PartialPolicy* otherPolicy);
  
  void merge(const GraphPolicy* otherPolicy);
  
  bool empty() const throw();
  
  std::vector<actasp::AnswerSet> plansFrom(const std::set<AspFluent>& state) throw();
  
private:
  
  typedef std::map<std::set<AspFluent>, ActionSet, StateComparator<AspFluent> >  PolicyMap;
  typedef std::list< std::list< AspFluent> > PlanList; 
  typedef std::list< std::pair< PlanList::const_iterator, std::list< AspFluent>::const_iterator> > PlanReference;
  typedef std::map<std::set<AspFluent>, PlanReference , StateComparator<AspFluent> > PlanIndex;
  
  PolicyMap policy;
  ActionSet allActions;
  PlanList plans;
  PlanIndex planIndex;
  
};
  
}

#endif
