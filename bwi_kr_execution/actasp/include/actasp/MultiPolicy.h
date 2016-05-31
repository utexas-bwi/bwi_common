#ifndef actasp_MultiPolicy_h__guard
#define actasp_MultiPolicy_h__guard

#include <actasp/PartialPolicy.h>

#include <actasp/AspFluent.h>
#include <actasp/state_utils.h>

#include <set>
#include <map>
#include <stdexcept>

namespace actasp {

class MultiPolicy : public PartialPolicy {
public:

  MultiPolicy(const ActionSet& actions);

  ActionSet actions(const std::set<AspFluent>& state) const throw();

  void merge(const AnswerSet& plan) throw(std::logic_error);
  void merge(const PartialPolicy* otherPolicy);
  void merge(const MultiPolicy* otherPolicy);

  bool empty() const throw();

private:
  std::map<std::set<AspFluent>, ActionSet, StateComparator<AspFluent> > policy;
  ActionSet allActions;

};

}

#endif
