#pragma once

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

  ActionSet actions(const std::set<AspFluent>& state) const noexcept;

  void merge(const AnswerSet& plan);
  void merge(const PartialPolicy* otherPolicy);
  void merge(const MultiPolicy* otherPolicy);

  bool empty() const noexcept;

private:
  std::map<std::set<AspFluent>, ActionSet, StateComparator<AspFluent> > policy;
  ActionSet allActions;

};

}


