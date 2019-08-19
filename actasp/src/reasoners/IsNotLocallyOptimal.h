#pragma once

#include "LexComparator.h"

#include <list>
#include <set>
#include <functional>

#include <actasp/asp/AspFluent.h>
#include <actasp/AnswerSet.h>

namespace actasp {

class AnswerSet;
  
struct IsNotLocallyOptimal : public std::unary_function<const AnswerSet&, bool> {

  typedef std::set<Plan, LexComparator> PlanSet;
  
  IsNotLocallyOptimal(const PlanSet* good, PlanSet* bad, const std::set<std::string>& allActions,
                      unsigned int shortestLength, bool planFitered);

  bool operator()(const Plan &plan);

  std::vector<AspFluent>::const_iterator findFirstSuspiciousAction(const Plan &) const;

  bool validFrom(const Plan &planCleaned, std::vector<AspFluent>::const_iterator firstSuspect) const;
  
  bool checkPlanValidity(const std::list<AspFluentRef>&) const;
  
  bool checkSectionWithLength(const std::list<AspFluentRef>& planCleaned,
                              std::list<AspFluentRef>::const_iterator firstSuspect,
                               int length) const;
  
  bool hasLoops(const AnswerSet& plan) const;


  
private:
  const PlanSet* good;
  PlanSet* bad;
  const std::set<std::string>& allActions;
  unsigned int shortestLength;
  bool planFiltered;

  bool
  validFrom(const std::list<AspFluentRef> &planCleaned,
            std::list<actasp::AspFluentRef>::const_iterator firstSuspect) const;
};
  
}


