#ifndef actasp_IsNotLocallyOptimal_h__guard
#define actasp_IsNotLocallyOptimal_h__guard

#include "LexComparator.h"

#include <list>
#include <set>
#include <functional>

#include <actasp/AspFluent.h>

namespace actasp {

class AnswerSet;
  
struct IsNotLocallyOptimal : public std::unary_function<const AnswerSet&, bool> {
    
  typedef std::set< std::list <AspFluentRef>, LexComparator > PlanSet;
  
  IsNotLocallyOptimal(const PlanSet* good, PlanSet* bad, const ActionSet& allActions, 
                      unsigned int shortestLength, bool planFitered);
    
  bool operator()(const AnswerSet& plan);
  
  std::list<AspFluentRef> cleanPlan(const AnswerSet& plan) const;
  
  std::list<AspFluentRef>::const_iterator findFirstSuspiciousAction(const std::list<AspFluentRef>&) const; 
  
  bool validFrom(const std::list<AspFluentRef>& planCleaned, std::list<AspFluentRef>::const_iterator firstSuspect) const;
  
  bool checkPlanValidity(const std::list<AspFluentRef>&) const;
  
  bool checkSectionWithLength(const std::list<AspFluentRef>& planCleaned, 
                              std::list<AspFluentRef>::const_iterator firstSuspect,
                               int length) const;
  
  bool hasLoops(const AnswerSet& plan) const;


  
private:
  const PlanSet* good;
  PlanSet* bad;
  const ActionSet& allActions;
  unsigned int shortestLength;
  bool planFiltered;
    
};
  
}

#endif
