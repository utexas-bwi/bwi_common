#ifndef actasp_ExecutionObserver_h__guard
#define actasp_ExecutionObserver_h__guard

#include <actasp/AspRule.h>

#include <vector>

namespace actasp {

class AspFluent;
class AnswerSet;
class PartialPolicy;

struct ExecutionObserver {

  virtual void actionStarted(const AspFluent& action) throw() =0 ;
  virtual void actionTerminated(const AspFluent& action) throw() =0;
  
  virtual void goalChanged(std::vector<actasp::AspRule> newGoalRules) throw() = 0;
  
  //TODO move this into a separate observer
  virtual void policyChanged(PartialPolicy* policy) throw() = 0;

  virtual ~ExecutionObserver() {}
};

}


#endif
