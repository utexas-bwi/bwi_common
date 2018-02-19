#ifndef plan_exec_QLearningActionSelector_h__guard
#define plan_exec_QLearningActionSelector_h__guard

#include "DefaultActionValue.h"

#include <actasp/state_utils.h>
#include <actasp/ExecutionObserver.h>
#include <actasp/ActionSelector.h>
#include <actasp/AspKR.h>

#include <map>
#include <set>

#include <iosfwd>

namespace plan_exec {

template <typename T>
class RewardFunction;

class QLearningActionSelector : public actasp::ActionSelector, public actasp::ExecutionObserver {
public:
  
   typedef std::set< actasp::AspFluent> State;
  
  QLearningActionSelector(double alpha, RewardFunction<State> *reward, actasp::AspKR *reasoner, DefaultActionValue *defval);
  
  actasp::ActionSet::const_iterator choose(const actasp::ActionSet &options) throw() ;
  
  void episodeEnded();
  
  void actionStarted(const actasp::AspFluent& action) throw();
  void actionTerminated(const actasp::AspFluent& action) throw();
  
  void readFrom(std::istream & fromStream) throw();
  void writeTo(std::ostream & toStream) throw();
  
 
  typedef std::map< actasp::AspFluent, double, actasp::ActionComparator> ActionValueMap;
  typedef std::map< State , ActionValueMap , actasp::StateComparator<actasp::AspFluent> > StateActionMap;
  
private:
  actasp::AspKR *reasoner;
  DefaultActionValue *defval;

  double alpha;
  RewardFunction<State> *reward;
  
  StateActionMap value;
  State initial;
  State final;
  actasp::AspFluent previousAction;
  int count;
};

}

#endif