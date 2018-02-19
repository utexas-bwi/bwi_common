#ifndef plan_exec_SarsaActionSelector_h__guard
#define plan_exec_SarsaActionSelector_h__guard

#include <actasp/ActionSelector.h>
#include <actasp/ExecutionObserver.h>
#include <actasp/state_utils.h>
#include <actasp/actaspfwd.h>
#include <actasp/AspFluent.h>

#include <actasp/GraphPolicy.h>
#include <actasp/FilteringKR.h>


namespace plan_exec {

template <typename T>
class RewardFunction;

class DefaultActionValue;


struct SarsaParams {
  
  SarsaParams() : alpha(0.8), gamma(0.9999), lambda(0.9), epsilon(0.15) {}
  
  double alpha;
  double gamma;
  double lambda;
  double epsilon;
};

class SarsaActionSelector : public actasp::ActionSelector, public actasp::ExecutionObserver {
public:
  
  typedef std::set< actasp::AspFluent> State;

  SarsaActionSelector(actasp::FilteringKR* reasoner, DefaultActionValue *defval, RewardFunction<State>*reward, const SarsaParams& p = SarsaParams() );

  actasp::ActionSet::const_iterator choose(const actasp::ActionSet &options) throw();

  void actionStarted(const actasp::AspFluent& action) throw();
  void actionTerminated(const actasp::AspFluent& action) throw();

  void policyChanged(actasp::PartialPolicy* newPolicy) throw();
  void goalChanged(std::vector<actasp::AspRule> newGoalRules) throw();
  
  //for value:
  void readFrom(std::istream & fromStream) throw();
  void writeTo(std::ostream & toStream) throw();

  //for notFilteredToFiltered:
  void readMapFrom(std::istream & fromStream) throw();
  void writeMapTo(std::ostream & toStream) throw();
  
  void episodeEnded() throw();

  void saveValueInitialState(const std::string& fileName);


  typedef std::map< actasp::AspFluent, double, actasp::ActionComparator> ActionValueMap;
  typedef std::map< State , ActionValueMap , actasp::StateComparator<actasp::AspFluent> > StateActionMap;

private:
  
  void updateValue(double v_s_prime);
  
  actasp::FilteringKR *reasoner;
  DefaultActionValue *defval;

  SarsaParams p;
  RewardFunction<State> *reward;

  StateActionMap value;
  StateActionMap e;
  State initial; 
  State initialNotFiltered;
  State final; 
  State finalNotFiltered;
  actasp::AspFluent previousAction;
  double v_s;

  //for filterstate:
  actasp::GraphPolicy *policy;
  std::vector<actasp::AspRule> goalRules;
  std::map< State, State, actasp::StateComparator<actasp::AspFluent> > notFilteredToFiltered;
  bool stateCompare(const std::set<actasp::AspFluent> state, const std::set<actasp::AspFluent> otherstate);

};

}


#endif
