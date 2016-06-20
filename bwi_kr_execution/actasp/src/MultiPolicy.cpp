#include <actasp/MultiPolicy.h>

#include <actasp/Action.h>
#include <actasp/action_utils.h>

#include <algorithm>

using namespace std;

namespace actasp {
  
MultiPolicy::MultiPolicy(const ActionSet& actions) : policy(), allActions(actions) {}
	
ActionSet MultiPolicy::actions(const std::set<AspFluent>& state) const throw() {
  
	std::map<set<AspFluent>, ActionSet >::const_iterator acts = policy.find(state);

	if(acts != policy.end()) {
		return acts->second;
	}
	
	return ActionSet();
}

void MultiPolicy::merge(const PartialPolicy* otherPolicy) {
    const MultiPolicy *other = dynamic_cast<const MultiPolicy*>(otherPolicy);
    if(other != NULL)
      merge(other);
    else 
      throw runtime_error("method not implemented for a partial policy other than MultiPolicy");
}

void MultiPolicy::merge(const AnswerSet& plan) throw(logic_error) {


  //ignore the last time step becuase it's the final state and has no actions
  unsigned int planLength = plan.maxTimeStep();

  set<AspFluent> state = plan.getFluentsAtTime(0);
  
	for (int timeStep = 1; timeStep <=planLength; ++timeStep) {
		
		set<AspFluent> stateWithAction = plan.getFluentsAtTime(timeStep);
    
    //find the action
    set<AspFluent>::iterator actionIt = find_if(stateWithAction.begin(),stateWithAction.end(),IsAnAction(allActions));
    
    if(actionIt == stateWithAction.end())
      throw logic_error("MultiPolicy: no action for some state");
    
    AspFluent action = *actionIt;
		
		//remove the action from there
		stateWithAction.erase(actionIt);
		
		ActionSet &stateActions = policy[state]; //creates an empty vector if not present

		stateActions.insert(action);
    
    //TODO check this
    state = stateWithAction;
    
	}

}

struct MergeActions {
  MergeActions( std::map<std::set<AspFluent>, ActionSet, StateComparator<AspFluent> > &policy) : policy(policy) {}
 
 void operator()(const std::pair<set<AspFluent>, ActionSet >& stateActions) {
  
   map<set<AspFluent>, ActionSet >::iterator found = policy.find(stateActions.first);
   if(found == policy.end())
     policy.insert(stateActions);
   
   else {
     found->second.insert(stateActions.second.begin(),stateActions.second.end());
   }
     
   
 }
 
  std::map<std::set<AspFluent>, ActionSet, StateComparator<AspFluent> > &policy;
};

void MultiPolicy::merge(const MultiPolicy* otherPolicy) {
  
  set_union(otherPolicy->allActions.begin(),otherPolicy->allActions.end(),
                 allActions.begin(),allActions.end(),
                 inserter(allActions,allActions.begin()));
  
  for_each(otherPolicy->policy.begin(),otherPolicy->policy.end(),MergeActions(policy));
}

bool MultiPolicy::empty() const throw() {
	return policy.empty();
}



}
