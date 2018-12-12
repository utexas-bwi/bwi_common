#include <actasp/GraphPolicy.h>

#include <actasp/action_utils.h>
#include "reasoners/LexComparator.h"

#include <algorithm>
#include <typeinfo>
#include <iostream>
#include <iterator>

using namespace std;

namespace actasp {

GraphPolicy::GraphPolicy(const ActionSet& actions) :  policy(), allActions(actions), plans(), planIndex() {}

ActionSet GraphPolicy::actions(const std::set<AspFluent>& state) const noexcept {

    std::map<set<AspFluent>, ActionSet >::const_iterator acts = policy.find(state);

    if(acts != policy.end()) {
        return acts->second;
    }

    return ActionSet();
}

void GraphPolicy::merge(const PartialPolicy* otherPolicy) {
    const GraphPolicy *other = dynamic_cast<const GraphPolicy*>(otherPolicy);
    if(other != nullptr)
        merge(other);
    else
        throw runtime_error("method not implemented for a partial policy other than GraphPolicy");
}

void GraphPolicy::merge(const AnswerSet& plan) {

    plans.push_back(list<AspFluent>());
    PlanList::iterator currentPlan = --plans.end();

    unsigned int planLength = plan.maxTimeStep();

    set<AspFluent> state = plan.getFluentsAtTime(0);

    for (int timeStep = 1; timeStep <=planLength; ++timeStep) {

        set<AspFluent> stateWithAction = plan.getFluentsAtTime(timeStep);

        //find the action
        set<AspFluent>::iterator actionIt = find_if(stateWithAction.begin(),stateWithAction.end(),IsAnAction(allActions));

        if(actionIt == stateWithAction.end())
            throw logic_error("GraphPolicy: no action for some state");

        AspFluent action = *actionIt;

        //remove the action from there
        stateWithAction.erase(actionIt);

        ActionSet &stateActions = policy[state]; //creates an empty vector if not present

        stateActions.insert(action);

        currentPlan->push_back(action);

        std::list<AspFluent>::const_iterator currentAction = --currentPlan->end();
        planIndex[state].push_back(make_pair(currentPlan,currentAction));

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

void GraphPolicy::merge(const GraphPolicy* otherPolicy) {

    set_union(otherPolicy->allActions.begin(),otherPolicy->allActions.end(),
              allActions.begin(),allActions.end(),
              inserter(allActions,allActions.begin()));

    for_each(otherPolicy->policy.begin(),otherPolicy->policy.end(),MergeActions(policy));

    PlanList::iterator firstPlan = plans.end();
    plans.insert(plans.end(),otherPolicy->plans.begin(), otherPolicy->plans.end());

    PlanIndex::const_iterator stateOtherPolicy = otherPolicy->planIndex.begin();


    for(; stateOtherPolicy != otherPolicy->planIndex.end(); ++stateOtherPolicy) {

        PlanReference newList;

        PlanReference::const_iterator oldReference = stateOtherPolicy->second.begin();

        for(; oldReference != stateOtherPolicy->second.end(); ++oldReference) {
            size_t planDist = distance(otherPolicy->plans.begin(),oldReference->first);

            PlanList::iterator newPlan = firstPlan;
            advance(newPlan,planDist);

            size_t actionDist = distance(oldReference->first->begin(),oldReference->second);
            list<AspFluent>::iterator newAction = newPlan->begin();
            advance(newAction,actionDist);

            newList.push_back(make_pair(newPlan,newAction));

        }

        PlanReference &stateProcessed = planIndex[stateOtherPolicy->first];
        stateProcessed.insert(stateProcessed.end(),newList.begin(), newList.end());

    }

}

bool GraphPolicy::empty() const noexcept {
    return policy.empty();
}

std::vector<actasp::AnswerSet> GraphPolicy::plansFrom(const std::set<AspFluent>& state) noexcept {

    vector<AnswerSet> result;

    PlanIndex::const_iterator stateIt = planIndex.find(state);
    if(stateIt == planIndex.end())
        return result;

    set< list<AspFluent>, LexComparator > plans;
    PlanReference::const_iterator planIt = stateIt->second.begin();

    for(; planIt != stateIt->second.end(); ++planIt)
        plans.insert( list<AspFluent>(planIt->second,planIt->first->end()) );
    
    set< list<AspFluent>, LexComparator >::const_iterator solutions = plans.begin();
    for(; solutions != plans.end(); ++solutions) {
        
        result.push_back(AnswerSet(solutions->begin(), solutions->end()));
      
//         copy(solutions->begin(), solutions->end(), ostream_iterator<string>(cout, " "));
//         cout << endl;
    }
//     cout << result.size();
//     cout << endl;

    return result;
}


}
