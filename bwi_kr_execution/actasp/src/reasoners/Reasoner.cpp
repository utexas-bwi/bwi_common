
#include <actasp/reasoners/Reasoner.h>

#include <actasp/QueryGenerator.h>
#include <actasp/action_utils.h>

#include "LexComparator.h"
#include "IsNotLocallyOptimal.h"

#include <vector>
#include <functional>
#include <sstream>
#include <cmath>
#include <iostream>
#include <iterator>


using namespace std;

namespace actasp {
  
Reasoner::Reasoner(QueryGenerator *actualReasoner,unsigned int max_n,const ActionSet& allActions) :
            clingo(actualReasoner), max_n(max_n), allActions(allActions) {}
  
ActionSet Reasoner::availableActions() const throw() {
  list<AnswerSet> actions = clingo->lengthRangePlanQuery(vector<AspRule>(),true,1,1,0);

  ActionSet computed;

  list<AnswerSet>::iterator act = actions.begin();
  for (; act != actions.end(); ++act) {
    computed.insert(act->getFluents().begin(), act->getFluents().end());
  }

  return computed;
}

AnswerSet Reasoner::currentStateQuery(const std::vector<actasp::AspRule>& query) const throw() {
  return clingo->currentStateQuery(query);
}

struct fluent2Rule {
  
  fluent2Rule() : timeStep(0), useTimeStep(false) {}
  fluent2Rule(unsigned int timeStep) : timeStep(timeStep), useTimeStep(true) {}
    
  AspRule operator() (const AspFluent& fluent){
      AspRule rule;
      rule.head.push_back(fluent);
      if(useTimeStep)
        rule.head[0].setTimeStep(timeStep);
      return rule;
  }
  
  unsigned int timeStep;
  bool useTimeStep;
};

bool Reasoner::updateFluents(const std::vector<actasp::AspFluent> &observations) throw() {

  //copy the observations in the heads of rules
  vector<AspRule> obsRules;
  obsRules.reserve(observations.size()+1); //we'll add noop later
  transform(observations.begin(),observations.end(),back_inserter(obsRules),fluent2Rule(1));
  //add the rule for the noop action
  
  AspRule noopRule;
  noopRule.head.push_back(AspFluent("noop",vector<string>(),1));
  
  obsRules.push_back(noopRule);

  list<AnswerSet> currentState = clingo->genericQuery(obsRules,1,"observationQuery",1);

  if (currentState.empty())
    return false; //the observations are incompatible with the current state and are discarded
   
  //the last answer set is the one with the timestep at 1. The ones before may have 0.
  set<AspFluent> newStateFluents = currentState.rbegin()->getFluentsAtTime(1);
  
  newStateFluents.erase(AspFluent("noop",vector<string>(),1));
  
  clingo->setCurrentState(newStateFluents);

  return true;
}

 bool Reasoner::isPlanValid(const AnswerSet& plan, const std::vector<actasp::AspRule>& goal)  const throw() {

  return !(clingo->monitorQuery(goal,plan).empty()); 
}

void Reasoner::resetCurrentState() throw() {
clingo->setCurrentState(set<AspFluent>());
}

void Reasoner::setCurrentState(const std::set<actasp::AspFluent>& newState) throw() {
  clingo->setCurrentState(newState);
}

AnswerSet Reasoner::computePlan(const std::vector<actasp::AspRule>& goal) const throw (std::logic_error){
  list<AnswerSet> plans = clingo->minimalPlanQuery(goal,true,max_n,1);
  
  if(plans.empty())
    return AnswerSet();
  
  else return *(plans.begin()); //it's really at most one
}

struct AnswerSetToList {
  list <AspFluentRef> operator()(const AnswerSet& aset) const {

    return list <AspFluentRef>(aset.getFluents().begin(), aset.getFluents().end());

  }
};

struct PlanLongerThan {

  PlanLongerThan(unsigned int length) : length(length) {}

  bool operator()(const AnswerSet& plan) const {
    return plan.maxTimeStep() > length;
  }

  unsigned int length;
};

struct AnswerSetRef {
  AnswerSetRef(const AnswerSet& aset) : aset(&aset) {}

  operator const AnswerSet&() const {
    return *aset;
  }

  const AnswerSet *aset;
};

std::vector< AnswerSet > Reasoner::computeAllPlans(const std::vector<actasp::AspRule>& goal, double suboptimality) const throw (std::logic_error) {

  if (suboptimality < 1) {
    stringstream num;
    num << suboptimality;
    throw logic_error("Clingo: suboptimality value cannot be less then one, found: " + num.str());
  }

 
  list<AnswerSet> firstAnswerSets = clingo->minimalPlanQuery(goal,true,max_n,0);

  if (firstAnswerSets.empty())
    return vector<AnswerSet>();

  unsigned int shortestLength = firstAnswerSets.begin()->maxTimeStep();

  int maxLength = ceil(suboptimality * shortestLength);

  if (maxLength == shortestLength)
    return vector<AnswerSet>(firstAnswerSets.begin(), firstAnswerSets.end());

  set< list <AspFluentRef>, LexComparator > goodPlans;
  transform(firstAnswerSets.begin(), firstAnswerSets.end(),inserter(goodPlans,goodPlans.begin()),AnswerSetToList());

  list<AnswerSet> moreAnswerSets = clingo->lengthRangePlanQuery(goal,true,shortestLength+1,maxLength,0);

  list<AnswerSet>::iterator currentFirst = moreAnswerSets.begin();

  set< list<AspFluentRef>, LexComparator > badPlans;
  //this object remembers the set of bad plans, cannot be created inside the loop

  IsNotLocallyOptimal isNotLocallyOptimal(&goodPlans, &badPlans,allActions,shortestLength,true);

  list<AnswerSetRef> goodPointers(firstAnswerSets.begin(),firstAnswerSets.end());
  while (currentFirst != moreAnswerSets.end()) {

    //process the plans in groups of increasing length

    list<AnswerSet>::iterator currentLast = find_if(currentFirst,moreAnswerSets.end(),PlanLongerThan(currentFirst->maxTimeStep()));

    size_t size_pre_copy = goodPointers.size();

    remove_copy_if(currentFirst,currentLast,back_inserter(goodPointers),isNotLocallyOptimal);

    list<AnswerSetRef>::iterator from = goodPointers.begin();
    advance(from,size_pre_copy);
    transform(from, goodPointers.end(),inserter(goodPlans,goodPlans.begin()),AnswerSetToList());

    currentFirst = currentLast;
  }

  vector<AnswerSet> finalVector(goodPointers.begin(),goodPointers.end());
  
//   cout << "  ---  good plans ---" << endl;
//   vector< AnswerSet>::const_iterator printIt = finalVector.begin();
//   for (; printIt != finalVector.end(); ++printIt) {
//     copy(printIt->getFluents().begin(),printIt->getFluents().end(),ostream_iterator<string>(cout, " "));
//     cout << endl;
//   }
//   cout << " ---- " << endl;

  return finalVector;

}

AnswerSet Reasoner::computeOptimalPlan(const std::vector<actasp::AspRule>& goal, bool filterActions, double suboptimality, bool minimum) const throw (std::logic_error) {
  if (suboptimality < 1) {
    stringstream num;
    num << suboptimality;
    throw logic_error("Clingo: suboptimality value cannot be less then one, found: " + num.str());
  }

 
  list<AnswerSet> firstAnswerSets = clingo->minimalPlanQuery(goal,true,max_n,0);

  if (firstAnswerSets.empty())
    return AnswerSet();

  unsigned int shortestLength = firstAnswerSets.begin()->maxTimeStep();

  int maxLength = ceil(suboptimality * shortestLength);

  AnswerSet optimalAnswerSet = clingo->optimalPlanQuery(goal,filterActions,maxLength,0,minimum);

  return optimalAnswerSet;
}


struct PolicyMerger {

  PolicyMerger(PartialPolicy * policy) :
    policy(policy) {}

  void operator()(const AnswerSet& set) {
    policy->merge(set);
  }

  PartialPolicy *policy;
};



struct CleanPlan {

  CleanPlan(const ActionSet &allActions) : allActions(allActions) {}

  list<AspFluentRef> operator()(const AnswerSet &planWithStates) const {
    list<AspFluentRef> actionsOnly;

    remove_copy_if(planWithStates.getFluents().begin(), planWithStates.getFluents().end(),
                   back_inserter(actionsOnly),not1(IsAnAction(allActions)));

    return actionsOnly;

  }

  const ActionSet &allActions;

};

PartialPolicy *Reasoner::computePolicy(const std::vector<actasp::AspRule>& goal, double suboptimality) const throw (std::logic_error) {

  MultiPolicy *p = new MultiPolicy(allActions);
  computePolicyHelper(goal,suboptimality,p);
  return p;

}

void Reasoner::computePolicyHelper(const std::vector<actasp::AspRule>& goal, double suboptimality, PartialPolicy* policy) const throw (std::logic_error) {
    if (suboptimality < 1) {
    stringstream num;
    num << suboptimality;
    throw logic_error("Clingo: suboptimality value cannot be less than one, found: " + num.str());
  }

//   clock_t kr1_begin = clock();
  list<AnswerSet> firstAnswerSets = clingo->minimalPlanQuery(goal,false,max_n,0);
//   clock_t kr1_end = clock();
//   cout << "The first kr call took " << (double(kr1_end - kr1_begin) / CLOCKS_PER_SEC) << " seconds" << endl;

  if (firstAnswerSets.empty())
    return;

  unsigned int shortestLength = firstAnswerSets.begin()->maxTimeStep();

  for_each(firstAnswerSets.begin(),firstAnswerSets.end(),PolicyMerger(policy));

  int maxLength = ceil(suboptimality * shortestLength);

  //cout << "min lenght is " << shortestLength << " ; max lenght is " << maxLength << endl;

  if (maxLength == shortestLength)
    return;

  //all accepted plans sorted lexicographically
  set< list <AspFluentRef>, LexComparator > goodPlans;

  //remove the states from the plans
  transform(firstAnswerSets.begin(),firstAnswerSets.end(),inserter(goodPlans,goodPlans.begin()), CleanPlan(allActions));



//   clock_t kr2_begin = clock();
  list<AnswerSet> answerSets = clingo->lengthRangePlanQuery(goal,false,shortestLength+1,maxLength,0);
//   clock_t kr2_end = clock();
//   cout << "The second kr call took " << (double(kr2_end - kr2_begin) / CLOCKS_PER_SEC) << " seconds" << endl;

  set< list <AspFluentRef>, LexComparator > badPlans;
  //this object remembers the set of bad plans, cannot be created inside the loop
  IsNotLocallyOptimal isNotLocallyOptimal(&goodPlans,&badPlans, allActions, shortestLength,false);

  list<AnswerSet>::iterator currentFirst = answerSets.begin();
//   clock_t filter_begin = clock();
  while (currentFirst != answerSets.end()) {

    //process the plans in groups of increasing length

    list<AnswerSet>::iterator currentLast = find_if(currentFirst,answerSets.end(),PlanLongerThan(currentFirst->maxTimeStep()));

    list<AnswerSetRef> goodPointers;
    remove_copy_if(currentFirst,currentLast,back_inserter(goodPointers),isNotLocallyOptimal);

    for_each(goodPointers.begin(),goodPointers.end(),PolicyMerger(policy));

    transform(goodPointers.begin(),goodPointers.end(),inserter(goodPlans, goodPlans.begin()), CleanPlan(allActions));

    currentFirst = currentLast;

  }
//   clock_t filter_end = clock();


   set< list <AspFluentRef>, LexComparator >::const_iterator printIt = goodPlans.begin();
   for (; printIt != goodPlans.end(); ++printIt) {
     copy(printIt->begin(),printIt->end(),ostream_iterator<string>(cout, " "));
     cout << endl;
   }
//   
//   cout << "filtering took " << (double(filter_end - filter_begin) / CLOCKS_PER_SEC) << " seconds" << endl;


  return;
  
}

std::list< std::list<AspAtom> > Reasoner::query(const std::string &queryString, unsigned int timestep) const throw() {
  
  return clingo->genericQuery(queryString,timestep,"query_output",0);
}

}
