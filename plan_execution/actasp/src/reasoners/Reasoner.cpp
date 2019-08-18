#include <actasp/reasoners/Reasoner.h>

#include <actasp/Solver.h>
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

Reasoner::Reasoner(Solver *queryGenerator, unsigned int max_n, const set<string> &allActions) :
            clingo(queryGenerator), max_n(max_n), allActions(allActions) {}
  
ActionSet Reasoner::availableActions() const noexcept {
  vector<Plan> all_possible_one_step_plans = clingo->lengthRangePlanQuery({}, 1, 1, 0, nullptr);

  ActionSet computed;

  for (const auto &act: all_possible_one_step_plans) {
    computed.insert(act.actions.begin(), act.actions.end());
  }

  return computed;
}

AnswerSet Reasoner::currentStateQuery(const vector<AspRule> &query) const noexcept {
  return clingo->currentStateQuery(query);
}

bool Reasoner::isPlanValid(const Plan &plan, const vector<AspRule> &goal) const noexcept {

  return !(clingo->monitorQuery(goal, plan, nullptr).empty());
}

Plan Reasoner::computePlan(const vector<AspRule> &goal) const noexcept(false) {
  vector<Plan> plans = clingo->minimalPlanQuery(goal, max_n, 1, nullptr);
  
  if(plans.empty())
    return Plan();
  
  else return *(plans.begin()); //it's really at most one
}

struct AnswerSetToList {
  list <AspFluentRef> operator()(const AnswerSet& aset) const {

    return list <AspFluentRef>(aset.fluents.begin(), aset.fluents.end());

  }
};

struct PlanLongerThan {

  PlanLongerThan(unsigned int length) : length(length) {}

  bool operator()(const AnswerSet& plan) const {
    return plan.maxTimeStep() > length;
  }

  unsigned int length;
};


vector<Plan> Reasoner::computeAllPlans(const vector<AspRule> &goal, double suboptimality) const noexcept(false) {

  if (suboptimality < 1) {
    stringstream num;
    num << suboptimality;
    throw logic_error("Clingo: suboptimality value cannot be less then one, found: " + num.str());
  }


  vector<Plan> firstAnswerSets = clingo->minimalPlanQuery(goal, max_n, 0, nullptr);

  if (firstAnswerSets.empty())
    return {};

  unsigned int shortestLength = firstAnswerSets.begin()->maxTimeStep();

  int maxLength = ceil(suboptimality * shortestLength);

  if (maxLength == shortestLength)
    return vector<Plan>(firstAnswerSets.begin(), firstAnswerSets.end());

  set< list <AspFluentRef>, LexComparator > goodPlans;
  transform(firstAnswerSets.begin(), firstAnswerSets.end(),inserter(goodPlans,goodPlans.begin()),AnswerSetToList());

  vector<Plan> moreAnswerSets = clingo->lengthRangePlanQuery(goal, shortestLength + 1, maxLength, 0, nullptr);

  vector<Plan>::iterator currentFirst = moreAnswerSets.begin();

  set< list<AspFluentRef>, LexComparator > badPlans;
  //this object remembers the set of bad plans, cannot be created inside the loop

  // TODO: Fix isNotLocallyOptimal
  //IsNotLocallyOptimal isNotLocallyOptimal(&goodPlans, &badPlans,allActions,shortestLength,true);

  list<PlanRef> goodPointers(firstAnswerSets.begin(), firstAnswerSets.end());
  while (currentFirst != moreAnswerSets.end()) {

    //process the plans in groups of increasing length

    vector<Plan>::iterator currentLast = find_if(currentFirst, moreAnswerSets.end(),
                                                 PlanLongerThan(currentFirst->maxTimeStep()));

    size_t size_pre_copy = goodPointers.size();

    //remove_copy_if(currentFirst,currentLast,back_inserter(goodPointers),isNotLocallyOptimal);

    list<PlanRef>::iterator from = goodPointers.begin();
    advance(from,size_pre_copy);
    transform(from, goodPointers.end(),inserter(goodPlans,goodPlans.begin()),AnswerSetToList());

    currentFirst = currentLast;
  }

  vector<Plan> finalVector(goodPointers.begin(), goodPointers.end());
  
//   cout << "  ---  good plans ---" << endl;
//   vector< AnswerSet>::const_iterator printIt = finalVector.begin();
//   for (; printIt != finalVector.end(); ++printIt) {
//     copy(printIt->getFluents().begin(),printIt->getFluents().end(),ostream_iterator<string>(cout, " "));
//     cout << endl;
//   }
//   cout << " ---- " << endl;

  return finalVector;

}

AnswerSet Reasoner::computeOptimalPlan(const vector<AspRule> &goal, bool filterActions, double suboptimality,
                                       bool minimum) const noexcept(false) {
  if (suboptimality < 1) {
    stringstream num;
    num << suboptimality;
    throw logic_error("Clingo: suboptimality value cannot be less then one, found: " + num.str());
  }


  vector<Plan> firstAnswerSets = clingo->minimalPlanQuery(goal, max_n, 0, nullptr);

  if (firstAnswerSets.empty())
    return AnswerSet();

  unsigned int shortestLength = firstAnswerSets.begin()->maxTimeStep();

  int maxLength = ceil(suboptimality * shortestLength);

  AnswerSet optimalAnswerSet = clingo->optimalPlanQuery(goal, maxLength, 0, nullptr);

  return optimalAnswerSet;
}


struct PolicyMerger {

  PolicyMerger(PartialPolicy * policy) :
    policy(policy) {}

  void operator()(const Plan &set) {
    policy->merge(set);
  }

  PartialPolicy *policy;
};



PartialPolicy *Reasoner::computePolicy(const vector<AspRule> &goal, double suboptimality) const noexcept(false) {

  MultiPolicy *p = new MultiPolicy(allActions);
  computePolicyHelper(goal,suboptimality,p);
  return p;

}

void Reasoner::computePolicyHelper(const vector<AspRule> &goal, double suboptimality,
                                   PartialPolicy *policy) const noexcept(false) {
    if (suboptimality < 1) {
    stringstream num;
    num << suboptimality;
    throw logic_error("Clingo: suboptimality value cannot be less than one, found: " + num.str());
  }

//   clock_t kr1_begin = clock();
  vector<Plan> firstAnswerSets = clingo->minimalPlanQuery(goal, max_n, 0, nullptr);
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

//   clock_t kr2_begin = clock();
  vector<Plan> answerSets = clingo->lengthRangePlanQuery(goal, shortestLength + 1, maxLength, 0, nullptr);
//   clock_t kr2_end = clock();
//   cout << "The second kr call took " << (double(kr2_end - kr2_begin) / CLOCKS_PER_SEC) << " seconds" << endl;

  set< list <AspFluentRef>, LexComparator > badPlans;
  //this object remembers the set of bad plans, cannot be created inside the loop
  // TODO: Fix is not locally optimal
  //IsNotLocallyOptimal isNotLocallyOptimal(&goodPlans,&badPlans, allActions, shortestLength,false);

  auto currentFirst = answerSets.begin();
//   clock_t filter_begin = clock();
  while (currentFirst != answerSets.end()) {

    //process the plans in groups of increasing length

    auto currentLast = find_if(currentFirst,answerSets.end(),PlanLongerThan(currentFirst->maxTimeStep()));

    list<PlanRef> goodPointers;
    //remove_copy_if(currentFirst,currentLast,back_inserter(goodPointers),isNotLocallyOptimal);

    for_each(goodPointers.begin(),goodPointers.end(),PolicyMerger(policy));


    currentFirst = currentLast;

  }
//   clock_t filter_end = clock();


   for (const auto &goodPlan: goodPlans) {
     //copy(goodPlan.begin(),goodPlan.end(),ostream_iterator<string>(cout, " "));
     //cout << endl;
   }
//   
//   cout << "filtering took " << (double(filter_end - filter_begin) / CLOCKS_PER_SEC) << " seconds" << endl;

}

vector<AnswerSet> Reasoner::query(const vector<AspRule> &query, unsigned int timestep) const noexcept {
  // TODO: Fix this class...
  assert(false);
  //return clingo->genericQuery(query,timestep,"query_output",0);
}

}
