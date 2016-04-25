#include "IsNotLocallyOptimal.h"

#include <actasp/AnswerSet.h>
#include <actasp/action_utils.h>
#include <actasp/state_utils.h>

#include <algorithm>

#include <iostream>
#include <iterator>

using namespace std;

namespace actasp {
  
IsNotLocallyOptimal::IsNotLocallyOptimal(const PlanSet* good, PlanSet* bad, 
                                         const ActionSet& allActions, 
                                         unsigned int shortestLength,
                                         bool planFiltered
                                        ) :
                                         good(good), bad(bad), allActions(allActions), 
                                         shortestLength(shortestLength),
                                         planFiltered(planFiltered){}

  
bool IsNotLocallyOptimal::operator()(const AnswerSet& plan) {
  
//   cout << "*** test IsNotLocallyOptimal" << endl;
  
//       PlanSet::const_iterator badIt = bad->begin();
//     cout << " --- bad plans --- " << bad->size() << endl;
//     for(; badIt != bad->end(); ++badIt) {
//       copy(badIt->begin(), badIt->end(), ostream_iterator<string>(cout, " "));
//       cout << endl;
//     }
//     cout << " --- " << endl;
  
  if(!planFiltered) {
    bool loops = hasLoops(plan);
    if(loops)
      return true;
  }
  

  const list<AspFluentRef> planCleaned = cleanPlan(plan); //remove fluents that are not actions

//   cout << "piano pulito: ";
//   copy(planCleaned.begin(), planCleaned.end(), ostream_iterator<string>(cout, " "));
//   cout << endl;
  
  //find the first action that does not belong to any minimal plan
  list<AspFluentRef>::const_iterator firstSuspect = findFirstSuspiciousAction(planCleaned); 

  
  if(firstSuspect == planCleaned.end()) {
//     cout << "good" << endl;
    return false;
  }
  
//   cout << "first suspect: " << firstSuspect->toString() << endl;
  
  for(int l = 1, size = planCleaned.size(); l <= size - shortestLength; ++l) {

    if(checkSectionWithLength(planCleaned,firstSuspect,l)) {
      bad->insert(planCleaned);
      return true;
    }

  }
  
  //last check, if the action before the suspect is useless
  
  list<AspFluentRef>::const_iterator beforeSuspect = firstSuspect;
  advance(beforeSuspect, -1);
  list<AspFluentRef> withoutBeforeSuspect(planCleaned.begin(),beforeSuspect);
  withoutBeforeSuspect.insert(withoutBeforeSuspect.end(),firstSuspect,planCleaned.end());
  bool lastCheck = checkPlanValidity(withoutBeforeSuspect);
  if(lastCheck) {
    bad->insert(planCleaned);
    //cout <<  "bad!" << endl;
  }
//  else
//    cout << "good after check" << endl;
  
  return lastCheck;
  
  
//   bool valid1 = validFrom(planCleaned,firstSuspect);
//   
//   bool valid2 = false;
//   
//   if(!valid1 && firstSuspect != planCleaned.begin())
//     valid2 = validFrom(planCleaned, -- firstSuspect);
//   
// //     badIt = bad->begin();
// //     cout << " --- bad plans --- " << bad->size() << endl;
// //     for(; badIt != bad->end(); ++badIt) {
// //       copy(badIt->begin(), badIt->end(), ostream_iterator<string>(cout, " "));
// //       cout << endl;
// //     }
// //     cout << " --- " << endl;
//     
//   return valid1 || valid2;
 
}

bool IsNotLocallyOptimal::hasLoops(const AnswerSet& plan) const {
    set<AspFluentRef> state;
    
    set< set<AspFluentRef>, StateComparator<AspFluentRef> > allStates;
    
    IsAnAction isAnAction(allActions);
    int timeStep = 0;
    
    AnswerSet::FluentSet::const_iterator p = plan.getFluents().begin();
    for(; p!= plan.getFluents().end(); ++p) {
      
      if(p->getTimeStep() != timeStep) {

        bool present = !(allStates.insert(state).second);

        if(present)
          return true;
        
        state.clear();
        ++timeStep;
      }

      if(!isAnAction(*p))
        state.insert(*p);

    }
    
    //last state
    bool present = !(allStates.insert(state).second);

    if(present)
        return true;
        
    return false;
}


bool IsNotLocallyOptimal::checkSectionWithLength(const std::list<AspFluentRef>& planCleaned, 
                              const std::list<AspFluentRef>::const_iterator firstSuspect,
                               int length) const {
  

  int pos = distance(planCleaned.begin(), firstSuspect); 
  int diffPos = std::max(-(length -1), -pos);
  unsigned int initialPos = pos + diffPos;
  
//    cout << "pos " << pos << " diffpos " << diffPos << " initialPos " << initialPos << endl;

  
  std::list<AspFluentRef>::const_iterator initial = firstSuspect;
  advance(initial, diffPos);
  
  for(int size = planCleaned.size(); initialPos <= pos && initialPos + length <= size; ++ initialPos, ++initial) {
    
//       cout << "initialPos: " << initialPos << endl;
    
      list<AspFluentRef> testPlan(planCleaned.begin(), initial);
      
      std::list<AspFluentRef>::const_iterator final = initial;
      advance(final,length);
      
      testPlan.insert(testPlan.end(),final,planCleaned.end());
      
//         cout << "test plan: ";
//         copy(testPlan.begin(), testPlan.end(), ostream_iterator<string>(cout, " "));
//         cout << endl;
    
      if(checkPlanValidity(testPlan)) {
//         cout << "bad" << endl;
        return true;
      }
  }
  
//   cout << "done with length " << length << endl;
  
  return false;
}


bool IsNotLocallyOptimal::validFrom(const list<AspFluentRef>& planCleaned, list<AspFluentRef>::const_iterator firstSuspect) const{

 //create a test plan without the suspicious action
  list<AspFluentRef> testPlan(planCleaned.begin(), firstSuspect);
  list<AspFluentRef>::iterator suspectInTest = testPlan.end();
  
  testPlan.insert(testPlan.end(),(++firstSuspect),planCleaned.end());
  advance(suspectInTest, -distance(firstSuspect,planCleaned.end()));
  
  bool done = false;
  while(!done) {
  
//   cout << "test plan: ";
//   copy(testPlan.begin(), testPlan.end(), ostream_iterator<string>(cout, " "));
//   cout << endl;
    
  bool isValid = checkPlanValidity(testPlan);
  if(isValid) {
    bad->insert(planCleaned);

//     cout << "bad" << endl;
    
    return true;
    
  }
  //keep removing actions starting from the first suspicious one
  if(suspectInTest == testPlan.end())
    done = true;
  else 
    suspectInTest = testPlan.erase(suspectInTest);
  
  }
  
//   cout << "good" << endl;
  return false;
}

list<AspFluentRef> IsNotLocallyOptimal::cleanPlan(const AnswerSet& plan) const{
  
  
  list<AspFluentRef> actionsOnly;
  
  if(planFiltered) {
    actionsOnly.insert(actionsOnly.end(),plan.getFluents().begin(),plan.getFluents().end());
  }
  else {
  remove_copy_if(plan.getFluents().begin(), plan.getFluents().end(),
                   back_inserter(actionsOnly),not1(IsAnAction(allActions)));
  }
  
  return actionsOnly;

}

list<AspFluentRef>::const_iterator IsNotLocallyOptimal::findFirstSuspiciousAction(const list<AspFluentRef>& plan) const {
  
  if(good->empty())
    return plan.end(); //shouldn't really happen...
    

  //either the lower bound or the element before it are the most similar plans to mine.
  PlanSet::iterator lb =  good->lower_bound(plan);
  
//   if(lb != good->end()) {
//     cout << "lower bound: ";
//     copy(lb->begin(), lb->end(), ostream_iterator<string>(cout, " "));
//     cout <<endl;
//   }
  

  int dist1 = -1, dist2 = -1;
  list<AspFluentRef>::const_iterator sus1=plan.end(), sus2= plan.end();
  
  if(lb != good->end()) {
    pair< list<AspFluentRef>::const_iterator, list<AspFluentRef>::const_iterator > different = 
              mismatch(lb->begin(), lb->end(), plan.begin(), ActionEquality());
    dist1 = distance(plan.begin(), different.second);
    sus1 = different.second;
  }
  
 
  if (lb != good->begin()) {
    --lb;

//     cout << "-- lower bound: ";
//     copy(lb->begin(), lb->end(), ostream_iterator<string>(cout, " "));
//     cout <<endl;
    
    pair< list<AspFluentRef>::const_iterator, list<AspFluentRef>::const_iterator > different =
      mismatch(lb->begin(), lb->end(), plan.begin(), ActionEquality());
    dist2 = distance(plan.begin(), different.second);
    sus2 = different.second;
  }
  
//    cout << "dist " << dist1 << " " << dist2 << endl;
  if(dist1 > dist2){
//     cout << "dist1" << endl;
    return sus1;
  }
//   else 
//     cout << "dist2" << endl;
  
  return sus2;
  
}
  
bool IsNotLocallyOptimal::checkPlanValidity(const list<AspFluentRef>& plan) const {
  PlanSet::const_iterator found = good->find(plan);
  if(found != good->end())
    return true;
  
  found = bad->find(plan);
  
  return found != bad->end();
}


}

