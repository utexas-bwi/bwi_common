

#include "QLearningActionSelector.h"

#include "RewardFunction.h"

#include <cstdlib>
#include <iterator>
#include <algorithm>

#include <sstream>
#include <iostream>
#include <fstream>

#define ROSOUTPUT 0
#if ROSOUTPUT
#include <ros/console.h>
#else
#include <iostream>
// #define ROS_DEBUG( X ) std::cout << "* "<< X << std::endl;
// #define ROS_DEBUG_STREAM( X ) std::cout << "* " << X << std::endl;
// #define ROS_INFO_STREAM( X ) std::cout << "** " << X<< std::endl ;

#define ROS_DEBUG( X )
#define ROS_DEBUG_STREAM( X ) 
#define ROS_INFO_STREAM( X ) 
#endif
#define EPSILON 0.1

using namespace actasp;
using namespace std;

namespace plan_exec {

struct CompareValues {

  CompareValues(QLearningActionSelector::ActionValueMap& value) : value(value) {}

  bool operator()(const AspFluent& first, const AspFluent& second) {
    return value[first] < value[second];
  }

  QLearningActionSelector::ActionValueMap& value;
};

QLearningActionSelector::QLearningActionSelector(double alpha, RewardFunction<State> *reward, 
                                                 actasp::AspKR *reasoner, DefaultActionValue *defval) :
  reasoner(reasoner),
  defval(defval),
  alpha(alpha),
  reward(reward),
  value(),
  initial(),
  final(),
  previousAction("noaction(0)"),
  count(0)  {}


struct CompareSecond {
  bool operator()(const pair<AspFluent, double>& first, const pair<AspFluent, double>& second) {
    return first.second < second.second;
  }
};

actasp::ActionSet::const_iterator QLearningActionSelector::choose(const actasp::ActionSet& options) throw() {

  if (!(initial.empty() || final.empty())) {

    ActionValueMap::const_iterator bestValuePair = max_element(value[final].begin(), value[final].end(),CompareSecond());
    
    double bestValue = 0;
    if(bestValuePair != value[final].end())
      bestValue = bestValuePair->second;

    double rew = reward->r(initial,previousAction,final);


    ROS_INFO_STREAM("old value: " << value[initial][previousAction]);
    ROS_INFO_STREAM("reward: " << rew);

    value[initial][previousAction] = (1 - alpha) * value[initial][previousAction] + alpha * (rew + bestValue);

    ROS_INFO_STREAM("new value: " << value[initial][previousAction]);

    initial.clear();
    final.clear();

  }


  stringstream ss;
  ss << "Evaluating options: ";
  copy(options.begin(), options.end(), ostream_iterator<string>(ss, " "));
  ss << endl;

  AnswerSet currentState = reasoner->currentStateQuery(vector<AspRule>());
  State state(currentState.getFluents().begin(), currentState.getFluents().end());

  ActionSet::const_iterator optIt = options.begin();
  for (; optIt != options.end(); ++optIt) {
    ActionValueMap &thisState = value[state];
    
    if(thisState.find(*optIt) == thisState.end()) {
      //initialize to default
      thisState[*optIt] = defval->value(*optIt);
    }
    ss << value[state][*optIt] << " ";
  }

  ROS_INFO_STREAM(ss.str());

  double prob = EPSILON;

  if (rand() <= prob * RAND_MAX) { //random
    ActionSet::const_iterator chosen =  options.begin();
    advance(chosen, rand() % options.size());
//     std::cout << "choosing random " << std::endl;
    return chosen;
  }

  actasp::ActionSet::const_iterator best = max_element(options.begin(), options.end(),CompareValues(value[state]));

  return best;

}

void QLearningActionSelector::actionStarted(const AspFluent&) throw() {
  initial.clear();
 
  AnswerSet currentState = reasoner->currentStateQuery(vector<AspRule>());
  initial.insert(currentState.getFluents().begin(), currentState.getFluents().end());
}


void QLearningActionSelector::actionTerminated(const AspFluent& action) throw() {
  AnswerSet currentState = reasoner->currentStateQuery(vector<AspRule>());
  final.clear();
  final.insert(currentState.getFluents().begin(), currentState.getFluents().end());
  previousAction = action;
}

void QLearningActionSelector::episodeEnded() {
  if(initial.empty())
    return;
    
  ROS_INFO_STREAM("old value: " << value[initial][previousAction]);
  value[initial][previousAction] = (1 - alpha) * value[initial][previousAction] + alpha * reward->r(initial,previousAction,final);
  ROS_INFO_STREAM("new value: " << value[initial][previousAction]);

  initial.clear();
  final.clear();
  ++count;
}


void QLearningActionSelector::readFrom(std::istream & fromStream) throw() {

  ROS_DEBUG("Loading value function");

  const string whiteSpaces(" \t");

  value.clear();

  while (fromStream.good() &&  !fromStream.eof()) {

    string stateLine;
    getline(fromStream,stateLine);

    size_t firstChar = min(stateLine.find_first_of(whiteSpaces),static_cast<size_t>(0));
    size_t lastChar = min(stateLine.find_last_not_of(whiteSpaces),stateLine.size());
    stateLine = stateLine.substr(firstChar,lastChar-firstChar+1);

    stringstream stateStream(stateLine);



    if (stateLine.empty())
      return;

    State state;
    copy(istream_iterator<string>(stateStream), istream_iterator<string>(), inserter(state, state.begin()));


    string actionLine;
    getline(fromStream,actionLine);

    while (actionLine.find("-----") == string::npos) {

      size_t firstChar = min(actionLine.find_first_of(whiteSpaces), static_cast<size_t>(0));
      size_t lastChar = min(actionLine.find_last_not_of(whiteSpaces),actionLine.size());
      actionLine = actionLine.substr(firstChar,lastChar-firstChar+1);

      stringstream actionStream(actionLine);

      double actionValue;
      string fluentString;

      actionStream >> actionValue >> fluentString;

      AspFluent action(fluentString);

      value[state].insert(make_pair(action,actionValue));

      getline(fromStream,actionLine);
    }

  }
}


void QLearningActionSelector::writeTo(std::ostream & toStream) throw() {

  ROS_DEBUG("Storing value function");

  StateActionMap::const_iterator stateIt = value.begin();
  //for each state
      ofstream stat("stats.txt", ios::app);
    AspFluent initialState("pos(2,0,0)");
  
  for (; stateIt != value.end(); ++stateIt) {
    

    if(stateIt->first.find(initialState) != stateIt->first.end()) {
        ActionValueMap::const_iterator actionIt= stateIt->second.begin();
        for (; actionIt != stateIt->second.end(); ++actionIt) {
           if(stateIt->first.find(initialState) != stateIt->first.end())
            stat << value[stateIt->first][actionIt->first] << " ";
        }
        
     }
     


    //write the state in a line
    copy(stateIt->first.begin(), stateIt->first.end(), ostream_iterator<string>(toStream, " "));

    //for each action
    ActionValueMap::const_iterator actionIt = stateIt->second.begin();
    for (; actionIt != stateIt->second.end(); ++actionIt) {

      toStream << endl;

      //write the value, then the action

      toStream <<  actionIt->second << " " << actionIt->first.toString();

    }

    //a separator for the next state
    toStream << endl << "-----" << endl;
  }
       stat << endl;
     stat.close();

}
}
