

/* This algorithm is an implementation of
 *
 * Seijen, Harm V., and Rich Sutton. "True Online TD (lambda)."
 * Proceedings of the 31st International Conference on Machine Learning (ICML-14). 2014.
 *
 */


#include "SarsaActionSelector.h"
#include "DefaultActionValue.h"
#include "RewardFunction.h"

#include <actasp/AspKR.h>

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


#include <algorithm>
#include <iterator>
#include <sstream>
#include <fstream>
#include <iostream>
#include <ctime>

using namespace std;
using namespace actasp;

namespace bwi_krexec {

SarsaActionSelector::SarsaActionSelector(actasp::AspKR* reasoner, DefaultActionValue *defval,
    RewardFunction<State>*reward, const SarsaParams& p) :
  reasoner(reasoner), defval(defval), p(p), reward(reward),value(),e(),
  initial(), final(), previousAction("nopreviousaction(0)"), v_s(0) {}

struct CompareValues {

  CompareValues(SarsaActionSelector::ActionValueMap& value) : value(value) {}

  bool operator()(const AspFluent& first, const AspFluent& second) {
    return value[first] < value[second];
  }

  SarsaActionSelector::ActionValueMap& value;
};

actasp::ActionSet::const_iterator SarsaActionSelector::choose(const actasp::ActionSet& options) throw() {


  stringstream ss;
  ss << "Evaluating options: ";
  copy(options.begin(), options.end(), ostream_iterator<string>(ss, " "));
  ss << endl;
  
  AnswerSet currentState = reasoner->currentStateQuery(vector<AspRule>());
  set<AspFluent> stateFluents(currentState.getFluents().begin(), currentState.getFluents().end());

  ActionSet::const_iterator optIt = options.begin();
  for (; optIt != options.end(); ++optIt) {
    
    ActionValueMap &thisState = value[stateFluents];

    if (thisState.find(*optIt) == thisState.end()) {
      //initialize to default
      thisState[*optIt] = defval->value(*optIt);
    }
    ss << value[stateFluents][*optIt] << " ";
  }
  
  ROS_DEBUG(ss.str());

  ROS_INFO_STREAM(ss.str());

  double prob = p.epsilon;

  ActionSet::const_iterator chosen;

  if (rand() <= prob * RAND_MAX) { //random
    chosen =  options.begin();
    advance(chosen, rand() % options.size());
//     std::cout << "choosing random " << std::endl;
  } else
    chosen = max_element(options.begin(), options.end(),CompareValues(value[stateFluents]));


  if (!(initial.empty() || final.empty())) {

    //we have a full state, action ,reward, state, action sequence!

    double v_s_prime = value[final][*chosen];
    updateValue(v_s_prime);

    initial.clear();
    final.clear();

  }

  return chosen;

}

void printE(SarsaActionSelector::StateActionMap &e) {
    cout << "--- E table ---" << endl;
  SarsaActionSelector::StateActionMap::iterator state = e.begin();
  for (; state != e.end(); ++state) {
    SarsaActionSelector::ActionValueMap::iterator action = state->second.begin();
    for (; action != state->second.end(); ++action) {
      copy(state->first.begin(), state->first.end(), ostream_iterator<string>(cout , " "));
      cout << endl << action->second << " " << action->first.toString() << endl;
    }
  }
  cout << "---" << endl;
}

void SarsaActionSelector::updateValue(double v_s_prime) {
  
  
//   printE(e);
  
  double rew = reward->r(initial,previousAction,final);

  double delta = rew + p.gamma * v_s_prime - v_s;

  //set the elegibility trace of the current state-action pair
  double &e_current = e[initial][previousAction];
//   cout << "E " << e_current << " ";
  e_current =  p.alpha + (1 - p.alpha) *(p.gamma * p.lambda * e_current);
//   cout << e_current <<endl;

  //set the value function for the current state-action pair
  double &v_current = value[initial][previousAction];
  v_current += delta * e_current + p.alpha * (v_s - v_current);

  //change every other state along the eligibility trace
  StateActionMap::iterator state = e.begin();

  for (; state != e.end(); ++state) {

    bool currentState = state->first == initial;

    ActionValueMap::iterator action = state->second.begin();

    for (; action != state->second.end(); ++action) {

      if (!currentState || !ActionEquality()(action->first,previousAction)) {

        //the current state action pair has alredy been delt with

        double &e_trace = e[state->first][action->first];
//         cout << "e " << e_trace << " ";
        e_trace *= p.lambda * p.gamma;
//         cout << e_trace << endl;

        double &v = value[state->first][action->first];
        v += delta * e_trace;

      }

    }

  }
  
//   printE(e);

  v_s = v_s_prime;

}


void SarsaActionSelector::actionStarted(const AspFluent&) throw() {

  AnswerSet state = reasoner->currentStateQuery(vector<AspRule>());
  initial.clear();
  initial.insert(state.getFluents().begin(), state.getFluents().end());
}


void SarsaActionSelector::actionTerminated(const AspFluent& action) throw() {

  if (final.empty()) { //we have the first state-action pair, we can initialize v_s

    ActionValueMap &initState = value[initial];
    if (initState.find(action) == initState.end()) {
      //use the default value
      value[initial][action] = defval->value(action);
    }

    v_s = value[initial][action];
  }

  AnswerSet state = reasoner->currentStateQuery(vector<AspRule>());
  final.clear();
  final.insert(state.getFluents().begin(), state.getFluents().end());
  previousAction = action;

}

void SarsaActionSelector::episodeEnded() throw() {


  if (!(initial.empty() || final.empty()) ) {
    //update the last state-action pair
    updateValue(0.);
  }
  
  e.clear();
  initial.clear();
  final.clear();
  previousAction = AspFluent("nopreviousaction(0)");
  v_s = 0;
}

void SarsaActionSelector::saveValueInitialState(const std::string& fileName) {
  ofstream initialValue(fileName.c_str(), ofstream::app);
  
  AnswerSet initialAnswerSet= reasoner->currentStateQuery(vector<AspRule>());
  State initialState(initialAnswerSet.getFluents().begin(), initialAnswerSet.getFluents().end());
  
  ActionValueMap &initial_value_map = value[initialState];
  ActionValueMap::iterator action_value = initial_value_map.begin();
  
  time_t rawtime;
  struct tm * timeinfo;
  char time_string[10];
  time(&rawtime);
  timeinfo = localtime(&rawtime);
  strftime (time_string,10,"%R",timeinfo);
  
  stringstream actionNames;
  
  actionNames << time_string << " ";
  
  for(; action_value != initial_value_map.end(); ++action_value) {
    initialValue << action_value->second << " ";
    actionNames << action_value->first.toString() << " ";
  }
  initialValue << actionNames.str() << endl;
  initialValue.close();
}


void SarsaActionSelector::readFrom(std::istream & fromStream) throw() {

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

void SarsaActionSelector::writeTo(std::ostream & toStream) throw() {

  ROS_DEBUG("Storing value function");

  StateActionMap::const_iterator stateIt = value.begin();
  //for each state
  ofstream stat("stats.txt", ios::app);
  AspFluent initialState("pos(10,0,0)");

  for (; stateIt != value.end(); ++stateIt) {


    if (stateIt->first.find(initialState) != stateIt->first.end()) {
      ActionValueMap::const_iterator actionIt= stateIt->second.begin();
      for (; actionIt != stateIt->second.end(); ++actionIt) {
        if (stateIt->first.find(initialState) != stateIt->first.end())
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
