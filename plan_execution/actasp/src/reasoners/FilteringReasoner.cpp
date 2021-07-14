#include <actasp/reasoners/FilteringReasoner.h>

#include <actasp/FilteringQueryGenerator.h>

#include <vector>
#include <sstream>
#include <boost/graph/graph_concepts.hpp>


using namespace std;

namespace actasp {
  
/**
 * @brief Decorate a reasoner with filtering reasoner capabilities
 * @param queryGenerator
 * @param max_n
 * @param allActions
 */
FilteringReasoner::FilteringReasoner(FilteringQueryGenerator *queryGenerator,unsigned int max_n,const ActionSet& allActions) :
  Reasoner(queryGenerator,max_n,allActions),
  clingo(queryGenerator) {}


GraphPolicy *FilteringReasoner::computePolicy(const std::vector<actasp::AspRule>& goal, double suboptimality) const noexcept(false) {
  auto *p = new GraphPolicy(allActions);
  computePolicyHelper(goal,suboptimality,p);
  return p;
}


struct AnswerSetStateComparator {
  bool operator()(const AnswerSet &first, const AnswerSet& second) const {
    return first.getFluentsAtTime(0).size() < second.getFluentsAtTime(0).size();
  }

};

//Filters the state fluents to only keep the ones necessary for the purpose of the plans:
// plans = vector of list of actions (example: "north(1)" , "east(2)" ..)
// goals = vector of fluents that have to be true at the final step (example: "not pos(3, 9, n)" to mean "be there"..)
//domain needs to have #show for fluents and actions.
// TODO:: Make it possible to add a "nofilter.txt" file in the domain directory to specify which fluents (name/arity) have to stay there no matter what.
AnswerSet FilteringReasoner::filterState(const std::vector<actasp::AnswerSet>& plans, const std::vector<actasp::AspRule>& goals) {

  //input checking:
  if (plans.empty() || goals.empty()) {
    return AnswerSet();
  }

  //get all the known fluents of the current state:
  AnswerSet currentState = Reasoner::currentStateQuery(vector<AspRule>());

  set<AspFluent> result;

  auto planIt = plans.begin();
  for (; planIt != plans.end(); ++planIt) {
    //make a query that only uses the domain and what I created, not current.asp
    std::list<actasp::AnswerSet> listResult = clingo->filteringQuery(currentState,*planIt,goals);
    if (!listResult.empty()) {
      std::list<actasp::AnswerSet>::const_iterator best = min_element(listResult.begin(), listResult.end(),AnswerSetStateComparator());
      set<AspFluent> statezero = best->getFluentsAtTime(0);
      auto statezeroIt = statezero.begin();
      for (;statezeroIt != statezero.end(); ++statezeroIt) {
        result.insert(*statezeroIt);
      }
    }

  }

  return AnswerSet(result.begin(), result.end());

} //end of Clingo::filterState method

}
