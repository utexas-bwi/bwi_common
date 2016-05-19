#ifndef actasp_AnswerSet_h__guard
#define actasp_AnswerSet_h__guard

#include <actasp/AspFluent.h>

#include <string>
#include <set>
#include <list>
#include <map>
#include <vector>
#include <algorithm>
#include <stdexcept>

namespace actasp {

class Action;

class AnswerSet {

public:
  
  AnswerSet() : satisfied(false), fluents() {}
  
  template <typename Iterator>
  AnswerSet(Iterator from, Iterator to) throw() : satisfied(true), fluents(from,to) {
    std::sort(fluents.begin(), fluents.end(), TimeStepComparator());
  }

  typedef std::vector<actasp::AspFluent> FluentSet;
	
	bool isSatisfied() const throw();
	
	bool contains(const actasp::AspFluent& fluent) const throw();
	
	std::list<Action *> instantiateActions(const std::map<std::string, actasp::Action*> &actionMap) const throw(std::logic_error);
	
	const FluentSet& getFluents() const throw() { return fluents;}
	
	std::set<actasp::AspFluent> getFluentsAtTime(unsigned int timeStep) const throw();
  
  unsigned int maxTimeStep() const throw(std::logic_error);

private:
  
	bool satisfied;
	FluentSet fluents;
};
	
}
#endif
