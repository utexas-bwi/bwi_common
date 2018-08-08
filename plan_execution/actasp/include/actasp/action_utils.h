
#ifndef actasp_action_util_h__guard
#define actasp_action_util_h__guard

#include <actasp/Action.h>
#include <actasp/AnswerSet.h>

#include <list>
#include <utility>
#include <set>
#include <functional>

namespace actasp {


struct ActionMapDeepCopy {
	
	std::pair<std::string, Action *> operator()(const std::pair<std::string, Action *> &other) {
		
		return std::make_pair(other.first, other.second->clone());
	}
};

struct ActionMapDelete {
	
	void operator()(const std::pair<std::string, Action *> &other) {
		delete other.second;
	}
};

struct IsAnAction : public std::unary_function<const AspFluent&,bool>{
  
  IsAnAction(const ActionSet& actions);
  
  bool operator()(const AspFluent& fluent) const;
  
  std::set<std::string> actionNames;
};

AnswerSet planToAnswerSet(const std::list<Action::Ptr>& plan);

// DEPRECATED
ActionSet actionMapToSet(const std::map<std::string, Action *>& actionMap);
ActionSet actionMapToSet(const std::map<std::string, ActionFactory>& actionMap);

}


#endif
