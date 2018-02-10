#ifndef actexec_ActionFactory_h__guard
#define actexec_ActionFactory_h__guard

#include "actasp/actaspfwd.h"

#include <string>
#include <map>
#include <stdexcept>

namespace plan_exec {

struct ActionFactory {
	
	typedef std::map<std::string, actasp::Action*> ActionMap;
	
	explicit ActionFactory(actasp::Action *act); //both simulation and real world
	ActionFactory(actasp::Action *act, bool simulation); //either simulation or real world
	
	static actasp::Action *byName(const std::string& name) throw (std::runtime_error);
	static ActionMap actions();
	
	static void setSimulation(bool value);
	
private:
	static bool simulation;
	static ActionMap &realActions();
	static ActionMap &simulatedActions();
	static ActionMap &bothActions();
};

}

#endif