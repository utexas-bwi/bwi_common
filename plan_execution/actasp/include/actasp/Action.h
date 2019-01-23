#pragma once

#include <actasp/asp/AspFluent.h>
#include <actasp/ResourceManager.h>

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>

namespace actasp {

struct Action {	

	typedef std::shared_ptr<Action> Ptr;
	virtual int paramNumber() const = 0;
	
	virtual std::string getName() const = 0;
	
	virtual void run() = 0;
	
	virtual bool hasFinished() const = 0;
  
    virtual bool hasFailed() const {return false;}

    // DEPRECATED
	virtual Action *cloneAndInit(const actasp::AspFluent & fluent) const = 0;

    // DEPRECATED
	virtual Action *clone() const =0;
	
	std::string toASP(unsigned int timeStep) const;
  
    AspFluent toFluent(unsigned int timeStep) const;
	
	bool operator==(const Action *other) const {
		return this->toASP(0) == other->toASP(0);
	}
	
	bool operator<(const Action *other) const {
		return this->toASP(0) < other->toASP(0);
	}
	
	virtual ~Action() {}
	
private:
	
 virtual std::vector<AspAtom::Argument> getParameters() const = 0;
};

typedef std::function<std::unique_ptr<actasp::Action>(const actasp::AspFluent &, actasp::ResourceManager &)> ActionFactory;


}


