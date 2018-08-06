#ifndef actasp_Action_h__guard
#define actasp_Action_h__guard

#include <actasp/AspFluent.h>
#include <actasp/ResourceManager.h>

#include <string>
#include <vector>
#include <boost/shared_ptr.hpp>

namespace actasp {

struct Action {	

	typedef boost::shared_ptr<Action> Ptr;
	virtual int paramNumber() const = 0;
	
	virtual std::string getName() const = 0;
	
	virtual void run() = 0;
	
	virtual bool hasFinished() const = 0;
  
    virtual bool hasFailed() const {return false;}

	virtual Action *cloneAndInit(const actasp::AspFluent & fluent) const = 0;

	virtual void configureWithResources(ResourceManager *resourceManager) {}

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
	
 virtual std::vector<std::string> getParameters() const = 0;
};

}

#endif
