#pragma once

#include <string>
#include <vector>
#include <stdexcept>
#include <set>
#include <functional>

namespace actasp {

class ActionComparator;
class ActionEquality;
  
class AspFluent {
public:

	AspFluent(const std::string& formula) noexcept(false);
	AspFluent(const std::string &name, const std::vector<std::string> &variables, unsigned int timeStep = 0) noexcept;

	unsigned int arity() const noexcept;

	void setTimeStep(unsigned int timeStep) noexcept;
	unsigned int getTimeStep() const noexcept;
	
	std::string getName() const noexcept;
	std::vector<std::string> getParameters() const noexcept;
	
	bool operator<(const AspFluent& other) const noexcept;
	bool operator==(const AspFluent& other) const noexcept;



	std::string toString() const noexcept;
	std::string toString(unsigned int timeStep) const noexcept;
  std::string toString(const std::string& timeStepVar) const noexcept;
  
  operator std::string() const { return this->toString(); } 

private:
	unsigned int timeStep;
	std::string cachedBase; //cached for optimization
	
	friend class ActionComparator;
  friend class ActionEquality;

};

struct ActionComparator : public std::binary_function<const AspFluent&, const AspFluent&, bool>{
 bool operator()(const AspFluent& first, const AspFluent& second) const {
   return first.cachedBase < second.cachedBase;
 }
};

struct ActionEquality : public std::binary_function<const AspFluent&, const AspFluent&, bool>{
 bool operator()(const AspFluent& first, const AspFluent& second) const {
   return first.cachedBase == second.cachedBase;
 }
};

struct TimeStepComparator : public std::binary_function<const AspFluent&, const AspFluent&, bool>{
 bool operator()(const AspFluent& first, const AspFluent& second) const {
   return first.getTimeStep() < second.getTimeStep();
 }
};

typedef std::set<AspFluent, ActionComparator> ActionSet;

struct AspFluentRef {
  
  AspFluentRef(const AspFluent &inobj) :  const_obj(&inobj) {}
  
  operator const AspFluent&() const {
    return *const_obj;
  }
  
  bool operator<(const AspFluent& other) const noexcept {
    return *const_obj < other;
  }
  bool operator==(const AspFluent& other) const noexcept {
    return *const_obj == other;
  }
  
  operator std::string() const { return const_obj->toString(); } 
  
  const AspFluent *const_obj;
};

AspFluent operator ""_f(const char *string, std::size_t size);

}

