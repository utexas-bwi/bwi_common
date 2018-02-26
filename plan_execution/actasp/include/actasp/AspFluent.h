#ifndef actasp_AspFluent_h__guard
#define actasp_AspFluent_h__guard

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

	AspFluent(const std::string& formula) throw (std::invalid_argument);
	AspFluent(const std::string &name, const std::vector<std::string> &variables, unsigned int timeStep = 0) throw ();

	unsigned int arity() const throw ();

	void setTimeStep(unsigned int timeStep) throw();
	unsigned int getTimeStep() const throw();
	
	std::string getName() const throw();
	std::vector<std::string> getParameters() const throw();
	
	bool operator<(const AspFluent& other) const throw();
	bool operator==(const AspFluent& other) const throw();

	std::string toString() const throw();
	std::string toString(unsigned int timeStep) const throw();
  std::string toString(const std::string& timeStepVar) const throw();
  
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
  
  bool operator<(const AspFluent& other) const throw() {
    return *const_obj < other;
  }
  bool operator==(const AspFluent& other) const throw() {
    return *const_obj == other;
  }
  
  operator std::string() const { return const_obj->toString(); } 
  
  const AspFluent *const_obj;
};

}
#endif
