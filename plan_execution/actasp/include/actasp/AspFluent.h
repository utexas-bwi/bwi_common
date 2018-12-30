#pragma once

#include <string>
#include <vector>
#include <stdexcept>
#include <set>
#include <functional>
#include <boost/variant.hpp>
#include <actasp/AspAtom.h>
#include <iostream>

namespace actasp {

template<typename FluentClass>
struct ActionComparator : public std::binary_function<const FluentClass&, const FluentClass&, bool>{
	bool operator()(const FluentClass& first, const FluentClass& second) const {
		return (std::string)first < (std::string)second;
	}
};

template<typename FluentClass>
struct ActionEquality : public std::binary_function<const FluentClass&, const FluentClass&, bool>{
	bool operator()(const FluentClass& first, const FluentClass& second) const {
		return (std::string)first == (std::string)second;
	}
};


class AspFluent: public AspAtom {
public:

	AspFluent(const std::string& formula) noexcept(false): AspAtom(formula) {
	  if (variables[variables.size()-1].type() == typeid(std::string)) {
      throw std::invalid_argument("AspFluent: The string " + formula + " must have a number or variable as it's last argument.");
	  }
	};
	AspFluent(const std::string &name, const std::vector<Argument> &variables) noexcept(false): AspAtom(name, variables) {
		if (variables[variables.size()-1].type() == typeid(std::string)) {
      throw std::invalid_argument("AspFluent: The formula for " + name + " must have a number or variable as it's last argument.");
    }
	};
	AspFluent(const std::string &name, const std::vector<Argument> &variables, uint32_t timeStep) noexcept: AspAtom(name, variables){
	  this->variables.emplace_back(timeStep);
	};
	AspFluent(const std::string &name, const std::vector<Argument> &variables, Variable timeStep) noexcept: AspAtom(name, variables){
		this->variables.emplace_back(timeStep);
	};

	uint32_t getTimeStep() const noexcept(false);
	Variable getTimeStepVariable() const noexcept(false);

	bool operator<(const AspFluent& other) const noexcept;
	bool operator>(const AspFluent& other) const noexcept;

private:
	
	friend class ActionComparator<AspFluent>;
  friend class ActionEquality<AspFluent>;

};


typedef std::set<AspFluent, ActionComparator<AspFluent>> ActionSet;


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
  
  operator std::string() const { return const_obj->to_string(); }
  
  const AspFluent *const_obj;
};

struct TimeStepComparator : public std::binary_function<const AspFluent&, const AspFluent&, bool>{
  bool operator()(const AspFluent& first, const AspFluent& second) const {
    return first.getTimeStep() < second.getTimeStep();
  }
};

AspFluent operator ""_f(const char *string, std::size_t size);

}

