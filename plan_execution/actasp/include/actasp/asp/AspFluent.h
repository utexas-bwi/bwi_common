#pragma once

#include <string>
#include <vector>
#include <stdexcept>
#include <set>
#include <functional>
#include <boost/variant.hpp>
#include <actasp/asp/AspAtom.h>
#include <iostream>

namespace actasp {

/**
 * @brief A convenience subclass of AspAtom that makes working with timesteps
 * and timestep variables easier.
 */
class AspFluent: public AspAtom {
public:

	AspFluent(const std::string& formula) noexcept(false): AspAtom(formula) {
	  if (arguments[arguments.size()-1].type() == typeid(std::string)) {
      throw std::invalid_argument("AspFluent: The string " + formula + " must have a number or variable as it's last argument.");
	  }
	};
	AspFluent(const std::string &name, const std::vector<Argument> &variables, const std::vector<Negation> &negation={}) noexcept(false): AspAtom(name, variables, negation) {
		if (variables[variables.size()-1].type() == typeid(std::string)) {
      throw std::invalid_argument("AspFluent: The formula for " + name + " must have a number or variable as it's last argument.");
    }
	};
	AspFluent(const std::string &name, const std::vector<Argument> &variables, uint32_t timeStep, const std::vector<Negation> &negation={}) noexcept: AspAtom(name, variables, negation){
	  this->arguments.emplace_back(timeStep);
	};
	AspFluent(const std::string &name, const std::vector<Argument> &variables, Variable timeStep, const std::vector<Negation> &negation={}) noexcept: AspAtom(name, variables, negation){
		this->arguments.emplace_back(timeStep);
	};

	uint32_t getTimeStep() const noexcept(false);
	Variable getTimeStepVariable() const noexcept(false);

	bool operator<(const AspFluent& other) const noexcept;
	bool operator>(const AspFluent& other) const noexcept;

};


typedef std::set<AspFluent> ActionSet;


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

struct NoTimeStepComparator : public std::binary_function<const AspFluent&, const AspFluent&, bool>{
	bool operator()(const AspFluent& first, const AspFluent& second) const {
    std::vector<AspAtom::Argument> first_without_time{first.getArguments().begin(), first.getArguments().end() - 1};
    std::vector<AspAtom::Argument> second_without_time{second.getArguments().begin(), second.getArguments().end() - 1};
		return first.getNegation() < second.getNegation() || first.getName() < second.getName() || first_without_time <
																																																	 second_without_time;
	}
};

struct NoTimeStepEquality : public std::binary_function<const AspFluent&, const AspFluent&, bool>{
  bool operator()(const AspFluent& first, const AspFluent& second) const {
    std::vector<AspAtom::Argument> first_without_time{first.getArguments().begin(), first.getArguments().end() - 1};
    std::vector<AspAtom::Argument> second_without_time{second.getArguments().begin(), second.getArguments().end() - 1};
    return first.getNegation() == second.getNegation() && first.getName() == second.getName() &&  first_without_time ==
                                                                                               second_without_time;
  }
};

struct NoTimeStepComparatorRef : public std::binary_function<const AspFluentRef&, const AspFluentRef&, bool>{
  bool operator()(const AspFluentRef& first_ref, const AspFluentRef& second_ref) const {
    const auto &first = *first_ref.const_obj;
    const auto &second = *second_ref.const_obj;
    std::vector<AspAtom::Argument> first_without_time{first.getArguments().begin(), first.getArguments().end() - 1};
    std::vector<AspAtom::Argument> second_without_time{second.getArguments().begin(), second.getArguments().end() - 1};
    return first.getNegation() < second.getNegation() || first.getName() < second.getName() || first_without_time <
                                                                                               second_without_time;
  }
};

struct NoTimeStepEqualityRef : public std::binary_function<const AspFluentRef&, const AspFluentRef&, bool>{
  bool operator()(const AspFluentRef& first_ref, const AspFluentRef& second_ref) const {
    const auto &first = *first_ref.const_obj;
    const auto &second = *second_ref.const_obj;
    std::vector<AspAtom::Argument> first_without_time{first.getArguments().begin(), first.getArguments().end() - 1};
    std::vector<AspAtom::Argument> second_without_time{second.getArguments().begin(), second.getArguments().end() - 1};
    return first.getNegation() == second.getNegation() && first.getName() == second.getName() &&  first_without_time ==
                                                                                                  second_without_time;
  }
};

AspFluent operator ""_f(const char *string, std::size_t size);

}

