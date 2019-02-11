#pragma once

#include <string>
#include <vector>
#include <stdexcept>
#include <set>
#include <functional>
#include <boost/variant.hpp>
#include <actasp/asp/AspFunction.h>
#include <iostream>

namespace actasp {

/**
 * @brief A convenience subclass of AspAtom that makes working with timesteps
 * and timestep variables easier.
 */
class AspFluent: public AspFunction {
public:

	AspFluent(const std::string& formula) noexcept(false): AspFunction(formula) {
	  const auto last_argument = &arguments.back();
	  if (!dynamic_cast<IntTerm*>(last_argument) && !dynamic_cast<SymbolicConstant*>(last_argument)) {
      throw std::invalid_argument("AspFluent: The string " + formula + " must have a number or variable as it's last argument.");
	  }
	};
	AspFluent(const std::string &name, const TermContainer &variables, const std::vector<Negation> &negation={}) noexcept(false): AspFunction(name, variables, negation) {
    const auto last_argument = &arguments.back();
    if (!dynamic_cast<IntTerm*>(last_argument) && !dynamic_cast<SymbolicConstant*>(last_argument)) {
      throw std::invalid_argument("AspFluent: The formula for " + name + " must have a number or variable as it's last argument.");
    }
	};
	AspFluent(const std::string &name, const TermContainer &variables, const std::vector<Negation> &negation, uint32_t timeStep) noexcept: AspFunction(name, variables, negation){
	  this->arguments.push_back(new IntTerm(timeStep));
	};
	AspFluent(const std::string &name, const TermContainer &variables, const std::vector<Negation> &negation, Variable timeStep) noexcept: AspFunction(name, variables, negation){
		this->arguments.push_back(new Variable(timeStep));
	};

  inline AspTerm* clone() const override {
    return new AspFluent(*this);
  }

  inline AspLiteral *literal_clone() const override {
    return new AspFluent(*this);
  }

	uint32_t getTimeStep() const noexcept(false);
	SymbolicConstant getTimeStepVariable() const noexcept(false);

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
    TermContainer first_without_time{first.getArguments().begin(), first.getArguments().end() - 1};
    TermContainer second_without_time{second.getArguments().begin(), second.getArguments().end() - 1};
		return first.getNegation() < second.getNegation() || first.getName() < second.getName() || first_without_time <
																																																	 second_without_time;
	}
};

struct NoTimeStepEquality : public std::binary_function<const AspFluent&, const AspFluent&, bool>{
  bool operator()(const AspFluent& first, const AspFluent& second) const {
    TermContainer first_without_time{first.getArguments().begin(), first.getArguments().end() - 1};
    TermContainer second_without_time{second.getArguments().begin(), second.getArguments().end() - 1};
    return first.getNegation() == second.getNegation() && first.getName() == second.getName() &&  first_without_time ==
                                                                                               second_without_time;
  }
};

struct NoTimeStepComparatorRef : public std::binary_function<const AspFluentRef&, const AspFluentRef&, bool>{
  bool operator()(const AspFluentRef& first_ref, const AspFluentRef& second_ref) const {
    const auto &first = *first_ref.const_obj;
    const auto &second = *second_ref.const_obj;
    TermContainer first_without_time{first.getArguments().begin(), first.getArguments().end() - 1};
    TermContainer second_without_time{second.getArguments().begin(), second.getArguments().end() - 1};
    return first.getNegation() < second.getNegation() || first.getName() < second.getName() || first_without_time <
                                                                                               second_without_time;
  }
};

struct NoTimeStepEqualityRef : public std::binary_function<const AspFluentRef&, const AspFluentRef&, bool>{
  bool operator()(const AspFluentRef& first_ref, const AspFluentRef& second_ref) const {
    const auto &first = *first_ref.const_obj;
    const auto &second = *second_ref.const_obj;
    TermContainer first_without_time{first.getArguments().begin(), first.getArguments().end() - 1};
    TermContainer second_without_time{second.getArguments().begin(), second.getArguments().end() - 1};
    return first.getNegation() == second.getNegation() && first.getName() == second.getName() &&  first_without_time ==
                                                                                                  second_without_time;
  }
};

AspFluent operator ""_f(const char *string, std::size_t size);

}

