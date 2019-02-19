#include <actasp/asp/AspFluent.h>
#include <sstream>
using namespace std;

namespace actasp {

AspFluent operator ""_f(const char *string, std::size_t size) {
  return AspFluent::from_string(string);
}

FluentLiteral operator ""_fl(const char *string, std::size_t size) {
	return FluentLiteral::from_string(string);
}

unsigned int AspFluent::getTimeStep() const noexcept(false) {
	return dynamic_cast<const IntTerm*>(&arguments.back())->value;
}

SymbolicConstant AspFluent::getTimeStepVariable() const noexcept(false) {
	return *dynamic_cast<const SymbolicConstant*>(&arguments.back());
}


bool AspFluent::operator<(const AspFluent& other) const noexcept{
	bool vars_less = std::lexicographical_compare(arguments.begin(), arguments.end() - 1, other.arguments.begin(), other.arguments.end() - 1);
	return getTimeStep() < other.getTimeStep() || name < other.name ||  ((name == other.name) && vars_less);
}

bool AspFluent::operator>(const AspFluent& other) const noexcept{
	return !(*this == other || *this < other);
}

}