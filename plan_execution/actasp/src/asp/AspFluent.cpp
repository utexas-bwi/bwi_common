#include <actasp/asp/AspFluent.h>

#include <sstream>
#include <cstdlib> //for atoi

using namespace std;

namespace actasp {

AspFluent operator ""_f(const char *string, std::size_t size) {
  return AspFluent(string);
}

unsigned int AspFluent::getTimeStep() const noexcept(false) {
	return boost::get<int>(arguments[arguments.size() - 1]);
}

Variable AspFluent::getTimeStepVariable() const noexcept(false) {
	return boost::get<Variable>(arguments[arguments.size() - 1]);
}


bool AspFluent::operator<(const AspFluent& other) const noexcept{
	bool vars_less = std::lexicographical_compare(arguments.begin(), arguments.end() - 1, other.arguments.begin(), other.arguments.end() - 1);
	return getTimeStep() < other.getTimeStep() || name < other.name ||  ((name == other.name) && vars_less);
}

bool AspFluent::operator>(const AspFluent& other) const noexcept{
	return !(*this == other || *this < other);
}

}