#include <actasp/asp/AspTerm.h>
#include <actasp/asp/AspFunction.h>
namespace actasp {

std::string SimpleTerm::get_string(const AspTerm &term) {
  return dynamic_cast<const StringTerm&>(term).value;
}

int32_t SimpleTerm::get_int(const AspTerm &term) {
  return dynamic_cast<const IntTerm&>(term).value;
}

Variable SimpleTerm::get_variable(const AspTerm &term) {
  return dynamic_cast<const Variable&>(term);
}

SymbolicConstant SimpleTerm::get_constant(const AspTerm &term) {
  return dynamic_cast<const SymbolicConstant&>(term);
}
}