#include <actasp/asp/AspFunction.h>
#include <sstream>
#include <iostream>
#include <clingo.hh>

using namespace std;

namespace actasp {

 AspFunction AspFunction::from_symbol(const Clingo::Symbol &atom) {
  TermContainer terms(atom.arguments().size());
  for (const auto &argument: atom.arguments()) {
    terms.push_back(AspTerm::from_symbol(argument)->clone());
  }
  return AspFunction(atom.name(), terms, atom.is_negative());
}

AspFunction operator ""_a(const char *string, std::size_t size) {
  return AspFunction::from_string(string);
}

AtomLiteral operator ""_al(const char *string, std::size_t size) {
  return AtomLiteral::from_string(string);
}


AspFunction AspFunction::from_string(const char *formula) {
  std::string as_str(formula);
  size_t pos = 0;
  while (std::string::npos != (pos = as_str.find("'", pos)))
  {
    as_str.replace(pos, 1, "\"", 1);
    pos += 1;
  }
  Clingo::Symbol symbol = Clingo::parse_term(as_str.c_str());
  if(symbol.type() != Clingo::SymbolType::Function)
    throw std::invalid_argument("AspFunction: The string " + string(formula) + " is not a valid function");
  return AspFunction::from_symbol(symbol);
}

unsigned int AspFunction::arity() const noexcept {
  return arguments.size();
}

// In general, we avoid string formatting in the ASP type implementations
// because this may change between solvers, but in practice atoms
// are fundamental enough that they're consistent across versions of Clingo
std::string AspFunction::to_string() const noexcept {
  std::stringstream sstream;
    if (negative) {
      sstream << "-";
    }
  sstream << name << "(";
  for (int i = 0; i < arguments.size(); ++i){
    if (i > 0) {
      sstream << ", ";
    }
    sstream << (arguments.at(i).to_string());
  }

  sstream << ")";
  return sstream.str();
}


std::ostream &operator<<(std::ostream &stream, const AspFunction &atom) {
  stream << (std::string)atom;
}

}
