#include <actasp/asp/AspAtom.h>
#include <sstream>
#include <iostream>

using namespace std;

namespace actasp {

static inline void ltrim(std::string &s) {
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](int ch) {
    return !std::isspace(ch);
  }));
}

// trim from start (copying)
static inline std::string ltrim_copy(std::string s) {
  ltrim(s);
  return s;
}

static inline void interpret_param(const std::string &param, std::vector<AspAtom::Argument> &variables) {
  const auto trimmed = ltrim_copy(param);
  try {
    // Try to interpret as int
    int asInt = std::stoi(trimmed);
    variables.emplace_back(asInt);
  } catch (std::exception &e) {
    // Is the parameter a quoted string?
    const auto front = trimmed.front();
    if (front == '"' || front == '\'') {
      const auto close_quote = trimmed.find(front, 1);
      variables.emplace_back(trimmed.substr(1, close_quote - 1));
    } else {
      variables.emplace_back(Variable(trimmed));
    }
  }
}

AspAtom operator ""_a(const char *string, std::size_t size) {
  return AspAtom(string);
}

AspAtom::AspAtom(const std::string& formula) {
  size_t first_par = formula.find_first_of('(');
  size_t last_par = formula.find_last_of(')');
  if(first_par == string::npos)
    throw std::invalid_argument("AspAtom: The string " + formula + " does not contain a '(', therefore is not a valid fluent");

  if(last_par == string::npos)
    throw std::invalid_argument("The string " + formula + " does not contain a ')', therefore is not a valid fluent");


  // See if there are any negations before the first paren

  size_t first_not = formula.find("not");
  size_t last_negation = 0;
  if (first_not == 0) {
    negation.emplace_back(Default);
    last_negation = 4;
    size_t second_not = formula.find("not", last_negation);
    if (second_not == 4) {
      negation.emplace_back(Default);
      last_negation = 8;
    }
  }

  first_not = formula.find('-', last_negation);
  if (first_not == last_negation) {
    negation.emplace_back(Classical);
    last_negation += 1;
    size_t second_not = formula.find('-', last_negation);
    if (second_not == last_negation) {
      negation.emplace_back(Classical);
      last_negation += 1;
    }
  }

  name = formula.substr(last_negation, first_par - last_negation);
  size_t start = first_par + 1;
  size_t comma = formula.find_first_of(',', start);

  if (comma == string::npos && last_par - first_par != 1) {
    const auto &param = formula.substr(first_par + 1, last_par - first_par - 1);
    interpret_param(param, arguments);
    return;
  }

  if (last_par - first_par == 1) {
    return;
  }

  size_t endi = comma;
  do {
    const auto &nextParam = formula.substr(start,endi-start);
    interpret_param(nextParam, arguments);
    start = endi + 1;
    endi = formula.find_first_of(',',endi+1);
    if (endi == string::npos) {
      endi = last_par;
    }
  } while (start < formula.size());

}


unsigned int AspAtom::arity() const noexcept {
  return arguments.size();
}

std::string AspAtom::getName() const noexcept {
  return name;
}
  
std::vector<AspAtom::Argument> AspAtom::getArguments() const noexcept {
    return arguments;
}


  std::string ArgumentToString::operator()(const std::string &string) const {
    return "\"" + string + "\"";
  }
  std::string ArgumentToString::operator()(const Variable &variable) const {
    return variable.name;
  }
  std::string ArgumentToString::operator()(const int32_t &integer) const {
    return std::to_string(integer);
  }

// In general, we avoid string formatting in the ASP type implementations
// because this may change between solvers, but in practice atoms
// are fundamental enough that they're consistent across versions of Clingo
std::string AspAtom::to_string() const noexcept {
  std::stringstream sstream;
  for (const auto &negation: this->negation) {
    if (negation == Default) {
      sstream << "not ";
    } else {
      sstream << "-";
    }
  }
  sstream << name << "(";
  for (int i = 0; i < arguments.size(); ++i){
    if (i > 0) {
      sstream << ", ";
    }
    sstream << boost::apply_visitor(ArgumentToString{},arguments[i]);
  }

  sstream << ")";
  return sstream.str();
}

vector<Negation> AspAtom::getNegation() const noexcept {
  return negation;
}

std::ostream &operator<<(std::ostream &stream, const AspAtom &atom) {
  stream << (std::string)atom;
}

}
