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

std::unique_ptr<AspTerm> AspTerm::from_ast(const Clingo::AST::Term &term) {

  using namespace Clingo::AST;
  if (term.data.is<Function>()){
    const auto as_function = term.data.get<Function>();
    return std::unique_ptr<AspTerm>(new AspFunction(std::string(as_function.name),ast_terms_to_container(as_function.arguments)));
  } else if (term.data.is<UnaryOperation>()) {
    const auto as_op = term.data.get<UnaryOperation>();
    const auto parsed_arg = from_ast(as_op.argument);
    const auto *as_func = static_cast<AspFunction*>(parsed_arg.get());
    return std::unique_ptr<AspFunction>(new AspFunction(as_func->name, as_func->arguments, true));
  } else if (term.data.is<Clingo::Symbol>()) {
    const auto as_symbol = term.data.get<Clingo::Symbol>();
    return from_symbol(as_symbol);
  } else if (term.data.is<Clingo::AST::Variable>()) {
    const auto as_variable = term.data.get<Clingo::AST::Variable>();
    return std::unique_ptr<AspTerm>(new Variable(std::string(as_variable.name)));
  } else if (term.data.is<Clingo::AST::Interval>()) {
    const auto as_interval = term.data.get<Clingo::AST::Interval>();
    return std::unique_ptr<AspTerm>(new Interval(*AspTerm::from_ast(as_interval.left), *AspTerm::from_ast(as_interval.right)));
  } else if (term.data.is<Clingo::AST::Pool>()) {
    assert(false);
  } else {
      std::cout << term << std::endl;
      assert(false);
      return {};
  }
}

std::unique_ptr<AspTerm> AspTerm::from_symbol(const Clingo::Symbol &symbol) {
  if (symbol.type() == Clingo::SymbolType::Number) {
    return std::unique_ptr<AspTerm>(new IntTerm(symbol.number()));
  } else if (symbol.type() == Clingo::SymbolType::String) {
    return std::unique_ptr<AspTerm>(new StringTerm(symbol.string()));
  } else if (symbol.type() == Clingo::SymbolType::Function) {
    if (symbol.arguments().empty() && islower(symbol.name()[0])) {
      // Symbolic constants are part of the ASP core language guidelines, but strangely aren't represented
      // separately in Clingo...
      return std::unique_ptr<AspTerm>(new SymbolicConstant(symbol.name()));
    } else if (symbol.arguments().empty() && isupper(symbol.name()[0])) {
      return std::unique_ptr<AspTerm>(new Variable(symbol.name()));
    } else {
      return std::unique_ptr<AspTerm>(new AspFunction(AspFunction::from_symbol(symbol)));
    }
  } else {
    // We don't model SymbolType::Infinum and Suprememum
    // If you need them, add them
    assert(false);
  }
}
}