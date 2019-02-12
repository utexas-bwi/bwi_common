#include <actasp/reasoners/Clingo5_2.h>

#include <actasp/asp/AspAggregate.h>
#include <actasp/asp/AspRule.h>
#include <actasp/AnswerSet.h>
#include <actasp/asp/AspFunction.h>
#include <actasp/asp/AspProgram.h>
#include <actasp/asp/AspMeta.h>
#include <actasp/asp/AspMinimize.h>
#include <actasp/action_utils.h>

#include <algorithm>
#include <iterator>
#include <sstream>
#include <fstream>
#include <cstdlib>
#include <iostream>
#include <limits>

#include <ros/ros.h>
#include <boost/filesystem.hpp>
#include <actasp/filesystem_utils.h>
#include <clingo.hh>


using namespace std;
using boost::filesystem::path;

namespace actasp {


Clingo5_2::Clingo5_2(
    const vector<string> &domain_files
) noexcept :
    domain_files(domain_files),
    incrementalVar("n") {

  Clingo::Control control;
  for (const auto &linkFile: domain_files) {
    assert(boost::filesystem::is_regular_file(linkFile));
    control.load(linkFile.c_str());
  }
  control.ground({{"base", Clingo::SymbolSpan{}}});
  for (const auto &atom: control.symbolic_atoms()) {
    const auto symbol = atom.symbol();
    // Domains can annotate which predicates denote actions using the "action" predicate with the name of the action
    // as the sole string argument
    if (symbol.type() == Clingo::SymbolType::Function) {
      if (symbol.name() == string("action")) {
        const string value = symbol.arguments()[0].string();
        action_names.insert(value);
        fluent_names.insert(value);
      } else if (symbol.name() == string("fluent")) {
        const string value = symbol.arguments()[0].string();
        fluent_names.insert(value);
      }
    }
  }
  if (control.has_const("incvar")) {
    incrementalVar = control.get_const("incvar").string();
  }

}

string literal_to_string(const AspLiteral &literal) {
  if (const auto downcast = dynamic_cast<const AspAtom*>(&literal)) {
    return downcast->to_string();
  } else if (const auto downcast = dynamic_cast<const BinRelation*>(&literal)) {
    return downcast->to_string();
  }
  else {
    assert(false);
  }
}

string rule_to_string(const AspRule *rule) {
  stringstream sstream;
  bool first = true;
  for (const auto &literal: rule->head) {
    if (!first) {
      sstream << ", ";
    }
    sstream << literal_to_string(literal);
    first = false;
  }
  if (!rule->body.empty()) {
    sstream << " :- ";
  }
  first = true;
  for (const auto &literal: rule->body) {
    if (!first) {
      sstream << ", ";
    }
    sstream << literal_to_string(literal);
    first = false;
  }
  sstream << ".";
  return sstream.str();
}

// TODO: Support minimization statements
string minimize_to_string(const AspMinimize *minimize) {
  assert(false);
}

// TODO: Support meta statements
string meta_to_string(const AspMeta *minimize) {
  assert(false);
}

string element_to_string(const AspElement *element) {
  if (const auto r = dynamic_cast<const AspRule*>(element)) {
    return rule_to_string(r);
  } else if (const auto c = dynamic_cast<const AspMinimize*>(element)) {
    return minimize_to_string(c);
  } else if (const auto m = dynamic_cast<const AspMeta*>(element)) {
    return meta_to_string(m);
  } else {
  assert(false);
  }
}

string program_to_string(const AspProgram &program) {
  stringstream stream;
  stream << "#program " << program.name;
  if (!program.variables.empty()) {
    stream << "(";
    bool past_first = false;
    for (const auto &variable: program.variables) {
      if (past_first) {
        stream << ", ";
      }
      stream << variable;
      past_first = true;
    }
    stream << ")";
  }
  stream << "." << std::endl;
  for (const auto &element: program.elements) {
    stream << element_to_string(element);
  }
  return stream.str();
}


struct RuleToString5_2 {

  string operator()(const AspRule &rule) const {

    stringstream ruleStream;

    //iterate over head
    for (int i = 0, size = rule.head.size(); i < size; ++i) {

      ruleStream << literal_to_string(rule.head[i]);

      if (i < (size - 1))
        ruleStream << " | ";
    }

    if (!(rule.body.empty()))
      ruleStream << ":- ";

    //iterate over body
    for (int i = 0, size = rule.body.size(); i < size; ++i) {

      ruleStream << literal_to_string(rule.body[i]);

      if (i < (size - 1))
        ruleStream << ", ";
    }

    if (!(rule.head.empty() && rule.body.empty()))
      ruleStream << "." << std::endl;

    return ruleStream.str();
  }

};


static AspFunction parse_function(const Clingo::Symbol &atom) {
  TermContainer terms(atom.arguments().size());
  for (const auto &argument: atom.arguments()) {
    if (argument.type() == Clingo::SymbolType::Number) {
      terms.push_back(new IntTerm(argument.number()));
    } else if (argument.type() == Clingo::SymbolType::String) {
      terms.push_back(new StringTerm(argument.string()));
    } else if (argument.type() == Clingo::SymbolType::Function) {
      terms.push_back(new AspFunction(parse_function(argument)));
    } else {
      // We don't model SymbolType::Infinum and Suprememum
      // If you need them, add them
      assert(false);
    }
  }
  // FIXME: Actually parse negation!
  return AspFunction(atom.name(), terms);

}

static AnswerSet model_to_answer_set(const Clingo::Model &model, const std::set<string> &fluent_names) {
  cout << "Model" << endl;
  vector<AspFluent> fluents;
  vector<AspAtom> atoms;
  for (auto &atom : model.symbols(Clingo::ShowType::All)) {
      if (atom.type() == Clingo::SymbolType::Function) {
        const auto parsed = parse_function(atom);
        if (fluent_names.find(atom.name()) != fluent_names.end()) {
          fluents.emplace_back(parsed.getName(), parsed.getArguments(), parsed.getNegation());
        } else {
          atoms.push_back(parsed);
        }
      } else {
        assert(false);
      }
  }

  for (auto &atom: model.symbols(Clingo::ShowType::All)) {
    cout << atom << endl;
  }
  cout << endl << endl;
  return AnswerSet(atoms, fluents);
}

static Plan
model_to_plan(const Clingo::Model &model, const std::set<string> &fluent_names, const std::set<string> &action_names) {
  cout << "Model" << endl;
  vector<AspFluent> fluents;
  vector<AspFluent> actions;
  vector<AspAtom> atoms;
  for (auto &atom : model.symbols(Clingo::ShowType::All)) {
    if (atom.type() == Clingo::SymbolType::Function) {
      const auto parsed = parse_function(atom);
      if (action_names.find(atom.name()) != action_names.end()) {
        actions.emplace_back(parsed.getName(), parsed.getArguments(), parsed.getNegation());
      } else if (fluent_names.find(atom.name()) != fluent_names.end()) {
        fluents.emplace_back(parsed.getName(), parsed.getArguments(), parsed.getNegation());
      } else {
        atoms.push_back(parsed);
      }
    } else {
      assert(false);
    }
  }

  for (auto &atom: model.symbols(Clingo::ShowType::All)) {
    cout << atom << endl;
  }
  cout << endl << endl;
  return Plan(atoms, fluents, actions);
}

static string aspString(const vector<AspRule> &query, unsigned int timeStep) {
  stringstream aspStream;
  vector<AspRule> rules;
  for (const auto &rule: query) {
    rules.push_back(add_timestep(rule, timeStep));
  }
  transform(query.begin(), query.end(), ostream_iterator<string>(aspStream), RuleToString5_2());
  return aspStream.str();
}

static inline void add(Clingo::Control &control, const AspProgram &program) {
  vector<const char *> as_strings;
  for (const auto &variable: program.variables) {
    as_strings.push_back(variable.name.c_str());
  }
  stringstream stream;
  stream << "#external query(n)." << std::endl;
  for (const auto &rule: program.elements) {
    stream << element_to_string(rule);
  }
  const auto program_str = stream.str();
  control.add(program.name.c_str(), as_strings, program_str.c_str());
}

template<typename E>
static vector<AspElement *> wrap(const vector<E> &elements) {
  vector<AspElement *> upcast;
  for (const auto &e: elements) {
    upcast.push_back((AspElement *) (&e));
  }
  return upcast;
}

static vector<AspFact> plan_as_facts(const Plan &plan) {
  vector<AspFact> facts;
  for (const auto &action: plan.actions) {
    facts.emplace_back(action);
  }
  return facts;
}

vector<Plan> Clingo5_2::minimalPlanQuery(const vector<AspRule> &goal,
                                         unsigned int max_plan_length,
                                         unsigned int answerset_number,
                                         const vector<AspAtom> *knowledge) const noexcept {

  vector<AspProgram> programs;
  programs.emplace_back("check", wrap<AspRule>(goal), vector<Variable>{Variable("n")});
  if (knowledge) {
    programs.emplace_back("base", wrap<AspAtom>(*knowledge));
  }
  return makePlanQuery(programs, 0, max_plan_length, answerset_number);

}

vector<actasp::Plan> Clingo5_2::lengthRangePlanQuery(const vector<AspRule> &goal,
                                                     unsigned int min_plan_length,
                                                     unsigned int max_plan_length,
                                                     unsigned int answerset_number,
                                                     const vector<AspAtom> *knowledge) const noexcept {
  vector<AspProgram> programs;
  programs.emplace_back("check", wrap<AspRule>(goal), vector<Variable>{Variable("n")});
  if (knowledge) {
    programs.emplace_back("base", wrap<AspAtom>(*knowledge));
  }
  return makePlanQuery(programs, max_plan_length, max_plan_length,
                                                    answerset_number);
}

actasp::Plan Clingo5_2::optimalPlanQuery(const vector<AspRule> &goal,
                                         unsigned int max_plan_length,
                                         unsigned int answerset_number,
                                         const vector<AspAtom> *knowledge) const noexcept {
  vector<AspProgram> programs;
  programs.emplace_back("check", wrap<AspRule>(goal), vector<Variable>{Variable("n")});
  if (knowledge) {
    programs.emplace_back("base", wrap<AspAtom>(*knowledge));
  }
  vector<Plan> allAnswers = makePlanQuery(programs, 0, max_plan_length, answerset_number);
  return allAnswers.front();
}

AnswerSet Clingo5_2::currentStateQuery(const vector<AspRule> &query) const noexcept {
  assert(false);
  // TODO: This used to be aspString(query, 0). Fix it
//  list<AnswerSet> sets = makeQuery(query, 0, 0, "stateQuery", 1, nullptr);

//  return (sets.empty()) ? AnswerSet() : *(sets.begin());
}

struct HasTimeStepZeroInHead5_2 : unary_function<const AspRule &, bool> {

  bool operator()(const AspRule &rule) const {
    if (rule.head.empty())
      return false;
    const auto head_fluent = rule.head_fluents()[0];
    auto as_fluent = AspFluent(head_fluent.getName(), head_fluent.getArguments(), head_fluent.getNegation());
    return as_fluent.getTimeStep() == 0; //I am assuming the heads have a single fluent
    //If the that's not the case, the option --shift has to be added to clingo's command line
  }
};


vector<AnswerSet>
Clingo5_2::genericQuery(const vector<AspRule> &query, unsigned int timestep,
                        const string &fileName,
                        unsigned int answerSetsNumber) const noexcept {
  return genericQuery(query, timestep, fileName, answerSetsNumber, nullptr);
}

vector<actasp::AnswerSet> Clingo5_2::genericQuery(const vector<AspRule> &query,
                                                  unsigned int timeStep,
                                                  const string &fileName,
                                                  unsigned int answerSetsNumber,
                                                  const vector<AspAtom> *knowledge) const noexcept {

  vector<AspRule> base;
  remove_copy_if(query.begin(), query.end(), back_inserter(base), not1(HasTimeStepZeroInHead5_2()));

  // TODO: Think about this interface
  assert(false);
  return makeQuery({}, timeStep, timeStep, answerSetsNumber);

}


vector<actasp::Plan> Clingo5_2::monitorQuery(const vector<AspRule> &goal,
                                             const Plan &plan, const vector<AspAtom> *knowledge) const noexcept {

  //   clock_t kr1_begin = clock();
  vector<AspProgram> programs;
  programs.emplace_back("check", wrap<AspRule>(goal), vector<Variable>{Variable("n")});
  if (knowledge) {
    programs.emplace_back("base", wrap<AspAtom>(*knowledge));
  }
  auto plan_facts = plan_as_facts(plan);
  programs.emplace_back("base", wrap<AspFact>(plan_facts));
  return makePlanQuery(programs, plan.actions.size(), plan.actions.size(), 1);

//   clock_t kr1_end = clock();
//   cout << "Verifying plan time: " << (double(kr1_end - kr1_begin) / CLOCKS_PER_SEC) << " seconds" << endl;
}

vector<Plan>
Clingo5_2::filteringQuery(const AnswerSet &currentState, const Plan &plan, const vector<AspRule> &goal,
                          const vector<AspAtom> *knowledge) const noexcept {

  vector<AspProgram> programs;
  programs.emplace_back("check", wrap<AspRule>(goal), vector<Variable>{Variable("n")});
  if (knowledge) {
    programs.emplace_back("base", wrap<AspAtom>(*knowledge));
  }
  auto plan_facts = plan_as_facts(plan);
  programs.emplace_back("base", wrap<AspFact>(plan_facts));
  assert(false);
  // TODO: Actually put the filtering back in
  //generate a string with all the fluents "0{fluent}1."
  //and add the minimize statement ( eg: :~ pos(x,y,z), ... . [1@1] )
  for (const auto &fluent: plan.actions) {
    TermContainer terms;
    terms.push_back(new AspFluent(fluent));
    AspAggregate(0, terms, 1);
    // TODO: Rewrite this in terms of minimize statement?
    //minimizeString << ":~ " << fluent->to_string() << ". [1]" << endl;
  }

  //make a query that only uses
  return makePlanQuery(programs, plan.fluents.size(), plan.fluents.size(), 0);

}
static bool
inc_solve_loop(Clingo::Control &control, Clingo::SolveHandle &handle, uint32_t initial_step, uint32_t final_step) {
  using Clingo::Number;
  using Clingo::Function;
  using Clingo::TruthValue;
  using Clingo::Control;
  Clingo::SolveResult result;

  int step = 0;
  while (step < final_step && (step == 0 || step < initial_step || !result.is_satisfiable())) {
    try {
      vector<Clingo::Part> parts;
      parts.emplace_back("check", Clingo::SymbolSpan{Number(step)});
      if (step == 0) {
        parts.emplace_back("base", Clingo::SymbolSpan{});
      } else {
        parts.push_back({"step", {Number(step)}});
        control.cleanup();
      }
      control.ground(parts);
      if (step == 0) {
        control.assign_external(Function("query", {Number(step)}), TruthValue::True);
      } else {
        control.release_external(Function("query", {Number(step - 1)}));
        control.assign_external(Function("query", {Number(step)}), TruthValue::True);
      }
      for (auto atom: control.theory_atoms()) {
        std::cout << atom << std::endl;
      }
      for (auto atom: control.symbolic_atoms()) {
        std::cout << atom.symbol() << std::endl;
      }
      std::cout << "_------ END PROGRAM" << std::endl;
      handle = control.solve();
      result = handle.get();
      step++;
      std::cout << handle.get() << std::endl;
    }
    catch (exception const &e) {
      cerr << "Grounding failed with: " << e.what() << endl;
      return false;
    }
  }
  return true;

}

vector<AnswerSet>
Clingo5_2::makeQuery(const vector<AspProgram> &programs, unsigned int initialTimeStep, unsigned int finalTimeStep,
                     unsigned int answerSetsNumber) const noexcept {

  auto logger = [](Clingo::WarningCode, char const *message) {
    cerr << message << endl;
  };

  Clingo::Control control({}, logger, 20);

  control.configuration()["solve"]["models"] = std::to_string(answerSetsNumber).c_str();

  for (const auto &path: domain_files) {
    control.load(path.c_str());
  }

  path
  queryDir = getQueryDirectory(domain_files, {});
  auto queryDirFiles = populateDirectory(queryDir, domain_files);

  for (const auto &program: programs) {
    add(control, program);
    const path part_path = (queryDir / program.name).string() + ".asp";
    std::ofstream query_file(part_path.string());
    query_file << program_to_string(program);
    query_file.close();
  }

  Clingo::SolveHandle handle;
  bool solved = inc_solve_loop(control, handle, initialTimeStep, finalTimeStep);

  if (!solved) {
    return {};
  }
  //const path outputFilePath = (queryDir / fileName).string() + "_output.txt";
  list<AnswerSet> allAnswers;

  vector<AnswerSet> sets;
  for (const auto &model: handle) {
    sets.push_back(model_to_plan(model, fluent_names, action_names));
  }
  return sets;

}

vector<Plan>
Clingo5_2::makePlanQuery(const vector<AspProgram> &programs, unsigned int initialTimeStep, unsigned int finalTimeStep,
                         unsigned int answerSetsNumber) const noexcept {

  auto logger = [](Clingo::WarningCode, char const *message) {
    cerr << message << endl;
  };

  Clingo::Control control({}, logger, 20);

  control.configuration()["solve"]["models"] = std::to_string(answerSetsNumber).c_str();

  for (const auto &path: domain_files) {
    control.load(path.c_str());
  }

  path
  queryDir = getQueryDirectory(domain_files, {});
  auto queryDirFiles = populateDirectory(queryDir, domain_files);

  for (const auto &program: programs) {
    add(control, program);
    const path part_path = (queryDir / program.name).string() + ".asp";
    std::ofstream query_file(part_path.string());
    query_file << program_to_string(program);
    query_file.close();
  }

  Clingo::SolveHandle handle;
  bool solved = inc_solve_loop(control, handle, initialTimeStep, finalTimeStep);

  if (!solved) {
    return {};
  }
  //const path outputFilePath = (queryDir / fileName).string() + "_output.txt";
  list<AnswerSet> allAnswers;

  vector<Plan> plans;
  for (const auto &model: handle) {
    plans.push_back(model_to_plan(model, fluent_names, action_names));
  }
  return plans;
}

}
