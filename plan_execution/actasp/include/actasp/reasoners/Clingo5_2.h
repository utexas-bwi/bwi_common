#pragma once

#include <actasp/FilteringQueryGenerator.h>

namespace Clingo {
class Model;
}

namespace actasp {

class AspProgram;

struct Clingo5_2 : public FilteringQueryGenerator {

  Clingo5_2(
      const std::vector<std::string> &domain_files
  ) noexcept;

  std::vector<Plan> minimalPlanQuery(const std::vector<AspRule> &goalRules,
                                     unsigned int  max_plan_length,
                                     unsigned int answerset_number,
                                     const std::vector<AspAtom> *knowledge = nullptr) const noexcept;

  std::vector<Plan> lengthRangePlanQuery(const std::vector<AspRule> &goalRules,
                                         unsigned int min_plan_length,
                                         unsigned int  max_plan_length,
                                         unsigned int answerset_number,
                                         const std::vector<AspAtom> *knowledge = nullptr
  ) const noexcept;

  Plan optimalPlanQuery(const std::vector<AspRule> &goalRules,
                        unsigned int  max_plan_length,
                        unsigned int answerset_number,
                        const std::vector<AspAtom> *knowledge = nullptr
  ) const noexcept;

  std::vector<Plan> monitorQuery(const std::vector<AspRule> &goalRules,
                                 const Plan &plan, const std::vector<AspAtom> *knowledge = nullptr) const noexcept;

  AnswerSet currentStateQuery(const std::vector<AspRule> &query) const noexcept override;

  std::vector<Plan> filteringQuery(const AnswerSet &currentState, const Plan &plan, const std::vector<AspRule> &goal,
                                   const std::vector<AspAtom> *knowledge = nullptr) const noexcept;

  std::vector<AnswerSet> genericQuery(const std::vector<AspRule> &query,
                                      unsigned int timestep,
                                      const std::string& fileName,
                                      unsigned int answerSetsNumber,
                                      const std::vector<AspAtom> *knowledge) const noexcept;

  std::vector<AnswerSet> genericQuery(const std::vector<AspRule> &query,
                                      unsigned int timestep,
                                      const std::string &fileName,
                                      unsigned int answerSetsNumber) const noexcept;


  std::set<std::string> get_all_actions() override {
    return action_names;
  };


private:

  std::vector<AnswerSet>
  makeQuery(const std::vector<AspProgram> &programs, unsigned int initialTimeStep, unsigned int finalTimeStep,
            unsigned int answerSetsNumber) const noexcept;

  std::vector<Plan>
  makePlanQuery(const std::vector<AspProgram> &programs, unsigned int initialTimeStep, unsigned int finalTimeStep,
                unsigned int answerSetsNumber) const noexcept;

  std::string incrementalVar;
  std::set<std::string> action_names;
  std::set<std::string> fluent_names;
  std::vector<std::string> domain_files;

};

}


