#pragma once

#include <actasp/FilteringQueryGenerator.h>

namespace actasp {

struct Clingo5_2 : public FilteringQueryGenerator {

  Clingo5_2(
            const std::vector<std::string>& linkFiles,
            const std::vector<actasp::AspFunction> &knowledge
  ) noexcept;

  std::list<actasp::AnswerSet> minimalPlanQuery(const std::vector<actasp::AspRule>& goalRules,
      unsigned int  max_plan_length,
      unsigned int answerset_number, bool actions_only=true) const noexcept;

  std::list<actasp::AnswerSet> lengthRangePlanQuery(const std::vector<actasp::AspRule>& goalRules,
      unsigned int min_plan_length,
      unsigned int  max_plan_length,
      unsigned int answerset_number, bool actions_only=true
  ) const noexcept;

  actasp::AnswerSet optimalPlanQuery(const std::vector<actasp::AspRule>& goalRules,
      unsigned int  max_plan_length,
      unsigned int answerset_number, bool actions_only=true
  ) const noexcept;

  std::list<actasp::AnswerSet> monitorQuery(const std::vector<actasp::AspRule>& goalRules,
      const AnswerSet& plan) const noexcept;

  AnswerSet currentStateQuery(const std::vector<actasp::AspRule>& query) const noexcept;
  
  std::list<actasp::AnswerSet> filteringQuery(const AnswerSet& currentState, const AnswerSet& plan,const std::vector<actasp::AspRule>& goals);

  std::list<actasp::AnswerSet> genericQuery(const std::vector<actasp::AspRule>& query,
      unsigned int timestep,
      const std::string& fileName,
      unsigned int answerSetsNumber) const noexcept;

  std::list<actasp::AnswerSet> genericQuery(const std::vector<actasp::AspRule>& query,
      unsigned int timestep,
      const std::string& fileName,
      unsigned int answerSetsNumber, bool useCopyFiles = true) const noexcept;

  virtual std::set<std::string> get_all_actions() {
    return action_names;
  };


private:

  std::list<actasp::AnswerSet> makeQuery(const std::vector<actasp::AspRule>& query, unsigned int initialTimeStep, unsigned int finalTimeStep,
                          const std::string &fileName, unsigned int answerSetsNumber, bool useCopyFiles=true) const noexcept;

  std::string generateMonitorQuery(const std::vector<actasp::AspRule>& goalRules, const AnswerSet& plan) const noexcept;

  std::string incrementalVar;
  std::set<std::string> action_names;
  std::set<std::string> fluent_names;
  std::vector<std::string> linkFiles;
  std::vector<std::string> copyFiles;
  const std::vector<actasp::AspFunction> &knowledge;
  bool actions_only;

};

}


