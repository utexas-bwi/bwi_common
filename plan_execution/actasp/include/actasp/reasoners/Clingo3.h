#pragma once

#include <actasp/QueryGenerator.h>

namespace actasp {

struct Clingo3 : public QueryGenerator {

  Clingo3(const std::string& incrementalVar,
          const std::vector<std::string>& linkFiles,
          const std::vector<std::string>& copyFiles,
          const ActionSet& actions,
          unsigned int max_time = 0
  ) noexcept;
  std::list<actasp::AnswerSet> minimalPlanQuery(const std::vector<actasp::AspFluentRule>& goalRules,
      bool filterActions,
      unsigned int  max_plan_length,
      unsigned int answerset_number) const noexcept;

  std::list<actasp::AnswerSet> lengthRangePlanQuery(const std::vector<actasp::AspFluentRule>& goalRules,
      bool filterActions,
      unsigned int min_plan_length,
      unsigned int  max_plan_length,
      unsigned int answerset_number) const noexcept;

  std::list<actasp::AnswerSet> monitorQuery(const std::vector<actasp::AspFluentRule>& goalRules,
      const AnswerSet& plan) const noexcept;


  AnswerSet currentStateQuery(const std::vector<actasp::AspFluentRule>& query) const noexcept;


  std::list<actasp::AnswerSet> genericQuery(const std::vector<actasp::AspFluentRule>& query,
      unsigned int timestep,
      const std::string& fileName,
      unsigned int answerSetsNumber) const noexcept;

  std::list< std::list<AspAtom> > genericQuery(const std::string& query,
      unsigned int timestep,
      const std::string& fileName,
      unsigned int answerSetsNumber) const noexcept;

private:

  std::list<actasp::AnswerSet> genericQuery(const std::string& query,
      unsigned int initialTimeStep,
      unsigned int finalTimeStep,
      const std::string& fileName,
      unsigned int answerSetsNumber) const noexcept;

  std::string makeQuery(const std::string& query,
                                 unsigned int initialTimeStep,
                                 unsigned int finalTimeStep,
                                 const std::string& fileName,
                              unsigned int answerSetsNumber) const noexcept;

  std::string generatePlanQuery(std::vector<actasp::AspFluentRule> goalRules,
                                bool filterActions) const noexcept;

  std::string incrementalVar;
  std::string actionFilter;
  unsigned int max_time;
  std::vector<std::string> linkFiles;
  std::vector<std::string> copyFiles;


};

}


