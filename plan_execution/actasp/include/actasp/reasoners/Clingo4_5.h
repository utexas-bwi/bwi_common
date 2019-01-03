#pragma once

#include <actasp/FilteringQueryGenerator.h>

namespace actasp {

struct Clingo4_5 : public FilteringQueryGenerator {



  Clingo4_5(const std::string& incrementalVar,
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

  actasp::AnswerSet optimalPlanQuery(const std::vector<actasp::AspFluentRule>& goalRules,
      bool filterActions,
      unsigned int  max_plan_length,
      unsigned int answerset_number,
      bool minimum) const noexcept;

  std::list<actasp::AnswerSet> monitorQuery(const std::vector<actasp::AspFluentRule>& goalRules,
      const AnswerSet& plan) const noexcept;


  AnswerSet currentStateQuery(const std::vector<actasp::AspFluentRule>& query) const noexcept;
  
  std::list<actasp::AnswerSet> filteringQuery(const AnswerSet& currentState, const AnswerSet& plan,const std::vector<actasp::AspFluentRule>& goals);

  std::list<actasp::AnswerSet> genericQuery(const std::vector<actasp::AspFluentRule>& query,
      unsigned int timestep,
      const std::string& fileName,
      unsigned int answerSetsNumber) const noexcept;

  std::list< std::list<AspAtom> > genericQuery(const std::string& query,
      unsigned int timestep,
      const std::string& fileName,
      unsigned int answerSetsNumber) const noexcept;

  std::list<actasp::AnswerSet> genericQuery(const std::vector<actasp::AspFluentRule>& query,
      unsigned int timestep,
      const std::string& fileName,
      unsigned int answerSetsNumber, bool useCopyFiles = true) const noexcept;

  std::list< std::list<AspAtom> > genericQuery(const std::string& query,
      unsigned int timestep,
      const std::string& fileName,
      unsigned int answerSetsNumber, bool useCopyFiles = true) const noexcept;

  actasp::AnswerSet optimizationQuery(const std::string& query, const std::string& fileName) const noexcept;

private:

  std::list<actasp::AnswerSet> genericQuery(const std::string& query,
      unsigned int initialTimeStep,
      unsigned int finalTimeStep,
      const std::string& fileName,
      unsigned int answerSetsNumber, bool useCopyFiles = true) const noexcept;

  std::string makeQuery(const std::string &query, unsigned int initialTimeStep, unsigned int finalTimeStep,
                          const std::string &fileName, unsigned int answerSetsNumber, bool useCopyFiles=true) const noexcept;

  std::string generatePlanQuery(std::vector<actasp::AspFluentRule> goalRules) const noexcept;
  
  std::string generateMonitorQuery(const std::vector<actasp::AspFluentRule>& goalRules, const AnswerSet& plan) const noexcept;

  std::string incrementalVar;
  ActionSet allActions;
  unsigned int max_time;
  std::vector<std::string> linkFiles;
  std::vector<std::string> copyFiles;

};

}


