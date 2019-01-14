#pragma once

#include <actasp/FilteringQueryGenerator.h>

namespace actasp {

struct Clingo5_2 : public FilteringQueryGenerator {



  Clingo5_2(
            const std::vector<std::string>& linkFiles,
            unsigned int max_time = 0
  ) noexcept;

  std::list<actasp::AnswerSet> minimalPlanQuery(const std::vector<actasp::AspFluentRule>& goalRules,
      unsigned int  max_plan_length,
      unsigned int answerset_number, bool actions_only=true) const noexcept;

  std::list<actasp::AnswerSet> lengthRangePlanQuery(const std::vector<actasp::AspFluentRule>& goalRules,
      unsigned int min_plan_length,
      unsigned int  max_plan_length,
      unsigned int answerset_number, bool actions_only=true
  ) const noexcept;

  actasp::AnswerSet optimalPlanQuery(const std::vector<actasp::AspFluentRule>& goalRules,
      unsigned int  max_plan_length,
      unsigned int answerset_number, bool actions_only=true
  ) const noexcept;

  std::list<actasp::AnswerSet> monitorQuery(const std::vector<actasp::AspFluentRule>& goalRules,
      const AnswerSet& plan) const noexcept;

  AnswerSet currentStateQuery(const std::vector<actasp::AspFluentRule>& query) const noexcept;
  
  std::list<actasp::AnswerSet> filteringQuery(const AnswerSet& currentState, const AnswerSet& plan,const std::vector<actasp::AspFluentRule>& goals);

  std::list<actasp::AnswerSet> genericQuery(const std::vector<actasp::AspFluentRule>& query,
      unsigned int timestep,
      const std::string& fileName,
      unsigned int answerSetsNumber) const noexcept;

  std::list<actasp::AnswerSet> genericQuery(const std::vector<actasp::AspFluentRule>& query,
      unsigned int timestep,
      const std::string& fileName,
      unsigned int answerSetsNumber, bool useCopyFiles = true) const noexcept;

  virtual std::set<std::string> get_all_actions() {
    return allActions;
  };


private:

  std::list<actasp::AnswerSet> makeQuery(const std::vector<actasp::AspFluentRule>& query, unsigned int initialTimeStep, unsigned int finalTimeStep,
                          const std::string &fileName, unsigned int answerSetsNumber, bool useCopyFiles=true) const noexcept;

  std::string generateMonitorQuery(const std::vector<actasp::AspFluentRule>& goalRules, const AnswerSet& plan) const noexcept;

  std::string incrementalVar;
  std::set<std::string> allActions;
  unsigned int max_time;
  std::vector<std::string> linkFiles;
  std::vector<std::string> copyFiles;
  bool actions_only;

};

}


