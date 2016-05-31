
#ifndef actasp_Clingo4_2_h__guard
#define actasp_Clingo4_2_h__guard

#include <actasp/FilteringQueryGenerator.h>

namespace actasp {

struct Clingo4_2 : public FilteringQueryGenerator {

  Clingo4_2(const std::string& incrementalVar,
          const std::string& queryDir,
          const std::string& domainDir,
          const ActionSet& actions,
          unsigned int max_time = 0
  ) throw();

  Clingo4_2(const std::string& incrementalVar,
          const std::string& queryDir,
          const std::string& domainDir,
          const ActionSet& actions,
          const std::string& currentFilePath,
          unsigned int max_time = 0
  ) throw();

  std::list<actasp::AnswerSet> minimalPlanQuery(const std::vector<actasp::AspRule>& goalRules,
      bool filterActions,
      unsigned int  max_plan_length,
      unsigned int answerset_number) const throw();

  std::list<actasp::AnswerSet> lengthRangePlanQuery(const std::vector<actasp::AspRule>& goalRules,
      bool filterActions,
      unsigned int min_plan_length,
      unsigned int  max_plan_length,
      unsigned int answerset_number) const throw();

  actasp::AnswerSet optimalPlanQuery(const std::vector<actasp::AspRule>& goalRules,
      bool filterActions,
      unsigned int  max_plan_length,
      unsigned int answerset_number,
      bool minimum) const throw();

  std::list<actasp::AnswerSet> monitorQuery(const std::vector<actasp::AspRule>& goalRules,
      const AnswerSet& plan) const throw();


  AnswerSet currentStateQuery(const std::vector<actasp::AspRule>& query) const throw();

  void setCurrentState(const std::set<actasp::AspFluent>& newState);
  
  std::list<actasp::AnswerSet> filteringQuery(const AnswerSet& currentState, const AnswerSet& plan,const std::vector<actasp::AspRule>& goals);

  std::list<actasp::AnswerSet> genericQuery(const std::vector<actasp::AspRule>& query,
      unsigned int timestep,
      const std::string& fileName,
      unsigned int answerSetsNumber) const throw();

  std::list< std::list<AspAtom> > genericQuery(const std::string& query,
      unsigned int timestep,
      const std::string& fileName,
      unsigned int answerSetsNumber) const throw();

private:

  std::list<actasp::AnswerSet> genericQuery(const std::string& query,
      unsigned int initialTimeStep,
      unsigned int finalTimeStep,
      const std::string& fileName,
      unsigned int answerSetsNumber,
      bool useCurrentState) const throw();

  std::string makeQuery(const std::string& query,
                                 unsigned int initialTimeStep,
                                 unsigned int finalTimeStep,
                                 const std::string& fileName,
                              unsigned int answerSetsNumber,
                              bool useCurrentState
                       ) const throw();

  std::string generatePlanQuery(std::vector<actasp::AspRule> goalRules) const throw();
  
  std::string generateMonitorQuery(const std::vector<actasp::AspRule>& goalRules, const AnswerSet& plan) const throw();

  std::string incrementalVar;
  ActionSet allActions;
  unsigned int max_time;
  std::string queryDir;
  std::string domainDir;
  std::string currentFilePath;

};

}

#endif
