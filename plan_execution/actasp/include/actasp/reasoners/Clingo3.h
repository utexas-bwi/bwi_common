
#ifndef actasp_Clingo3_h__guard
#define actasp_Clingo3_h__guard

#include <actasp/QueryGenerator.h>

namespace actasp {

struct Clingo3 : public QueryGenerator {

  Clingo3(const std::string& incrementalVar,
          const std::string& queryDir,
          const std::string& domainDir,
          const ActionSet& actions,
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

  std::list<actasp::AnswerSet> monitorQuery(const std::vector<actasp::AspRule>& goalRules,
      const AnswerSet& plan) const throw();


  AnswerSet currentStateQuery(const std::vector<actasp::AspRule>& query) const throw();

  void setCurrentState(const std::set<actasp::AspFluent>& newState);


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
      unsigned int answerSetsNumber) const throw();

  std::string makeQuery(const std::string& query,
                                 unsigned int initialTimeStep,
                                 unsigned int finalTimeStep,
                                 const std::string& fileName,
                              unsigned int answerSetsNumber) const throw();

  std::string generatePlanQuery(std::vector<actasp::AspRule> goalRules,
                                bool filterActions) const throw();

  std::string incrementalVar;
  std::string actionFilter;
  unsigned int max_time;
  std::string queryDir;
  std::string domainDir;


};

}

#endif
