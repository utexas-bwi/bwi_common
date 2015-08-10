
#ifndef actasp_Clingo_h__guard
#define actasp_Clingo_h__guard

#include <actasp/AspKR.h>
#include <actasp/MultiPolicy.h>

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <stdexcept>
#include <set>

namespace actasp {

class Action;

class Clingo : public AspKR {
public:

	Clingo(unsigned int max_n,
         const std::string& incrementalVar,
	       const std::string& queryDir,
	       const std::string& domainDir,
	       const ActionSet& actions,
         unsigned int max_time = 0
        ) throw();

  ActionSet availableActions() const throw();
        
	AnswerSet currentStateQuery(const std::vector<actasp::AspRule>& query) const throw();
	
	bool updateFluents(const std::vector<actasp::AspFluent> &observations) throw();
  
  std::list< std::list<AspAtom> > query(const std::string &queryString, unsigned int initialTimeStep,
                                   unsigned int finalTimeStep) const throw();
	
	bool isPlanValid(const AnswerSet& plan, const std::vector<actasp::AspRule>& goal)  const throw();
  
  void reset() throw();

	AnswerSet computePlan(const std::vector<actasp::AspRule>& goal) const throw ();
	
	std::vector< AnswerSet> computeAllPlans(const std::vector<actasp::AspRule>& goal, double suboptimality) const throw ();
	
	MultiPolicy computePolicy(const std::vector<actasp::AspRule>& goal, double suboptimality) const throw (std::logic_error);
	
	void setMaxTimeStep(unsigned int max_n) throw() {
		this->max_n = max_n;
  }

private:

	unsigned int max_n;
  std::string incrementalVar;
  unsigned int max_time;
	std::string queryDir;
	std::string domainDir;
  ActionSet allActions;
  std::string actionFilter;

	std::string generatePlanQuery(std::vector<actasp::AspRule> goalRules,
									bool filterActions) const throw();

	std::list<actasp::AnswerSet> krQuery(	const std::string& query, 
											unsigned int initialTimeStep,
                      unsigned int finalTimeStep,
											const std::string& fileName, 
											unsigned int answerSetsNumber) const throw();
                      

};

}
#endif
