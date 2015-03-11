#include "RemoteReasoner.h"

#include "bwi_kr_execution/CurrentStateQuery.h"
#include "bwi_kr_execution/UpdateFluents.h"
#include "bwi_kr_execution/ComputePlan.h"
#include "bwi_kr_execution/ComputeAllPlans.h"
#include "bwi_kr_execution/IsPlanValid.h"

#include "msgs_utils.h"

#include "actasp/Action.h"

#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <algorithm>
#include <iterator>

using namespace std;
using namespace ros;

namespace bwi_krexec {
  
  RemoteReasoner::RemoteReasoner(unsigned int max_n,
         const std::string& queryDir,
         const std::string& domainDir,
         const actasp::ActionSet& actions,
         unsigned int max_time) : local(max_n,"n",queryDir,domainDir,actions,max_time) {}
  
actasp::AnswerSet RemoteReasoner::currentStateQuery(const std::vector<actasp::AspRule>& query) const throw() {
  return local.currentStateQuery(query);
}

actasp::ActionSet RemoteReasoner::availableActions() const throw() {
   return local.availableActions();
}

std::list< std::list<actasp::AspAtom> > RemoteReasoner::query(const std::string &queryString, unsigned int initialTimeStep,
                                   unsigned int finalTimeStep) const throw() {
  return local.query(queryString,initialTimeStep,finalTimeStep);
}

bool RemoteReasoner::updateFluents(const std::vector<actasp::AspFluent> &observations) throw() {
  NodeHandle n;
  ros::ServiceClient updateClient = n.serviceClient<bwi_kr_execution::UpdateFluents> ( "update_fluents" );
  updateClient.waitForExistence();
  
  bwi_kr_execution::UpdateFluents uf;
  
  transform(observations.begin(),observations.end(),back_inserter(uf.request.fluents),TranslateFluent());
  
  updateClient.call(uf);
  
  return uf.response.consistent;
}

actasp::AnswerSet RemoteReasoner::computePlan(const std::vector<actasp::AspRule>& goal) const throw (std::logic_error) {
  
  return local.computePlan(goal);
  
}


bool RemoteReasoner::isPlanValid(const actasp::AnswerSet& plan, const std::vector<actasp::AspRule>& goal)  const throw() {
  return local.isPlanValid(plan,goal);
  
}

std::vector< actasp::AnswerSet > RemoteReasoner::computeAllPlans(
                const std::vector<actasp::AspRule>& goal, 
                double suboptimality) const throw (std::logic_error) {
  
  return local.computeAllPlans(goal,suboptimality); 
  
                  
}

actasp::MultiPolicy RemoteReasoner::computePolicy(const std::vector<actasp::AspRule>& goal, 
                                  double suboptimality) const throw (std::logic_error) {
 
  return local.computePolicy(goal,suboptimality);
}

void RemoteReasoner::reset() throw() {
  NodeHandle n;
  ros::ServiceClient resetClient = n.serviceClient<std_srvs::Empty> ( "reset_state" );
  resetClient.waitForExistence();
  
  std_srvs::Empty empty;
  resetClient.call(empty);
}


}
