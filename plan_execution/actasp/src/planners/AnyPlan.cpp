#include <actasp/planners/AnyPlan.h>

#include <actasp/MultiPlanner.h>
#include <actasp/Action.h>

#include <cstdlib>

using namespace std;

namespace actasp {

AnyPlan::AnyPlan(actasp::MultiPlanner *actualPlanner, double suboptimality) : 
        actualPlanner(actualPlanner),
        suboptimality(suboptimality){}

Plan AnyPlan::computePlan(const std::vector<actasp::AspRule> &goal) const {

	vector<Plan> allPlans = actualPlanner->computeAllPlans(goal, suboptimality);

	if (allPlans.empty())
		return Plan();

	//pick one plan and return it, destroy the others

	int picked =rand() % allPlans.size();

	return allPlans[picked];

}

}
