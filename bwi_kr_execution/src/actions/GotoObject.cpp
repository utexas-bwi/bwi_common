#include "GotoObject.h"

#include "ActionFactory.h"
#include "plan_execution/msgs_utils.h"

#include "actasp/AnswerSet.h"

#include "plan_execution/CurrentStateQuery.h"

#include <ros/ros.h>

#include <algorithm>


using namespace std;
using namespace actasp;

namespace bwi_krexec {


GotoObject::GotoObject() :
        LogicalNavigation("goto"),
        failed(false) {}


void GotoObject::run() {

    LogicalNavigation::run();

    ros::NodeHandle n;
    ros::ServiceClient krClient = n.serviceClient<plan_execution::CurrentStateQuery>("current_state_query");
    krClient.waitForExistence();

    plan_execution::CurrentStateQuery csq;

    krClient.call(csq);

    AnswerSet answer = plan_exec::TranslateAnswerSet()(csq.response.answer);

    failed = !answer.contains(AspFluent("facing", this->getParameters(), 0));


}

std::vector<std::string> GotoObject::getParameters() const {
    std::vector<std::string> parameters;
    parameters.push_back(to_string(location_id));
    return parameters;
}

std::vector<std::string> GotoObject::prepareGoalParameters() const {
    auto attrs = ltmc.get_entity_attributes(location_id, "map_name");
    // FIXME: Fail more gracefully here
    assert(attrs.size() == 1);
    vector<string> parameters;
    parameters.push_back(attrs.at(0).get_string_value());
    return parameters;
}

ActionFactory gotoFactory(new GotoObject());


}