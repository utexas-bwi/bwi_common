#include "NavigateTo.h"

#include "ActionFactory.h"
#include "plan_execution/msgs_utils.h"

#include "actasp/AnswerSet.h"
#include <knowledge_representation/Entity.h>

#include "plan_execution/CurrentStateQuery.h"

#include <ros/ros.h>

#include <algorithm>

using namespace std;
using namespace actasp;

namespace bwi_krexec {


    NavigateTo::NavigateTo():
            LogicalNavigation("navigate_to"),
            location_id(-1) {
    }


    void NavigateTo::run()  {
        bwi_krexec::LogicalNavigation::run();
    }

    std::vector<std::string> NavigateTo::getParameters() const {
        std::vector<std::string> parameters;
        parameters.push_back(to_string(location_id));
        return parameters;
    }

    std::vector<std::string> NavigateTo::prepareGoalParameters() const {
        knowledge_rep::Entity location(location_id, ltmc);
        // FIXME: Fail more gracefully here
        assert(location.is_valid());
        auto attrs = location.get_attributes("name");
        // FIXME: Fail more gracefully here
        assert(attrs.size() == 1);
        vector<string> parameters;
        parameters.push_back(attrs.at(0).get_string_value());
        return parameters;
    }

    ActionFactory navigateToFactory(new NavigateTo());


}
