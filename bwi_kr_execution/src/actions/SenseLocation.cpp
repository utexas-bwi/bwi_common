#include "SenseLocation.h"

#include "ActionFactory.h"
#include "plan_execution/msgs_utils.h"

#include "actasp/AnswerSet.h"

#include "plan_execution/CurrentStateQuery.h"



#include <ros/ros.h>

#include <algorithm>

using namespace std;
using namespace actasp;

namespace bwi_krexec {


    SenseLocation::SenseLocation():
            LogicalNavigation("sense_location"){
    }

    std::vector<std::string> SenseLocation::getParameters() const {
        return {};
    }

    std::vector<std::string> SenseLocation::prepareGoalParameters() const {
        return {};
    }

}
