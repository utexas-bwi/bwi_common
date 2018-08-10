#include "SenseLocation.h"

#include "plan_execution/msgs_utils.h"

#include "actasp/AnswerSet.h"

#include "plan_execution/CurrentStateQuery.h"



#include <ros/ros.h>

#include <algorithm>

using namespace std;
using namespace actasp;

namespace bwi_krexec {


    SenseLocation::SenseLocation(knowledge_rep::LongTermMemoryConduit &ltmc):
            LogicalNavigation("sense_location", ltmc){
    }

    std::vector<std::string> SenseLocation::getParameters() const {
        return {};
    }

    std::vector<std::string> SenseLocation::prepareGoalParameters() const {
        return {};
    }

}
