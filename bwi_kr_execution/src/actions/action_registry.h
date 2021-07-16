#ifndef BWI_KREXEC_ACTION_REGISTRY_H
#define BWI_KREXEC_ACTION_REGISTRY_H

#include "NavigateTo.h"
#include "OpenDoor.h"
#include "OpenSimulatedDoor.h"
#include "GoThrough.h"
#include "ChangeFloor.h"

namespace bwi_krexec {
    static std::map <std::string, actasp::ActionFactory> real_actions = {
            {"navigate_to", bwi_krexec::NavigateTo::create},
            {"open_door", bwi_krexec::OpenDoor::create},
            {"go_through", bwi_krexec::GoThrough::create},
            {"change_floor", bwi_krexec::ChangeFloor::create},
    };

static std::map <std::string, actasp::ActionFactory> simulated_actions = {
  {"navigate_to", bwi_krexec::NavigateTo::create},
  {"open_door", bwi_krexec::OpenSimulatedDoor::create},
  {"go_through", bwi_krexec::GoThrough::create},
  {"change_floor", bwi_krexec::ChangeFloor::create},
};
}

#endif
