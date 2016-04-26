#ifndef bwi_krexec_ChangeFloor_h__guard
#define bwi_krexec_ChangeFloor_h__guard

#include <boost/shared_ptr.hpp>

#include "actasp/Action.h"
#include "CallGUI.h"

namespace bwi_krexec {
  
struct SimulatedChangeFloor : public actasp::Action {

  public:

    SimulatedChangeFloor();

    int paramNumber() const {return 1;}
    
    std::string getName() const{return "changefloor";}
    
    void run();
    
    bool hasFinished() const;
    
    bool hasFailed() const;
    
    actasp::Action *cloneAndInit(const actasp::AspFluent & fluent) const;
    
    actasp::Action *clone() const {return new SimulatedChangeFloor(*this);}

  private:

    std::vector<std::string> getParameters() const;

    std::string dest_room;

    bool robot_teleported;
    ros::Time robot_teleportation_start_time;

    bool done;
    bool failed;

    boost::shared_ptr<CallGUI> askToChangeFloor;

};
  
  
}

#endif
