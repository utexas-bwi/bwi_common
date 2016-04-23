#ifndef bwi_krexec_ChangeFloor_h__guard
#define bwi_krexec_ChangeFloor_h__guard

#include <boost/shared_ptr.hpp>

#include "actasp/Action.h"
#include "CallGUI.h"

namespace bwi_krexec {
  
struct ChangeFloor : public actasp::Action {

  ChangeFloor();

  int paramNumber() const {return 1;}
  
  std::string getName() const{return "changefloor";}
  
  void run();
  
  bool hasFinished() const;
  
  bool hasFailed() const;
  
  actasp::Action *cloneAndInit(const actasp::AspFluent & fluent) const;
  
  actasp::Action *clone() const {return new ChangeFloor(*this);}

private:
 
std::vector<std::string> getParameters() const;

std::string dest_room;

bool asked;
bool done;
bool failed;

boost::shared_ptr<CallGUI> askToChangeFloor;

};
  
  
}

#endif
