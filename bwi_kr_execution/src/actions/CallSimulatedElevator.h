#ifndef bwi_krexec_CallSimulatedElevator_h__guard
#define bwi_krexec_CallSimulatedElevator_h__guard

#include <boost/shared_ptr.hpp>

#include "actasp/Action.h"

namespace bwi_krexec {

struct CallSimulatedElevator : public actasp::Action {

  CallSimulatedElevator();

  int paramNumber() const {return 2;}

  std::string getName() const{return "callelevator";}

  void run();

  bool hasFinished() const;

  bool hasFailed() const;

  actasp::Action *cloneAndInit(const actasp::AspFluent & fluent) const;

  actasp::Action *clone() const {return new CallSimulatedElevator(*this);}

private:

 std::vector<std::string> getParameters() const;

 std::string elevator;
 bool going_up;

 bool done;
 bool failed;
 bool requestSent;
 std::string selectedDoor;

 std::vector<std::string> doors;

};


}

#endif
