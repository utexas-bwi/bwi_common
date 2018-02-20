#ifndef bwi_krexec_CallElevator_h__guard
#define bwi_krexec_CallElevator_h__guard

#include <boost/shared_ptr.hpp>

#include "actasp/Action.h"
#include "CallGUI.h"

namespace bwi_krexec {

struct CallElevator : public actasp::Action {

  CallElevator();

  int paramNumber() const {return 2;}

  std::string getName() const{return "callelevator";}

  void run();

  bool hasFinished() const;

  bool hasFailed() const;

  actasp::Action *cloneAndInit(const actasp::AspFluent & fluent) const;

  actasp::Action *clone() const {return new CallElevator(*this);}

private:

std::vector<std::string> getParameters() const;

std::string elevator;
bool going_up;

bool asked;
bool done;
bool failed;
std::string facing_door;

int randLED;
int randSpeech;

boost::shared_ptr<CallGUI> askToCallElevator;

};


}

#endif
