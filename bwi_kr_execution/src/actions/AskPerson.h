
#ifndef bwi_krexec_AskPerson_h__guard
#define bwi_krexec_AskPerson_h__guard

#include "actasp/Action.h"

#include <ros/ros.h>

#include <string>

namespace bwi_krexec {

class AskPerson : public actasp::Action{
public:
  AskPerson();

  int paramNumber() const {return 2;}
  
  std::string getName() const {return "askperson";}
  
  void run();
  
  bool hasFinished() const {return done;}
  
  actasp::Action *cloneAndInit(const actasp::AspFluent & fluent) const;
  
  virtual actasp::Action *clone() const {return new AskPerson(*this);}
  
private:
  
 std::vector<std::string> getParameters() const;
 std::string person_to_ask;
 std::string person_to_know;
 bool done;
 
};

}
 
#endif
 