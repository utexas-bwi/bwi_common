#ifndef bwi_krexec_Knock_h__guard
#define bwi_krexec_Knock_h__guard

#include "actasp/Action.h"

#include <ros/ros.h>

#include <string>

namespace bwi_krexec {

class Knock : public actasp::Action{
public:
  Knock();

  int paramNumber() const {return 1;}
  
  std::string getName() const {return "knock";}
  
  void run();
  
  bool hasFinished() const {return done;}
  
  actasp::Action *cloneAndInit(const actasp::AspFluent & fluent) const;
  
  virtual actasp::Action *clone() const {return new Knock(*this);}
  
private:
  
 std::vector<std::string> getParameters() const;
 std::string door;
 bool done;
 
};

}
 
#endif
 
