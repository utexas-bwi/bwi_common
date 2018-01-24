
#ifndef bwi_krexec_SearchPerson_h__guard
#define bwi_krexec_SearchPerson_h__guard

#include "actasp/Action.h"

#include <ros/ros.h>

#include <sound_play/SoundRequest.h>

#include <string>

namespace bwi_krexec {

class SearchPerson : public actasp::Action{
public:
  SearchPerson();

  int paramNumber() const {return 2;}
  
  std::string getName() const {return "searchperson";}
  
  void run();
  
  bool hasFinished() const {return done;}
  
  bool hasFailed() const {return failed;}
  
  actasp::Action *cloneAndInit(const actasp::AspFluent & fluent) const;
  
  virtual actasp::Action *clone() const {return new SearchPerson(*this);}
  
private:
  
 std::vector<std::string> getParameters() const;
 std::string person;
 std::string room;
 bool done,failed;
 
};

}
 
#endif
 
