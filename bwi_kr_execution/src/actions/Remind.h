#ifndef bwi_krexec_Remind_h__guard
#define bwi_krexec_Remind_h__guard

#include "actasp/Action.h"

#include <ros/ros.h>

#include <sound_play/SoundRequest.h>

#include <string>

namespace bwi_krexec {

class Remind : public actasp::Action{
public:
  Remind();

  int paramNumber() const {return 3;}
  
  std::string getName() const {return "remind";}
  
  void run();
  
  bool hasFinished() const {return done;}
  
  actasp::Action *cloneAndInit(const actasp::AspFluent & fluent) const;
  
  virtual actasp::Action *clone() const {return new Remind(*this);}
  
private:
  
 std::vector<std::string> getParameters() const;
 std::string person_to_remind;
 std::string meeting;
 std::string room;
 static ros::Publisher remind_pub;
 static bool pub_set;
 bool done;
 
};

}
 
#endif
 
