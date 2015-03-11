
#ifndef bwi_krexec_SearchRoom_h__guard
#define bwi_krexec_SearchRoom_h__guard

#include "actasp/Action.h"

#include <ros/ros.h>

#include <sound_play/SoundRequest.h>

#include <string>

namespace bwi_krexec {

class SearchRoom : public actasp::Action{
public:
  SearchRoom();

  int paramNumber() const {return 2;}
  
  std::string getName() const {return "searchroom";}
  
  void run();
  
  bool hasFinished() const {return done;}
  
  actasp::Action *cloneAndInit(const actasp::AspFluent & fluent) const;
  
  virtual actasp::Action *clone() const {return new SearchRoom(*this);}
  
private:
  
 std::vector<std::string> getParameters() const;
 std::string person;
 std::string room;
 static ros::Publisher ask_pub;
 static bool pub_set;
 bool done;
 
};

}
 
#endif
 
