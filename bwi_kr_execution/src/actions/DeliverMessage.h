#ifndef bwi_krexec_DeliverMessage_h__guard
#define bwi_krexec_DeliverMessage_h__guard

#include "actasp/Action.h"

#include <ros/ros.h>

#include <sound_play/SoundRequest.h>
#include <bwi_kr_execution/HriMessage.h>

#include <string>

namespace bwi_krexec {

class DeliverMessage : public actasp::Action{
public:
  DeliverMessage();

  int paramNumber() const {return 2;}
  
  std::string getName() const {return "delivermessage";}
  
  void run();
  
  bool hasFinished() const {return done;}
  
  actasp::Action *cloneAndInit(const actasp::AspFluent & fluent) const;
  
  virtual actasp::Action *clone() const {return new DeliverMessage(*this);}
  
private:
  
 std::vector<std::string> getParameters() const;
 std::string person;
 std::string message_id;
 bwi_kr_execution::HriMessage message;
 
 bool done;
 
};

}
 
#endif
 
