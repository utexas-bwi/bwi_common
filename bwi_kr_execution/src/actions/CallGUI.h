
#ifndef bwi_krexec_CallGUI_h__guard
#define bwi_krexec_CallGUI_h__guard

#include <bwi_msgs/QuestionDialogRequest.h>
#include "bwi_msgs/QuestionDialog.h"

#include "actasp/Action.h"

#include <string>

namespace bwi_krexec {

class CallGUI : public actasp::Action {
public:

  enum TYPE {
    DISPLAY = bwi_msgs::QuestionDialogRequest::DISPLAY,
    CHOICE_QUESTION = bwi_msgs::QuestionDialogRequest::CHOICE_QUESTION,
    TEXT_QUESTION = bwi_msgs::QuestionDialogRequest::TEXT_QUESTION
  };

  CallGUI(const std::string &name, const TYPE type,  const std::string& message,
          float timeOut = 0.0f,
          const std::vector<std::string> &options = std::vector<std::string>());

  int paramNumber() const { return 0;}

  std::string getName() const { return name; }

  std::string getResponse();

  int getResponseIndex();

  void run();

  bool hasFinished() const { return done; }

  actasp::Action *cloneAndInit(const actasp::AspFluent & fluent) const;

  virtual actasp::Action *clone() const { return new CallGUI(*this); }

private:

  std::vector<std::string> getParameters() const {
    return std::vector<std::string>();
  }

  std::string name;
  TYPE type;
  std::string message;
  float timeOut;
  std::vector<std::string> options;
  bool done;

  bwi_msgs::QuestionDialog req;

};

}

#endif
