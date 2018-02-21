#include "Knock.h"

#include "ActionFactory.h"

#include "CallGUI.h"

#include "plan_execution/AspFluent.h"

#include <plan_execution/UpdateFluents.h>

#include <ros/ros.h>
#include <bwi_services/SpeakMessage.h>

#include <string>
#include <iostream>

using namespace std;

namespace bwi_krexec {

Knock::Knock() : 
            door(),
            done(false){
            }


void Knock::run() {

  ros::NodeHandle n;
  ros::ServiceClient speakClient = n.serviceClient<bwi_services::SpeakMessage> ( "speak_message" );
  speakClient.waitForExistence();

  ros::ServiceClient updateClient = n.serviceClient<plan_execution::UpdateFluents> ( "update_fluents" );
  updateClient.waitForExistence();

  std::stringstream ss;
  ss << "Can I come in?\n";

  bwi_services::SpeakMessage message_srv;
  message_srv.request.message = ss.str();
  speakClient.call(message_srv);

  vector<string> options;
  options.push_back("Yes");
  options.push_back("No");

  CallGUI knock("knock", CallGUI::CHOICE_QUESTION, ss.str(), 20.0, options);
  knock.run();

  plan_execution::UpdateFluents uf;
  plan_execution::AspFluent fluent;

  fluent.variables.push_back(door);

  fluent.name = knock.getResponseIndex() == 0 ? "accessgranted" : "-accessgranted";
  uf.request.fluents.push_back(fluent);
  updateClient.call(uf);

  CallGUI clear("clear", CallGUI::DISPLAY,  "");
  clear.run();

  done = true;
}

actasp::Action* Knock::cloneAndInit(const actasp::AspFluent& fluent) const {
  Knock *newAction = new Knock();
  newAction->door = fluent.getParameters().at(0);
  
  return newAction;
}

std::vector<std::string> Knock::getParameters() const {
  vector<string> param;
  param.push_back(door);
  return param;
}

ActionFactory KnockFactory(new Knock());
  
}