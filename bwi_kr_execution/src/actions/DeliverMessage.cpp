#include "DeliverMessage.h"

#include "ActionFactory.h"

#include "CallGUI.h"

#include "bwi_kr_execution/AspFluent.h"

#include <bwi_kr_execution/UpdateFluents.h>
#include <bwi_kr_execution/CurrentStateQuery.h>
#include <bwi_kr_execution/GetHriMessage.h>

#include <ros/ros.h>
#include <bwi_services/SpeakMessage.h>

#include <string>
#include <iostream>

using namespace std;

namespace bwi_krexec {

DeliverMessage::DeliverMessage() : 
            person(),
            message_id(),
            done(false){
            }

struct IsFluentAt {
  
  bool operator()(const bwi_kr_execution::AspFluent& fluent) {
    return fluent.name == "at";
  }
  
};

void DeliverMessage::run() {

  ros::NodeHandle n;
  ros::ServiceClient speakClient = n.serviceClient<bwi_services::SpeakMessage> ( "speak_message" );
  ros::ServiceClient updateClient = n.serviceClient<bwi_kr_execution::UpdateFluents> ("update_fluents");
  ros::ServiceClient krClient = n.serviceClient<bwi_kr_execution::CurrentStateQuery> ("current_state_query");
  ros::ServiceClient messageClient = n.serviceClient<bwi_kr_execution::GetHriMessage> ("look_up_message");

  speakClient.waitForExistence();
  updateClient.waitForExistence();
  krClient.waitForExistence();
  messageClient.waitForExistence();

  bwi_kr_execution::GetHriMessage ghm;
  ghm.request.message_id = message_id;
  messageClient.call(ghm);
  message = ghm.response.message;

  std::stringstream ss;
  
  ss << "Hi " << message.to << "! ";

  bwi_services::SpeakMessage message_srv;
  message_srv.request.message = ss.str();
  speakClient.call(message_srv);

  ss << "I have a message for you from " << message.from << ".\n";
  ss << "Do you want to read it now?\n";

  vector<string> options;
  options.push_back("Yes");
  options.push_back("No");

  CallGUI ask("ask", CallGUI::CHOICE_QUESTION, ss.str(), 20.0, options);
  ask.run();

  bwi_kr_execution::UpdateFluents uf;

  if (ask.getResponseIndex() == 0) {
    vector<string> options;
    options.push_back("Got it");
    CallGUI message_gui("message", CallGUI::CHOICE_QUESTION, message.content, 10.0, options);
    message_gui.run();

    bwi_kr_execution::AspFluent delivered;
    delivered.name = "messagedelivered";
    delivered.variables.push_back(person);
    delivered.variables.push_back(message_id);
    uf.request.fluents.push_back(delivered);
  }
  else if (ask.getResponseIndex() == 1) {
    string s = "OK. You can ask me again later.\n";
    CallGUI message("message", CallGUI::DISPLAY, s, 5.0);
    message.run();
    ros::Duration(3.0).sleep();

    bwi_kr_execution::AspFluent not_delivered;
    not_delivered.name = "-messagedelivered";
    not_delivered.variables.push_back(person);
    not_delivered.variables.push_back(message_id);
    uf.request.fluents.push_back(not_delivered);
  }
  else {
    bwi_kr_execution::CurrentStateQuery csq;
    krClient.call(csq);
    vector<bwi_kr_execution::AspFluent>::const_iterator atIt = 
                      find_if(csq.response.answer.fluents.begin(), csq.response.answer.fluents.end(), IsFluentAt());
                      
    if(atIt == csq.response.answer.fluents.end()) {
      ROS_ERROR("DeliverMessage: fluent \"at\" missing ");
    }
    else {
      bwi_kr_execution::AspFluent not_inroom;
      not_inroom.name = "-inside";
      not_inroom.variables.push_back(person);
      not_inroom.variables.push_back(atIt->variables[0]);
      uf.request.fluents.push_back(not_inroom);
    }
  }

  updateClient.call(uf);

  CallGUI clear("clear", CallGUI::DISPLAY,  "");
  clear.run();

  done = true;
}

actasp::Action* DeliverMessage::cloneAndInit(const actasp::AspFluent& fluent) const {
  DeliverMessage *newAction = new DeliverMessage();
  newAction->person = fluent.getParameters().at(0);
  newAction->message_id = fluent.getParameters().at(1);

  return newAction;
}

std::vector<std::string> DeliverMessage::getParameters() const {
  vector<string> param;
  param.push_back(person);
  param.push_back(message_id);
  return param;
}


ActionFactory DeliverMessageFactory(new DeliverMessage());
  
}