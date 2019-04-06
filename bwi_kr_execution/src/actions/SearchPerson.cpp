#include "SearchPerson.h"


#include "CallGUI.h"

#include "plan_execution/AspFluent.h"

#include "plan_execution/CurrentStateQuery.h"
#include <plan_execution/UpdateFluents.h>
#include <bwi_services/SpeakMessage.h>

#include <ros/ros.h>

#include <string>
#include <iostream>

using namespace std;

namespace bwi_krexec {

SearchPerson::SearchPerson() : 
            person(),
            room(),
            object(),
            done(false),
            failed(false){
            }

  
void SearchPerson::run() {

  person[0] = toupper(person[0]);

  ros::NodeHandle n;

  //current state query
  ros::ServiceClient currentClient = n.serviceClient<plan_execution::CurrentStateQuery> ( "current_state_query" );
  currentClient.waitForExistence();
  ros::ServiceClient speakClient = n.serviceClient<bwi_services::SpeakMessage> ( "speak_message" );
  speakClient.waitForExistence();

  plan_execution::AspFluent atFluent;
  atFluent.name = "at";
  atFluent.timeStep = 0;
  atFluent.variables.push_back(room);
  
  plan_execution::AspRule rule;
  rule.head.push_back(atFluent);
  
  plan_execution::CurrentStateQuery csq;
  csq.request.query.push_back(rule);
  
  currentClient.call(csq);
  
  bool at = csq.response.answer.satisfied;

  if (at) {

    std::stringstream ss;
    ss << "Is " << person << " here?";

    bwi_services::SpeakMessage message_srv;
    message_srv.request.message = ss.str();
    speakClient.call(message_srv);
  }

  vector<string> options;
  options.push_back("yes");
  options.push_back("no");

  CallGUI SearchPerson("searchPerson", CallGUI::CHOICE_QUESTION,  "Is " + person + " here " + "?", 60.0, options);
  SearchPerson.run();

  int response = SearchPerson.getResponseIndex();

  if (response >= 0) {
    CallGUI thank("thank", CallGUI::DISPLAY,  "Thank you!");
    thank.run();
  }
  

  ros::ServiceClient krClient = n.serviceClient<plan_execution::UpdateFluents> ( "update_fluents" );
  krClient.waitForExistence();

  person[0] = tolower(person[0]);

  plan_execution::UpdateFluents uf;

  plan_execution::AspFluent inside;
  inside.timeStep = 0;
  inside.variables.push_back(person);
  inside.variables.push_back(room);
  inside.name = ((response == 0) ? "inside" : "-inside");
  uf.request.fluents.push_back(inside);

  if (object[0] == 'o') {
    plan_execution::AspFluent near;
    near.timeStep = 0;
    near.variables.push_back(person);
    near.variables.push_back(object);
    near.name = ((response == 0) ? "near" : "-near");
    uf.request.fluents.push_back(near);
  }
  
  krClient.call(uf);

  done = true;

}  
  
actasp::Action* SearchPerson::cloneAndInit(const actasp::AspFluent& fluent) const {
  SearchPerson *newAction = new SearchPerson();
  newAction->person = fluent.getParameters().at(0);
  newAction->room = fluent.getParameters().at(1);
  newAction->object = fluent.getParameters().at(2);
  
  return newAction;
}

std::vector<std::string> SearchPerson::getParameters() const {
  vector<string> param;
  param.push_back(person);
  param.push_back(room);
  param.push_back(object);
  return param;
}

}
