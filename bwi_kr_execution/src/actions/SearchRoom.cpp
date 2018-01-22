#include "SearchRoom.h"

#include "ActionFactory.h"

#include "CallGUI.h"

#include "bwi_kr_execution/AspFluent.h"

#include "bwi_kr_execution/CurrentStateQuery.h"
#include <bwi_kr_execution/UpdateFluents.h>
#include <bwi_services/SpeakMessage.h>

#include <ros/ros.h>
#include <sound_play/sound_play.h>

#include <string>
#include <iostream>

using namespace std;

namespace bwi_krexec {

SearchRoom::SearchRoom() : 
            person(),
            room(),
            done(false),
            failed(false){
            }

  
void SearchRoom::run() {

  person[0] = toupper(person[0]);

  ros::NodeHandle n;

  //current state query
  ros::ServiceClient currentClient = n.serviceClient<bwi_kr_execution::CurrentStateQuery> ( "current_state_query" );
  currentClient.waitForExistence();
  ros::ServiceClient speakClient = n.serviceClient<bwi_services::SpeakMessage> ( "speak_message" );
  speakClient.waitForExistence();

  bwi_kr_execution::AspFluent atFluent;
  atFluent.name = "at";
  atFluent.timeStep = 0;
  atFluent.variables.push_back(room);
  
  bwi_kr_execution::AspRule rule;
  rule.head.push_back(atFluent);
  
  bwi_kr_execution::CurrentStateQuery csq;
  csq.request.query.push_back(rule);
  
  currentClient.call(csq);
  
  bool at = csq.response.answer.satisfied;

  if (at) {

    std::stringstream ss;
    ss << "Is " << person << " in the room?";

    bwi_services::SpeakMessage message_srv;
    message_srv.request.message = ss.str();
    speakClient.call(message_srv);
  }

  vector<string> options;
  options.push_back("yes");
  options.push_back("no");

  CallGUI searchRoom("searchRoom", CallGUI::CHOICE_QUESTION,  "Is " + person + " in the room " + room + "?", 60.0, options);
  searchRoom.run();

  int response = searchRoom.getResponseIndex();

  if (response >= 0) {
    CallGUI thank("thank", CallGUI::DISPLAY,  "Thank you!");
    thank.run();
  }
  

  ros::ServiceClient krClient = n.serviceClient<bwi_kr_execution::UpdateFluents> ( "update_fluents" );
  krClient.waitForExistence();

  bwi_kr_execution::UpdateFluents uf;
  bwi_kr_execution::AspFluent fluent;
  fluent.timeStep = 0;
  person[0] = tolower(person[0]);
  fluent.variables.push_back(person);
  fluent.variables.push_back(room);

  fluent.name = ((response == 0) ? "inroom" : "-inroom");

  uf.request.fluents.push_back(fluent);
  krClient.call(uf);

  done = true;

}  
  
actasp::Action* SearchRoom::cloneAndInit(const actasp::AspFluent& fluent) const {
  SearchRoom *newAction = new SearchRoom();
  newAction->person = fluent.getParameters().at(0);
  newAction->room = fluent.getParameters().at(1);
  
  return newAction;
}

std::vector<std::string> SearchRoom::getParameters() const {
  vector<string> param;
  param.push_back(person);
  param.push_back(room);
  return param;
}


ActionFactory SearchRoomFactory(new SearchRoom());
  
}
