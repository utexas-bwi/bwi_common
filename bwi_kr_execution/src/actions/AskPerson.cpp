#include "AskPerson.h"

#include "ActionFactory.h"

#include "CallGUI.h"

#include "bwi_kr_execution/AspFluent.h"

#include <bwi_kr_execution/UpdateFluents.h>

#include <ros/ros.h>
#include <sound_play/sound_play.h>

#include <string>
#include <iostream>

using namespace std;

namespace bwi_krexec {

AskPerson::AskPerson() : 
            person_to_ask(),
            person_to_know(),
            done(false){
            }

ros::Publisher AskPerson::ask_pub;
bool AskPerson::pub_set(false);
  
void AskPerson::run() {

  ros::NodeHandle n;

  if (!pub_set) { 
    ask_pub = n.advertise<sound_play::SoundRequest>("robotsound", 1000);
    pub_set = true;
  }

  if (ask_pub.getNumSubscribers() == 0) return; //if the subscriber is not connected, sleep
  
  ros::ServiceClient krClient = n.serviceClient<bwi_kr_execution::UpdateFluents> ( "update_fluents" );
  krClient.waitForExistence();
  
  //speak
  sound_play::SoundRequest sound_req;
  sound_req.sound = sound_play::SoundRequest::SAY;
  sound_req.command = sound_play::SoundRequest::PLAY_ONCE;

  stringstream ss;
  ss << "Hi " << person_to_ask << "! ";
  ss << "Do you know where " << person_to_know << " is?";
  sound_req.arg = ss.str();
  ask_pub.publish(sound_req);

  vector<string> options;
  options.push_back("Yes! " + person_to_know + " is in GDC and I know exactly where.");
  options.push_back("No, I have no idea.");

  CallGUI askPerson("askPerson", CallGUI::CHOICE_QUESTION,  "Hi " + person_to_ask + "! Do you know where " + person_to_know + " is?", 60.0, options);
  askPerson.run();

  int response = askPerson.getResponseIndex();

  if (response < 0) {    
    bwi_kr_execution::UpdateFluents uf;
    bwi_kr_execution::AspFluent fluent;
    
    fluent.timeStep = 0;
    fluent.variables.push_back(person_to_ask);
    fluent.variables.push_back(person_to_know);

    fluent.name = "-know";

    uf.request.fluents.push_back(fluent);
    krClient.call(uf);
  }

  if (response == 1) {
    sound_req.arg = "OK, thank you anyway!";
    ask_pub.publish(sound_req);
    CallGUI thank("thank", CallGUI::DISPLAY,  "OK, thank you anyway!");
    thank.run();

    bwi_kr_execution::UpdateFluents uf;
    bwi_kr_execution::AspFluent fluent;
    
    fluent.timeStep = 0;
    fluent.variables.push_back(person_to_ask);
    fluent.variables.push_back(person_to_know);

    fluent.name = "-know";

    uf.request.fluents.push_back(fluent);
    krClient.call(uf);
  }

  if (response == 0) {

      bool know = false;
      int count = 0;

      sound_req.arg = "Great! Please tell me the room number.";
      CallGUI *askPerson = new CallGUI("askPerson", CallGUI::TEXT_QUESTION,  "Great! Please tell me the room number.", 60.0);
      
      while ((! know) && (count < 3)) {

        ask_pub.publish(sound_req);
        askPerson->run();

        if ((askPerson->getResponseIndex() == -3) && (askPerson->getResponse() != "")) {
              bwi_kr_execution::UpdateFluents uf;
              bwi_kr_execution::AspFluent fluent;
              
              fluent.timeStep = 0;
              fluent.variables.push_back(person_to_know);
              fluent.variables.push_back(askPerson->getResponse());

              fluent.name = "inroom";

              uf.request.fluents.push_back(fluent);

              krClient.call(uf);
              know = uf.response.consistent;
        }

        count++;

        sound_req.arg = "The room number doesn't exist. Please try again.";
        delete askPerson;
        askPerson = new CallGUI("askPerson", CallGUI::TEXT_QUESTION,  "The room number doesn't exist. Please try again.", 60.0);
      }

      sound_req.arg = "Thank you!";
      CallGUI *thank = new CallGUI("thank", CallGUI::DISPLAY,  "Thank you!");

      if (!know) {
        sound_req.arg = "OK, thank you anyway!";
        delete thank;
        thank = new CallGUI("thank", CallGUI::DISPLAY,  "OK, thank you anyway!");
      }

      //ask_pub.publish(sound_req);
      thank->run();

      bwi_kr_execution::UpdateFluents uf;
      bwi_kr_execution::AspFluent fluent;
      
      fluent.timeStep = 0;
      fluent.variables.push_back(person_to_ask);
      fluent.variables.push_back(person_to_know);

      fluent.name = (know ? "know" : "-know");

      uf.request.fluents.push_back(fluent);
      krClient.call(uf);

  }

  done = true;

}  
  
actasp::Action* AskPerson::cloneAndInit(const actasp::AspFluent& fluent) const {
  AskPerson *newAction = new AskPerson();
  newAction->person_to_ask = fluent.getParameters().at(0);
  newAction->person_to_know = fluent.getParameters().at(1);
  
  return newAction;
}

std::vector<std::string> AskPerson::getParameters() const {
  vector<string> param;
  param.push_back(person_to_ask);
  param.push_back(person_to_know);
  return param;
}


ActionFactory AskPersonFactory(new AskPerson());
  
}
