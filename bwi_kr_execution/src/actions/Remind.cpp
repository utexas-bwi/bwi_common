#include "Remind.h"

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

Remind::Remind() : 
            person_to_remind(),
            meeting(),
            room(),
            done(false){
            }

ros::Publisher Remind::remind_pub;
bool Remind::pub_set(false);

void Remind::run() {

  ros::NodeHandle n;
  if (!pub_set) { 
    remind_pub = n.advertise<sound_play::SoundRequest>("robotsound", 1000);
    pub_set = true;
  }

  ros::ServiceClient krClient = n.serviceClient<bwi_kr_execution::UpdateFluents> ( "update_fluents" );
  krClient.waitForExistence();

  if (remind_pub.getNumSubscribers() == 0) return; //if the subscriber is not connected, sleep

  //speak
  sound_play::SoundRequest sound_req;
  sound_req.sound = sound_play::SoundRequest::SAY;
  sound_req.command = sound_play::SoundRequest::PLAY_ONCE;
  std::stringstream ss;
  
  ss << "Hi " << person_to_remind << "!\n";
  ss << "You have a meeting right now.\n";
  ss << "Please come to room " << room << " .\n";
  sound_req.arg = ss.str();

  remind_pub.publish(sound_req);

  ss.str("");
  ss << "Hi " << person_to_remind << "!\n";
  ss << "This is a friendly reminder that you have a meeting right now: \n";
  ss << "Meeting name: " << meeting << "\n";
  ss << "Meeting room: " << room;

  vector<string> options;
  options.push_back("Okay, I'll come soon.");

  CallGUI remindPerson("remindPerson", CallGUI::CHOICE_QUESTION, ss.str(), 60.0, options);
  remindPerson.run();

  bwi_kr_execution::UpdateFluents uf;
  bwi_kr_execution::AspFluent fluent;

  fluent.timeStep = 0;
  fluent.variables.push_back(person_to_remind);
  fluent.variables.push_back(meeting);

  fluent.name = "inmeeting";

  uf.request.fluents.push_back(fluent);
  krClient.call(uf);

  CallGUI thank("thank", CallGUI::DISPLAY,  "Thank you!");
  thank.run();

  done = true;
}

actasp::Action* Remind::cloneAndInit(const actasp::AspFluent& fluent) const {
  Remind *newAction = new Remind();
  newAction->person_to_remind = fluent.getParameters().at(0);
  newAction->meeting = fluent.getParameters().at(1);
  newAction->room = fluent.getParameters().at(2);
  
  return newAction;
}

std::vector<std::string> Remind::getParameters() const {
  vector<string> param;
  param.push_back(person_to_remind);
  param.push_back(meeting);
  param.push_back(room);
  return param;
}


ActionFactory RemindFactory(new Remind());
  
}