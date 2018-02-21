#include "AskPerson.h"

#include "ActionFactory.h"

#include "CallGUI.h"

#include "plan_execution/AspFluent.h"

#include <plan_execution/UpdateFluents.h>

#include <ros/ros.h>

#include <string>
#include <iostream>

using namespace std;

namespace bwi_krexec {

AskPerson::AskPerson() : 
            person_to_ask(),
            person_to_know(),
            done(false){
            }
  
void AskPerson::run() {

  ros::NodeHandle n;
  
  ros::ServiceClient krClient = n.serviceClient<plan_execution::UpdateFluents> ( "update_fluents" );
  krClient.waitForExistence();

  stringstream ss;
  ss << "Hi " << person_to_ask << "! ";
  ss << "Do you know where " << person_to_know << " is?";

  vector<string> options;
  options.push_back("Yes! In GDC");
  options.push_back("Yes! NOT in GDC");
  options.push_back("No, I have no idea");

  CallGUI askPerson("askPerson", CallGUI::CHOICE_QUESTION,  "Hi " + person_to_ask + "! Do you know where " + person_to_know + " is?", 60.0, options);
  askPerson.run();

  int response = askPerson.getResponseIndex();

  if (response < 0) {    
    plan_execution::UpdateFluents uf;
    plan_execution::AspFluent fluent;
    
    fluent.timeStep = 0;
    fluent.variables.push_back(person_to_ask);
    fluent.variables.push_back(person_to_know);

    fluent.name = "-know";

    uf.request.fluents.push_back(fluent);
    krClient.call(uf);
  }

  if (response == 2) {
    CallGUI thank("thank", CallGUI::DISPLAY,  "OK, thank you anyway!");
    thank.run();

    plan_execution::UpdateFluents uf;
    plan_execution::AspFluent fluent;
    
    fluent.timeStep = 0;
    fluent.variables.push_back(person_to_ask);
    fluent.variables.push_back(person_to_know);

    fluent.name = "-know";

    uf.request.fluents.push_back(fluent);
    krClient.call(uf);
  }

    if (response == 1) {
    CallGUI thank("thank", CallGUI::DISPLAY,  "OK, thank you!");
    thank.run();

    plan_execution::UpdateFluents uf;
    plan_execution::AspFluent fluent;
    
    fluent.name = "-ingdc";
    
    fluent.variables.push_back(person_to_know);

    uf.request.fluents.push_back(fluent);
    krClient.call(uf);
  }
  
  
  if (response == 0) {

      bool know = false;
      int count = 0;
      
      stringstream question_text;
      question_text << "Great! Please tell me the room number." << endl;
      question_text << "Examples of rooms are:" << endl; 
      question_text << "Peter's office: l3_508" << endl;
      question_text << "Matteo's office: l3_418" << endl;
      question_text << "Shiqi's office: l3_420" << endl;
      question_text << "Jivko's office: l3_432" << endl;
      question_text << "Lab: l3_414b" << endl;
      
      CallGUI *askPerson = new CallGUI("askPerson", CallGUI::TEXT_QUESTION,  question_text.str(), 60.0);
      
      while ((! know) && (count < 3)) {

        askPerson->run();

        if ((askPerson->getResponseIndex() == -3) && (askPerson->getResponse() != "")) {
              plan_execution::UpdateFluents uf;
              plan_execution::AspFluent fluent;
              
              fluent.timeStep = 0;
              fluent.variables.push_back(person_to_know);
              fluent.variables.push_back(askPerson->getResponse());

              fluent.name = "inroom";

              uf.request.fluents.push_back(fluent);

              krClient.call(uf);
              know = uf.response.consistent;
        }

        count++;

        delete askPerson;
        askPerson = new CallGUI("askPerson", CallGUI::TEXT_QUESTION,  "The room number doesn't exist. Please try again.", 60.0);
      }

      CallGUI *thank = new CallGUI("thank", CallGUI::DISPLAY,  "Thank you!");

      if (!know) {
        delete thank;
        thank = new CallGUI("thank", CallGUI::DISPLAY,  "OK, thank you anyway!");
      }

      thank->run();

      plan_execution::UpdateFluents uf;
      plan_execution::AspFluent fluent;
      
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
