
#include <actasp/AnswerSet.h>
#include <actasp/Action.h>

#include <iostream>
#include <vector>
#include <sstream>
#include <algorithm>
#include <iterator>

using namespace std;
using namespace actasp;


//// Code copied from Clingo.cpp

class SimpleAction : public actasp::Action {
public:

	SimpleAction() : done(false) {}

	int paramNumber() const {
		return 0;
	}

	std::string getName() const {
		return name;
	}

	void run() {
		std::cout << "running " << name << std::endl;
		done = true;
	}

	bool hasFinished() const {
		return done;
	}

	virtual actasp::Action *cloneAndInit(const actasp::AspFluent &f) const {
		SimpleAction *newAction = new SimpleAction(*this);
		newAction->name = f.getName();
		newAction->params = f.getParameters();
		return newAction;
	}
	
	virtual actasp::Action *clone() const {
		return new SimpleAction(*this);
	}

private:
	std::string name;
	std::vector<std::string> params;
	bool done;

	std::vector<std::string> getParameters() const {
		return params;
	}

};

struct DeleteAction {
	
	void operator()(Action *act) {
		delete act;
	}
};

struct CreateFluent {
	AspFluent operator()(const std::string & fluentDescription) {
		return AspFluent(fluentDescription);
	}
};


static std::set<actasp::AspFluent> parseAnswerSet(const std::string& answerSetContent) {

  stringstream predicateLine(answerSetContent);

  set<AspFluent> predicates;

  //split the line based on spaces
  transform(istream_iterator<string>(predicateLine),
            istream_iterator<string>(),
            inserter(predicates,predicates.begin()),
            CreateFluent());

  return predicates;

}

static std::vector<actasp::AnswerSet> readAnswerSets(istream &input) {
	
	string line;
	string answerSetContent;
	while (input) {
		getline(input, line);
		answerSetContent += line;
		answerSetContent += "\n";
	}

	bool satisfiable = answerSetContent.find("UNSATISFIABLE") == string::npos;

	vector<AnswerSet> allSets;

	if (satisfiable) {

		stringstream content(answerSetContent);

		string firstLine;
		string eachAnswerset;

		while (content) {

			getline(content,firstLine);
			if (firstLine.find("Answer") != string::npos) {
				getline(content,eachAnswerset);

				set<AspFluent> fluents = parseAnswerSet(eachAnswerset);

				allSets.push_back(AnswerSet(fluents.begin(), fluents.end()));
			}
		}
	}

	return allSets;
	
}

int main() {
	
	std::map<std::string, actasp::Action *> actionMap;

	actionMap.insert(std::make_pair(std::string("approach"), new SimpleAction()));
	actionMap.insert(std::make_pair(std::string("gothrough"), new SimpleAction()));
	actionMap.insert(std::make_pair(std::string("goto"), new SimpleAction()));
	actionMap.insert(std::make_pair(std::string("opendoor"), new SimpleAction()));
	actionMap.insert(std::make_pair(std::string("searchroom"), new SimpleAction()));
	actionMap.insert(std::make_pair(std::string("askperson"), new SimpleAction()));	
	actionMap.insert(std::make_pair(std::string("changefloor"), new SimpleAction()));
    actionMap.insert(std::make_pair(std::string("callelevator"), new SimpleAction()));
    actionMap.insert(std::make_pair(std::string("remind"), new SimpleAction()));
	
	vector<AnswerSet> sets = readAnswerSets(cin);
	
	cout << endl << "Number of answer sets: " << sets.size() << endl;
	
	for(int i=0; i<sets.size(); ++i) {
		
		cout <<  "------------------------------" << endl;
		
		cout << "Plan" << endl;
		
		list<Action*> plan = sets[i].instantiateActions(actionMap);
		list<Action*>::iterator pIt = plan.begin();
		
		for(int t=1; pIt != plan.end(); ++pIt, ++t)
			if ((*pIt) != NULL)
				cout << (*pIt)->toASP(t) << " ";
		
		for_each(plan.begin(),plan.end(),DeleteAction());
		
		cout << endl << endl << "Fluents" << endl;
		
		
		//fluents in an answer set are ordered by time step.

		unsigned int lastTimeStep = sets[i].maxTimeStep();
		
		for(int t = 0; t <= lastTimeStep; ++t) {
			
		set<AspFluent> fluentsAtTimeT = sets[i].getFluentsAtTime(t);
		set<AspFluent>::const_iterator flu = fluentsAtTimeT.begin();

		
		
		for(; flu != fluentsAtTimeT.end(); ++flu)
			cout << flu->toString() << " ";
		
		cout << endl << endl;
		}
		
		cout << endl;
		
	}
	
	
	
	return 0;
}