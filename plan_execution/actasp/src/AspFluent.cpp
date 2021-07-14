#include <actasp/AspFluent.h>

#include <sstream>
#include <cstdlib> //for atoi

using namespace std;

namespace actasp {

AspFluent operator ""_f(const char *string, std::size_t size) {
  return AspFluent(string);
}

AspFluent::AspFluent(const std::string& formula) noexcept(false) :
      timeStep(),
			cachedBase(){
        
  //this used to be nice, but it turned out to be a major bottleneck, so I had to reimplement it for efficiency.

   string current;
   //current.reserve(100);
   size_t first_par = formula.find_first_of('(');
   size_t last_par = formula.find_last_of(')');
   size_t last_comma = formula.find_last_of(',');
   
   if(first_par == string::npos)
    throw std::invalid_argument("AspFluent: The string " + formula + " does not contain a '(', therefore is not a valid fluent");
   
   if(last_par == string::npos)
     throw std::invalid_argument("The string " + formula + " does not contain a ')', therefore is not a valid fluent");
   
   size_t time_begins = (last_comma == string::npos)? first_par+1 : last_comma+1;
   timeStep = atoi(&formula[time_begins]);
   cachedBase.assign(&formula[0],time_begins);
  
}

AspFluent::AspFluent(const std::string &name, const std::vector<std::string> &variables, unsigned int timeStep) noexcept
    :
		timeStep(timeStep),
		cachedBase() {
  stringstream ss;

  ss << name << "(";

  int i=0;
  for (int size = variables.size(); i<size; ++i)
    ss << variables[i] << ",";

  cachedBase = ss.str();

}

unsigned int AspFluent::arity() const  noexcept {
	return this->getParameters().size() + 1;
}

void AspFluent::setTimeStep(unsigned int timeStep) noexcept {
	

	this->timeStep = timeStep;
	
}

unsigned int AspFluent::getTimeStep() const noexcept {
	return this->timeStep;
}

string AspFluent::getName() const noexcept {
	return cachedBase.substr(0,cachedBase.find_first_of('('));
}

vector<string> AspFluent::getParameters() const noexcept {
  
  size_t start = cachedBase.find_first_of('(')+1;
  
  vector<string> params;
  
  size_t comma = cachedBase.find_first_of(',',start);
  
  while(comma != string::npos) {
    params.push_back(cachedBase.substr(start,comma-start));
    start = comma+1;
    comma = cachedBase.find_first_of(',',comma+1);
  }
  
	return params;
}

bool AspFluent::operator<(const AspFluent& other) const noexcept{
	if(this->timeStep < other.timeStep)
		return true;

	if(this->timeStep > other.timeStep)
		return false;

	return  this->cachedBase < other.cachedBase;
}

bool AspFluent::operator==(const AspFluent& other) const noexcept {
	if(this->timeStep != other.timeStep)
		return false;
	
	return this->cachedBase == other.cachedBase;
}

std::string AspFluent::toString(unsigned int timeStep) const noexcept {
    
  stringstream ss;
  ss << timeStep << ")";
  return cachedBase + ss.str();
}

std::string AspFluent::toString(const string& timeStepVar) const noexcept {
  return cachedBase + timeStepVar + ")";
}

std::string AspFluent::toString() const noexcept {
	return this->toString(this->timeStep);
}


}