#include <actasp/AspAtom.h>

using namespace std;

namespace actasp {

AspAtom::AspAtom(const std::string& formula) :atom(formula) {}


unsigned int AspAtom::arity() const noexcept {
  return getParameters().size();
}

std::string AspAtom::getName() const noexcept {
  return atom.substr(0,atom.find_first_of('('));
}
  
std::vector<std::string> AspAtom::getParameters() const noexcept {
    
  size_t start = atom.find_first_of('(');
  size_t end = atom.find_last_of(')'); //should be the last character...
  
  if(start == string::npos || end == string::npos)
    return vector<string>();
  
  start++;
  
  vector<string> params;
  
  if(start >= end)
    return params;
  
  size_t comma = atom.find_first_of(',',start);
  size_t end_param = std::min(comma,end);
  
  while(start < end) {
    params.push_back(atom.substr(start,end_param-start));
    start = end_param+1;
    end_param = std::min(atom.find_first_of(',',start),end);
  }
  
  return params;
}


}
