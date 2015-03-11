#ifndef actasp_AspAtom_h__guard
#define actasp_AspAtom_h__guard

#include <string>
#include <stdexcept>
#include <vector>

namespace actasp {

  
class AspAtom {
public:

  AspAtom(const std::string& formula) throw ();

  unsigned int arity() const throw();

  std::string getName() const throw();
  
  virtual std::vector<std::string> getParameters() const throw();
  
  virtual bool operator<(const AspAtom& other) const throw() {return atom < other.atom;}
  virtual bool operator==(const AspAtom& other) const throw(){return atom == other.atom;};

  virtual std::string toString() const throw() {return atom;}
  
  virtual operator std::string() const { return this->toString(); } 

  virtual ~AspAtom() {}
  
private:
  std::string atom; //cached for optimization
};


}
#endif
