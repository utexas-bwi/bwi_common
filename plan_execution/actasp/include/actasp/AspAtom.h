#ifndef actasp_AspAtom_h__guard
#define actasp_AspAtom_h__guard

#include <string>
#include <stdexcept>
#include <vector>

namespace actasp {

  
class AspAtom {
public:

  AspAtom(const std::string& formula) throw ();

  unsigned int arity() const noexcept;

  std::string getName() const noexcept;
  
  virtual std::vector<std::string> getParameters() const noexcept;
  
  virtual bool operator<(const AspAtom& other) const noexcept {return atom < other.atom;}
  virtual bool operator==(const AspAtom& other) const noexcept{return atom == other.atom;};

  virtual std::string toString() const noexcept {return atom;}
  
  virtual operator std::string() const { return this->toString(); } 

  virtual ~AspAtom() {}
  
private:
  std::string atom; //cached for optimization
};


}
#endif
