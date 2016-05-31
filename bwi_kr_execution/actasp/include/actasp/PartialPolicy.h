#ifndef actasp_PartialPolicy_h__guard
#define actasp_PartialPolicy_h__guard


#include <actasp/AspFluent.h>

#include <set>
#include <stdexcept>

namespace actasp {
class AnswerSet;
  
struct PartialPolicy {
  
  virtual ActionSet actions(const std::set<AspFluent>& state) const throw() = 0;
  
  virtual void merge(const AnswerSet& plan) throw(std::logic_error) = 0;
  virtual void merge(const PartialPolicy* otherPolicy) = 0;
  
  virtual bool empty() const throw() = 0;
  
  virtual ~PartialPolicy() {}
    
};
  
}

#endif
