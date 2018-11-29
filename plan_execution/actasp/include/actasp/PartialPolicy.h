#pragma once


#include <actasp/AspFluent.h>

#include <set>
#include <stdexcept>

namespace actasp {
class AnswerSet;
  
struct PartialPolicy {
  
  virtual ActionSet actions(const std::set<AspFluent>& state) const noexcept = 0;
  
  virtual void merge(const AnswerSet& plan) = 0;
  virtual void merge(const PartialPolicy* otherPolicy) = 0;
  
  virtual bool empty() const noexcept = 0;
  
  virtual ~PartialPolicy() {}
    
};
  
}


