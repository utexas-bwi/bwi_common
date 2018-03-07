#ifndef actasp_FilteringKR_h__guard
#define actasp_FilteringKR_h__guard

#include <actasp/AspKR.h>


namespace actasp {


struct FilteringKR : public actasp::AspKR {
  
  virtual GraphPolicy* computePolicy(const std::vector<actasp::AspRule>& goal, double suboptimality) const throw (std::logic_error) = 0;
  
  virtual AnswerSet filterState(const std::vector<actasp::AnswerSet>& plans, const std::vector<actasp::AspRule>& goals) = 0;
  
  virtual ~FilteringKR() {}
};
  
}
#endif
