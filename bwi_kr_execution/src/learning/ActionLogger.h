#ifndef plan_exec_ActionLogger_h__guard
#define plan_exec_ActionLogger_h__guard

#include "actasp/ExecutionObserver.h"

#include <fstream>

namespace actasp {
  class PartialPolicy;
}

struct ActionLogger : public actasp::ExecutionObserver {
  
  ActionLogger();
  
  void actionStarted(const actasp::AspFluent& action) throw();
  void actionTerminated(const actasp::AspFluent& action) throw();
  
  void setFile(const std::string& path);
  void taskCompleted();
  
  void goalChanged(std::vector<actasp::AspRule> newGoalRules) throw() {}
  void policyChanged(actasp::PartialPolicy* policy) throw() {}
  
  ~ActionLogger();

private:
  
  std::ofstream *dest_file;
};



#endif
