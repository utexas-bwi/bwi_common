#ifndef bwi_krexec_ActionLogger_h__guard
#define bwi_krexec_ActionLogger_h__guard

#include "actasp/ExecutionObserver.h"

#include <fstream>

struct ActionLogger : public actasp::ExecutionObserver {
  
  ActionLogger();
  
  void actionStarted(const actasp::AspFluent& action) throw();
  void actionTerminated(const actasp::AspFluent& action) throw();
  
  void setFile(const std::string& path);
  void taskCompleted();
  
  ~ActionLogger();

private:
  
  std::ofstream *dest_file;
};



#endif
