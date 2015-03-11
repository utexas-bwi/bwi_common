
#ifndef actasp_execution_observer_utiles_h__guard
#define actasp_execution_observer_utiles_h__guard

#include <actasp/AspFluent.h>
#include <actasp/ExecutionObserver.h>

namespace actasp {

  
struct NotifyActionTermination {
  
  NotifyActionTermination(const AspFluent& action) : action(action) {}
  
  void operator()(ExecutionObserver *observer) {
    observer->actionTerminated(action);
  }
  
  AspFluent action;
};

struct NotifyActionStart {
  
  NotifyActionStart(const AspFluent& action) : action(action) {}
  
  void operator()(ExecutionObserver *observer) {
    observer->actionStarted(action);
  }
  
  AspFluent action;
};

}

#endif
