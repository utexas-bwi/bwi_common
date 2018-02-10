#ifndef actasp_PlanningObserver_h__guard
#define actasp_PlanningObserver_h__guard



namespace actasp {

class AnswerSet;

struct PlanningObserver {

  virtual void planChanged(const AnswerSet& newPlan) throw() =0 ;

  virtual ~PlanningObserver() {}
};

}


#endif
