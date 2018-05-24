#ifndef actasp_PlanningObserver_h__guard
#define actasp_PlanningObserver_h__guard



namespace actasp {

class AnswerSet;

struct PlanningObserver {

  virtual void planChanged(const AnswerSet& newPlan) noexcept =0 ;

  virtual ~PlanningObserver() {}
};

}


#endif
