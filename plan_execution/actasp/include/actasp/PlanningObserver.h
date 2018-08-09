#ifndef actasp_PlanningObserver_h__guard
#define actasp_PlanningObserver_h__guard

#include <functional>


namespace actasp {

class AnswerSet;

struct PlanningObserver {

  virtual void planChanged(const AnswerSet& newPlan) noexcept =0 ;

  virtual ~PlanningObserver() = default;

  bool operator==(const PlanningObserver &other) const {
    return this == &other;
  }
};

inline bool operator==(const std::reference_wrapper<PlanningObserver> &first,
                       const std::reference_wrapper<PlanningObserver> &second) {
  return first.get() == second.get();
}

}


#endif
