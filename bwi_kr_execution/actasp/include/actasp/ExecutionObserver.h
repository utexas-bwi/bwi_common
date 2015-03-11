#ifndef actasp_ExecutionObserver_h__guard
#define actasp_ExecutionObserver_h__guard



namespace actasp {

class AspFluent;
class AnswerSet;

struct ExecutionObserver {

  virtual void actionStarted(const AspFluent& action) throw() =0 ;
  virtual void actionTerminated(const AspFluent& action) throw() =0;

  virtual ~ExecutionObserver() {}
};

}


#endif
