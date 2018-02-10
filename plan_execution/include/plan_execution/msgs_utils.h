#ifndef plan_execution_msgs_utils_h__guard
#define plan_execution_msgs_utils_h__guard

#include "plan_execution/AspFluent.h"
#include "plan_execution/AspRule.h"
#include "plan_execution/AnswerSet.h"

#include "actasp/AspFluent.h"
#include "actasp/AspRule.h"
#include "actasp/AnswerSet.h"

namespace plan_exec {

struct TranslateFluent {
  
 actasp::AspFluent operator()(const plan_execution::AspFluent& bwiFluent);
 plan_execution::AspFluent operator()(const actasp::AspFluent& actaspFluent);
};

struct TranslateRule {
  actasp::AspRule operator()(const plan_execution::AspRule& bwiRule);
  plan_execution::AspRule operator()(const actasp::AspRule& actaspRule);

};

struct TranslateAnswerSet {
  actasp::AnswerSet operator()(const plan_execution::AnswerSet& bwiAnswerSet);
  plan_execution::AnswerSet operator()(const actasp::AnswerSet& actaspAnswerSet);
  
};

}


#endif
