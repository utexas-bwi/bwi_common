
#include "actasp_ros/msgs_utils.h"

#include <algorithm>
#include <iterator>
#include <actasp_ros/AspFluent.h>

using namespace std;

namespace actasp_ros {

actasp::AspFluent TranslateFluent::operator()(const actasp_ros::AspFluent &bwiFluent) {
  std::stringstream ss;
  ss << bwiFluent.name << "(";
  for (const auto &argument : bwiFluent.arguments) {
    ss << argument << ", ";
  }
  ss << bwiFluent.timeStep << ")";

  return actasp::AspFluent::from_string(ss.str().c_str());
}

actasp_ros::AspFluent TranslateFluent::operator()(const actasp::AspFluent &actaspFluent) {

  actasp_ros::AspFluent bwiFluent;
  bwiFluent.name = actaspFluent.name;
  bwiFluent.timeStep = actaspFluent.getTimeStep();
  bwiFluent.arguments.reserve(actaspFluent.arguments.size());
  transform(actaspFluent.arguments.begin(), actaspFluent.arguments.end(),
            bwiFluent.arguments.begin(), [](const actasp::AspTerm &as_term) {
        return as_term.to_string();
      });

  return bwiFluent;
}


actasp::AspRule TranslateRule::operator()(const actasp_ros::AspRule &bwiRule) {
  actasp::AspRule actaspRule;

  // transform(bwiRule.head.begin(), bwiRule.head.end(), back_inserter(actaspRule.head), TranslateFluent());
  //transform(bwiRule.body.begin(), bwiRule.body.end(), back_inserter(actaspRule.body), TranslateFluent());
  assert(false);

  return actaspRule;
}

actasp_ros::AspRule TranslateRule::operator()(const actasp::AspRule &actaspRule) {
  actasp_ros::AspRule bwiRule;
  // TODO: Fix the rule message
  //transform(actaspRule.head.begin(), actaspRule.head.end(), back_inserter(bwiRule.head), TranslateFluent());
  //transform(actaspRule.body.begin(), actaspRule.body.end(), back_inserter(bwiRule.body), TranslateFluent());
  assert(false);
  return bwiRule;

}

actasp::AnswerSet TranslateAnswerSet::operator()(const actasp_ros::AnswerSet &bwiAnswerSet) {

  if (!bwiAnswerSet.satisfied)
    return actasp::AnswerSet();

  std::vector<actasp::AspFluent> fluents;
  transform(bwiAnswerSet.fluents.begin(), bwiAnswerSet.fluents.end(), back_inserter(fluents), TranslateFluent());

  return actasp::AnswerSet({}, fluents);
}

actasp_ros::AnswerSet TranslateAnswerSet::operator()(const actasp::AnswerSet &actaspAnswerSet) {
  actasp_ros::AnswerSet bwiAnswerSet;

  transform(actaspAnswerSet.fluents.begin(), actaspAnswerSet.fluents.end(), back_inserter(bwiAnswerSet.fluents),
            TranslateFluent());
  bwiAnswerSet.satisfied = actaspAnswerSet.satisfied;

  return bwiAnswerSet;
}


}
