#pragma once

#include "actasp_ros/AspFluent.h"
#include "actasp_ros/AspRule.h"
#include "actasp_ros/AnswerSet.h"

#include "actasp/asp/AspFluent.h"
#include "actasp/asp/AspRule.h"
#include "actasp/AnswerSet.h"

namespace actasp_ros {

struct TranslateFluent {

  actasp::AspFluent operator()(const actasp_ros::AspFluent &bwiFluent);

  actasp_ros::AspFluent operator()(const actasp::AspFluent &actaspFluent);
};

struct TranslateAtom {

  actasp::AspAtom operator()(const actasp_ros::AspFunction &ros_atom);

  actasp_ros::AspFunction operator()(const actasp::AspAtom &atom);
};

struct TranslateRule {
  actasp::AspRule operator()(const actasp_ros::AspRule &bwiRule);

  actasp_ros::AspRule operator()(const actasp::AspRule &actaspRule);

};

struct TranslateAnswerSet {
  actasp::AnswerSet operator()(const actasp_ros::AnswerSet &bwiAnswerSet);

  actasp_ros::AnswerSet operator()(const actasp::AnswerSet &actaspAnswerSet);

};

}