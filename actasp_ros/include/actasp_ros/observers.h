#pragma once

#include <utility>

#include <ros/ros.h>
#include <actasp/ExecutionObserver.h>
#include <actasp/PlanningObserver.h>
#include <actasp/Solver.h>
#include <actasp/AnswerSet.h>
#include <utility>
#include <boost/filesystem.hpp>
#include <iostream>

namespace actasp_ros {

struct ConsoleObserver : public actasp::ExecutionObserver, public actasp::PlanningObserver {

  void actionStarted(const actasp::AspFluent &action) noexcept override {
    ROS_INFO_STREAM("Starting execution: " << action.to_string());
  }

  void actionTerminated(const actasp::AspFluent &action, bool succeeded) noexcept override {
    ROS_INFO_STREAM("Terminating execution: " << action.to_string() << " Success:" << succeeded);
  }

  void planChanged(const actasp::AnswerSet &newPlan) noexcept override {
    std::stringstream planStream;

    ROS_INFO_STREAM("plan size: " << newPlan.fluents.size());

    for (const auto &fluent: newPlan.fluents) {
      planStream << fluent.to_string() << " ";
    }

    ROS_INFO_STREAM(planStream.str());
  }

  void planTerminated(const PlanStatus status, const actasp::AspFluent &final_action,
                      const actasp::AnswerSet &plan_remainder) noexcept override {
    std::stringstream planStream;

    planStream << "Plan execution terminated. Status: " << planStatusToString(status) << ". ";

    planStream << "Final action: " << final_action.to_string() << std::endl;

    planStream << "Remaining plan: ";

    for (const auto &fluent: plan_remainder.fluents) {
      planStream << fluent.to_string() << " ";
    }

    ROS_INFO_STREAM(planStream.str());
  }

  void goalChanged(const std::vector<actasp::AspRule> &newGoalRules) noexcept override {
  }

  void policyChanged(actasp::PartialPolicy *policy) noexcept override {}

};


struct RosActionServerInterfaceObserver : public actasp::ExecutionObserver, public actasp::PlanningObserver {

  actionlib::SimpleActionServer<actasp_ros::ExecutePlanAction> &server;
  actasp_ros::ExecutePlanResult result;
  actasp_ros::ExecutePlanFeedback feedback;

  explicit RosActionServerInterfaceObserver(actionlib::SimpleActionServer<actasp_ros::ExecutePlanAction> &server)
      : server(server), result(), feedback() {}

  void actionStarted(const actasp::AspFluent &action) noexcept override {
    feedback.plan.clear();
    feedback.event_type = actasp_ros::ExecutePlanFeedback::ACTION_STARTED_EVENT;
    feedback.plan.push_back(actasp_ros::TranslateFluent()(action));
    server.publishFeedback(feedback);
    //ros::spinOnce();
  }

  void actionTerminated(const actasp::AspFluent &action, bool succeeded) noexcept override {
    feedback.plan.clear();
    feedback.event_type = actasp_ros::ExecutePlanFeedback::ACTION_ENDED_EVENT;
    feedback.plan.push_back(actasp_ros::TranslateFluent()(action));
    server.publishFeedback(feedback);
    //ros::spinOnce();
  }

  void planChanged(const actasp::AnswerSet &newPlan) noexcept override {
    feedback.plan.clear();
    feedback.event_type = actasp_ros::ExecutePlanFeedback::PLAN_CHANGED_EVENT;
    std::vector<actasp::AspFluent> fluents = newPlan.fluents;
    std::transform(fluents.begin(), fluents.end(), std::back_inserter(feedback.plan), actasp_ros::TranslateFluent());
    server.publishFeedback(feedback);
    //ros::spinOnce();
  }

  void planTerminated(const PlanStatus status, const actasp::AspFluent &final_action,
                      const actasp::AnswerSet &plan_remainder) noexcept override {
    switch (status) {
      case SUCCEEDED:
        result.status = actasp_ros::ExecutePlanResult::SUCCEEDED;
        break;
      case FAILED_TO_PLAN:
        result.status = actasp_ros::ExecutePlanResult::FAILED_TO_PLAN;
        break;
      case TOO_MANY_ACTION_FAILURES:
        result.status = actasp_ros::ExecutePlanResult::TOO_MANY_ACTION_FAILURES;
        break;
    }
    std::vector<actasp::AspFluent> fluents = plan_remainder.fluents;
    result.final_action = actasp_ros::TranslateFluent()(final_action);
    std::transform(fluents.begin(), fluents.end(), std::back_inserter(result.plan_remainder),
                   actasp_ros::TranslateFluent());
  }

  void goalChanged(const std::vector<actasp::AspRule> &newGoalRules) noexcept override {
  }

  void policyChanged(actasp::PartialPolicy *policy) noexcept override {}

};

struct ExplainingRosActionServerInterfaceObserver : public RosActionServerInterfaceObserver {

  std::unique_ptr<actasp::Solver> explainer;
  std::vector<actasp::AspRule> goalRules;

  ExplainingRosActionServerInterfaceObserver(actionlib::SimpleActionServer<actasp_ros::ExecutePlanAction> &server,
                                             std::unique_ptr<actasp::Solver> explainer)
      : explainer(std::move(explainer)), RosActionServerInterfaceObserver(server) {}


  void planTerminated(const PlanStatus status, const actasp::AspFluent &final_action,
                      const actasp::AnswerSet &plan_remainder) noexcept override {
    RosActionServerInterfaceObserver::planTerminated(status, final_action, plan_remainder);
    if (status == FAILED_TO_PLAN) {
      std::vector<actasp::AspRule> hypotheses;
      // Hypotheticals are rules in the head of the goal
      std::copy_if(goalRules.begin(), goalRules.end(), std::back_inserter(hypotheses),
                   [](actasp::AspRule &rule) {
                     return !rule.head.empty();
                   });

      std::stringstream fluentsString, minimizeString;

      /*for (const auto &rule : hypotheses) {
        fluentsString << "0{" << rule.head[0].to_string() << "}1." << std::endl;
        minimizeString  << ":~ " << rule.head[0].to_string() << ". [-1, 1]" << std::endl;
      }*/
      // FIXME: Support this type of query again
      /*std::vector<actasp::AspRule> query;
      std::list<actasp::AnswerSet> answers = explainer->genericQuery(query, 0, "diagnoticQuery", 0);
      const auto &answer = answers.front();
      std::vector<actasp::AspRule> failedHypotheses;
      for (const auto &rule : hypotheses) {
        if (!answer.contains(rule.head[0].to_string())) {
          failedHypotheses.push_back(rule);
        }
      }

      if (!failedHypotheses.empty()) {
        result.message = "I could not complete the task due to inconsistent hypotheses. ";
        std::transform(failedHypotheses.begin(), failedHypotheses.end(), std::back_inserter(result.inconsistent_rules),
        actasp_ros::TranslateRule());
      }
      else {
        result.message = "I could not find a plan to complete the task. ";
      }*/
    };


  }

  void goalChanged(const std::vector<actasp::AspRule> &newGoalRules) noexcept override {
    goalRules.clear();
    std::copy(newGoalRules.begin(), newGoalRules.end(), std::back_inserter(goalRules));
  }

};


}