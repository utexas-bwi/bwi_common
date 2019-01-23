#pragma once

#include <utility>

#include <ros/ros.h>
#include <actasp/ExecutionObserver.h>
#include <actasp/PlanningObserver.h>
#include <actasp/QueryGenerator.h>
#include <actasp/AnswerSet.h>
#include <utility>
#include <boost/filesystem.hpp>
#include <iostream>

namespace plan_exec {

struct ConsoleObserver : public actasp::ExecutionObserver, public actasp::PlanningObserver {

    void actionStarted(const actasp::AspFluent &action) noexcept override {
        ROS_INFO_STREAM("Starting execution: " << action.to_string());
    }

    void actionTerminated(const actasp::AspFluent &action, bool succeeded) noexcept override {
        ROS_INFO_STREAM("Terminating execution: " << action.to_string() << " Success:" << succeeded);
    }

    void planChanged(const actasp::AnswerSet &newPlan) noexcept override {
        std::stringstream planStream;

        ROS_INFO_STREAM("plan size: " << newPlan.getFluents().size());

        copy(newPlan.getFluents().begin(), newPlan.getFluents().end(), std::ostream_iterator<std::string>(planStream, " "));

        ROS_INFO_STREAM(planStream.str());
    }

    void planTerminated(const PlanStatus status, const actasp::AspFluent &final_action, const actasp::AnswerSet &plan_remainder) noexcept override {
        std::stringstream planStream;

        planStream << "Plan execution terminated. Status: " << planStatusToString(status) << ". ";

        planStream << "Final action: " << final_action.to_string() << std::endl;

        planStream << "Remaining plan: ";

        copy(plan_remainder.getFluents().begin(), plan_remainder.getFluents().end(), std::ostream_iterator<std::string>(planStream, " "));

        ROS_INFO_STREAM(planStream.str());
    }

    void goalChanged(const std::vector<actasp::AspRule>& newGoalRules) noexcept override {
    }

    void policyChanged(actasp::PartialPolicy *policy) noexcept override {}

};


struct RosActionServerInterfaceObserver : public actasp::ExecutionObserver, public actasp::PlanningObserver {

    actionlib::SimpleActionServer<plan_execution::ExecutePlanAction> &server;
    plan_execution::ExecutePlanResult result;
    plan_execution::ExecutePlanFeedback feedback;

    explicit RosActionServerInterfaceObserver(actionlib::SimpleActionServer<plan_execution::ExecutePlanAction> &server): server(server), result(), feedback() {}

    void actionStarted(const actasp::AspFluent &action) noexcept override {
        feedback.plan.clear();
        feedback.event_type = plan_execution::ExecutePlanFeedback::ACTION_STARTED_EVENT;
        feedback.plan.push_back(plan_exec::TranslateFluent()(action));
        server.publishFeedback(feedback);
        //ros::spinOnce();
    }

    void actionTerminated(const actasp::AspFluent &action, bool succeeded) noexcept override {
        feedback.plan.clear();
        feedback.event_type = plan_execution::ExecutePlanFeedback::ACTION_ENDED_EVENT;
        feedback.plan.push_back(plan_exec::TranslateFluent()(action));
        server.publishFeedback(feedback);
        //ros::spinOnce();
    }

    void planChanged(const actasp::AnswerSet &newPlan) noexcept override {
        feedback.plan.clear();
        feedback.event_type = plan_execution::ExecutePlanFeedback::PLAN_CHANGED_EVENT;
        std::vector<actasp::AspFluent> fluents = newPlan.getFluents();
        std::transform(fluents.begin(), fluents.end(), std::back_inserter(feedback.plan), plan_exec::TranslateFluent());
        server.publishFeedback(feedback);
        //ros::spinOnce();
    }

    void planTerminated(const PlanStatus status, const actasp::AspFluent &final_action, const actasp::AnswerSet &plan_remainder) noexcept override {
       switch (status) {
           case SUCCEEDED:
               result.status = plan_execution::ExecutePlanResult::SUCCEEDED;
               break;
           case FAILED_TO_PLAN:
               result.status = plan_execution::ExecutePlanResult::FAILED_TO_PLAN;
               break;
           case TOO_MANY_ACTION_FAILURES:
               result.status = plan_execution::ExecutePlanResult::TOO_MANY_ACTION_FAILURES;
               break;
       }
       std::vector<actasp::AspFluent> fluents = plan_remainder.getFluents();
       result.final_action = plan_exec::TranslateFluent()(final_action);
       std::transform(fluents.begin(), fluents.end(), std::back_inserter(result.plan_remainder), plan_exec::TranslateFluent());
    }

    void goalChanged(const std::vector<actasp::AspRule>& newGoalRules) noexcept override {
    }

    void policyChanged(actasp::PartialPolicy *policy) noexcept override {}

};

struct ExplainingRosActionServerInterfaceObserver : public RosActionServerInterfaceObserver {

  std::unique_ptr<actasp::QueryGenerator> explainer;
  std::vector<actasp::AspRule> goalRules;

  ExplainingRosActionServerInterfaceObserver(actionlib::SimpleActionServer<plan_execution::ExecutePlanAction> &server, std::unique_ptr<actasp::QueryGenerator> explainer)
      :  explainer(std::move(explainer)), RosActionServerInterfaceObserver(server){}



  void planTerminated(const PlanStatus status, const actasp::AspFluent &final_action,
                      const actasp::AnswerSet &plan_remainder) noexcept override {
    RosActionServerInterfaceObserver::planTerminated(status, final_action, plan_remainder);
    if (status == FAILED_TO_PLAN) {
      std::vector<actasp::AspRule> hypotheses;
      // Hypotheticals are rules in the head of the goal
      std::copy_if(goalRules.begin(), goalRules.end(), std::back_inserter(hypotheses),
      [](actasp::AspRule & rule){
        return !rule.head.empty();
      });

      std::stringstream fluentsString, minimizeString;

      for (const auto &rule : hypotheses) {
        fluentsString << "0{" << rule.head[0].to_string() << "}1." << std::endl;
        minimizeString  << ":~ " << rule.head[0].to_string() << ". [-1, 1]" << std::endl;
      }
      // FIXME: Support this type of query again
      std::vector<actasp::AspRule> query;
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
        plan_exec::TranslateRule());
      }
      else {
        result.message = "I could not find a plan to complete the task. ";
      }
    };


  }

  void goalChanged(const std::vector<actasp::AspRule>& newGoalRules) noexcept override {
    goalRules.clear();
    std::copy(newGoalRules.begin(), newGoalRules.end(), std::back_inserter(goalRules));
}

};


}