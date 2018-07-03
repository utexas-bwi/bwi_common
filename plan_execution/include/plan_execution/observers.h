#ifndef PLAN_EXECUTION_OBSERVERS_H
#define PLAN_EXECUTION_OBSERVERS_H

#include <ros/ros.h>
#include <actasp/ExecutionObserver.h>
#include <actasp/PlanningObserver.h>
#include <utility>

namespace plan_exec {

struct ConsoleObserver : public actasp::ExecutionObserver, public actasp::PlanningObserver {

    void actionStarted(const actasp::AspFluent &action) noexcept override {
        ROS_INFO_STREAM("Starting execution: " << action.toString());
    }

    void actionTerminated(const actasp::AspFluent &action, bool succeeded) noexcept override {
        ROS_INFO_STREAM("Terminating execution: " << action.toString() << " Success:" << succeeded);
    }

    void planChanged(const actasp::AnswerSet &newPlan) noexcept {
        std::stringstream planStream;

        ROS_INFO_STREAM("plan size: " << newPlan.getFluents().size());

        copy(newPlan.getFluents().begin(), newPlan.getFluents().end(), std::ostream_iterator<std::string>(planStream, " "));

        ROS_INFO_STREAM(planStream.str());
    }

    void planTerminated(const PlanStatus status, const actasp::AspFluent &final_action, const actasp::AnswerSet &plan_remainder) noexcept {
        std::stringstream planStream;

        planStream << "Plan execution terminated. Status: " << planStatusToString(status) << ". ";

        planStream << "Final action: " << final_action.toString() << std::endl;

        planStream << "Remaining plan: ";

        copy(plan_remainder.getFluents().begin(), plan_remainder.getFluents().end(), std::ostream_iterator<std::string>(planStream, " "));

        ROS_INFO_STREAM(planStream.str());
    }

    void goalChanged(const std::vector<actasp::AspRule>& newGoalRules) noexcept override {
    }

    void policyChanged(actasp::PartialPolicy *policy) noexcept {}

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

    void planChanged(const actasp::AnswerSet &newPlan) noexcept {
        feedback.plan.clear();
        feedback.event_type = plan_execution::ExecutePlanFeedback::PLAN_CHANGED_EVENT;
        std::vector<actasp::AspFluent> fluents = newPlan.getFluents();
        std::transform(fluents.begin(), fluents.end(), std::back_inserter(feedback.plan), plan_exec::TranslateFluent());
        server.publishFeedback(feedback);
        //ros::spinOnce();
    }

    void planTerminated(const PlanStatus status, const actasp::AspFluent &final_action, const actasp::AnswerSet &plan_remainder) noexcept {
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

    void policyChanged(actasp::PartialPolicy *policy) noexcept {}


};
}

#endif //PLAN_EXECUTION_OBSERVERS_H
