#pragma once

#include "actasp/Action.h"

#include <actionlib/client/simple_action_client.h>

namespace actasp_ros {

template<typename ROSAction, typename Goal, typename Result>
class RosAction : public actasp::Action {
public:

  typedef actionlib::SimpleActionClient<ROSAction> ActionClient;
  typedef typename boost::shared_ptr<const Result> ResultConstPtr;

  explicit RosAction(const std::string &action_topic_name) :
      done(false),
      failed(false),
      action_topic_name(action_topic_name),
      request_in_progress(false), ac() {}

  ~RosAction() {
    if (request_in_progress) {
      // The goal was sent but the action is being terminated before being allowed to finish. Cancel that command!
      ac->cancelGoal();
    }
  }

  virtual void run() {
    if (!request_in_progress) {
      auto goal = prepareGoal();
      // The subclass didn't provide a goal. This is usually an error
      if (!goal) {
        ROS_ERROR_STREAM("Ran " << getName() << " but did not return a goal from prepareGoal()");
        failed = true;
        onFinished(false, {});
        done = true;
        return;
      }
      ac = std::unique_ptr<ActionClient>(new ActionClient(action_topic_name, true));
      bool got_server = ac->waitForServer(ros::Duration(7));
      if (!got_server) {
        ROS_WARN("Could not get server for %s at %s", getName().c_str(), action_topic_name.c_str());
        done = true;
        failed = true;
        onFinished(false, {});
        ac.reset();
        return;
      }
      ac->sendGoal(*goal);
      request_in_progress = true;
    }

    bool got_result = checkForResult();
    if (got_result) {
      onFinished(!failed, *result);
      // Mark the request as completed.
      done = true;
      ac.reset();
    }

  }

  bool hasFinished() const { return done; }

  bool hasFailed() const { return failed; }

protected:

  virtual boost::optional<Goal> prepareGoal() = 0;

  virtual void onFinished(bool success, const Result &result) = 0;

  bool checkForResult() {
    bool finished_before_timeout = ac->waitForResult(ros::Duration(0.5f));

    // If the action finished, need to do some work here.
    if (finished_before_timeout) {
      // Keep a copy of the result
      result = std::unique_ptr<Result>(new Result(*ac->getResult()));

      if (ac->getState() == actionlib::SimpleClientGoalState::ABORTED ||
          ac->getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
        failed = true;
      }

      // Cleanup the simple action client.
      request_in_progress = false;
      return true;
    }

    return false;

  }


private:
  std::unique_ptr<ActionClient> ac;
  std::unique_ptr<Result> result;
  const std::string action_topic_name;
  bool done;
  bool failed;
  bool request_in_progress;
};
}
