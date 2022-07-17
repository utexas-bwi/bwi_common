/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, University of Texas at Austin 
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of UT Austin nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
*********************************************************************/

#include <actionlib/client/simple_client_goal_state.h>

namespace bwi_interruptable_action_server {

  template <class ActionSpec>
  InterruptableActionServer<ActionSpec>::InterruptableActionServer(ros::NodeHandle n, 
                                                                   std::string name,
                                                                   int max_attempts,
                                                                   NewGoalCallback new_goal_callback,
                                                                   ResultCallback result_callback) :
      n_(n),
      original_goal_available_(false),
      switch_to_original_goal_(false),
      next_goal_available_(false),
      pursue_current_goal_(false),
      pursuing_current_goal_(false),
      max_attempts_(max_attempts),
      result_callback_(result_callback),
      new_goal_callback_(new_goal_callback) {

    interruptable_server_name_ = name + "_interruptable";

    // create the action server.
    as_.reset(new actionlib::ActionServer<ActionSpec>(n_, 
                                                      interruptable_server_name_,
                                                      boost::bind(&InterruptableActionServer::goalCallback, this, _1),
                                                      boost::bind(&InterruptableActionServer::cancelCallback, this, _1),
                                                      false));

    // create the pause and resume services.
    pause_server_ = n_.advertiseService(interruptable_server_name_ + "/pause", 
                                        &InterruptableActionServer::pause, 
                                        this);
    resume_server_ = n_.advertiseService(interruptable_server_name_ + "/resume", 
                                         &InterruptableActionServer::resume, 
                                         this);

    // Create the lower level simple action client to the uninterruptable action server.
    ac_.reset(new actionlib::SimpleActionClient<ActionSpec>(name, true));
  }

  template <class ActionSpec>
  InterruptableActionServer<ActionSpec>::~InterruptableActionServer() {
    if (original_goal_available_) {
      original_goal_.setCanceled(Result(), "The goal was cancelled as the action server is shutting down.");
    }

    if (pursuing_current_goal_ || pursue_current_goal_) {
      current_goal_.setCanceled(Result(), "The goal was cancelled as the action server is shutting down.");
    }

    if (next_goal_available_) {
      next_goal_.setCanceled(Result(), "The goal was cancelled as the action server is shutting down.");
    }
  }

  template <class ActionSpec>
  bool InterruptableActionServer<ActionSpec>::pause(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    boost::recursive_mutex::scoped_lock lock(lock_);
    bool ret_val = true;
    if (original_goal_available_) {
      ROS_ERROR_STREAM(interruptable_server_name_ + " : Already paused one goal, cannot pause another.");
      ret_val = false;
    } else if (!pursue_current_goal_) {
      ROS_ERROR_STREAM(interruptable_server_name_ + " : Not currently actively pursuing a goal, cannot pause.");
      ret_val = false;
    } else {
      pursue_current_goal_ = false;
      original_goal_ = current_goal_;
      original_goal_available_ = true;
    }
    return ret_val;
  }

  template <class ActionSpec>
  bool InterruptableActionServer<ActionSpec>::resume(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    boost::recursive_mutex::scoped_lock lock(lock_);
    bool ret_val = true;
    if (!original_goal_available_) {
      ROS_ERROR_STREAM(interruptable_server_name_ + " : No paused goal available, cannot resume goal.");
      ret_val = false;
    } else {
      // Restart the current goal again.
      switch_to_original_goal_ = true;
    }
    return ret_val;
  }

  template <class ActionSpec>
  void InterruptableActionServer<ActionSpec>::goalCallback(GoalHandle goal) {
    boost::recursive_mutex::scoped_lock lock(lock_);
    // check that the timestamp is past or equal to that of the current goal and the next goal
    if ((!current_goal_.getGoal() || goal.getGoalID().stamp >= current_goal_.getGoalID().stamp)
        && (!next_goal_.getGoal() || goal.getGoalID().stamp >= next_goal_.getGoalID().stamp)) {

      //if next_goal has not been accepted already... its going to get bumped, but we need to let the client know we're preempting
      if (next_goal_available_) {
        next_goal_.setCanceled(Result(), "This goal was canceled because another goal was recieved by the simple action server");
        next_goal_available_ = false;
      }

      next_goal_ = goal;
      next_goal_available_ = true; 
      
      ROS_INFO_STREAM(interruptable_server_name_ + " : Received new goal! Passing it to low level action server.");
    } else {
      goal.setCanceled(Result(), "This goal was canceled because another goal was recieved by the simple action server");
    }

  }
  
  template <class ActionSpec>
  void InterruptableActionServer<ActionSpec>::cancelCallback(GoalHandle goal) {
    boost::recursive_mutex::scoped_lock lock(lock_);
    if (goal == current_goal_) {
      pursue_current_goal_ = false;
    } else if (goal == next_goal_) {
      next_goal_.setCanceled(Result());
      next_goal_available_ = false;
    } else if (goal == original_goal_ && original_goal_available_) {
      // Don't need to pursue the original goal again. Since we were in paused state, we might be pursuing a goal
      // currently. Cancel the current and next goals as well.
      original_goal_.setCanceled(Result());
      original_goal_available_ = false;
      if (next_goal_available_) {
        next_goal_.setCanceled(Result());
        next_goal_available_ = false;
      }
      pursue_current_goal_ = false;
    }
  }

  template <class ActionSpec>
  void InterruptableActionServer<ActionSpec>::spin() {
    as_->start();
    ros::Rate r(30);
    while (n_.ok()) {
      ros::spinOnce();
      boost::recursive_mutex::scoped_lock lock(lock_);

      // Switch to original goal if resume was called.
      if (switch_to_original_goal_) {
        if (pursuing_current_goal_) {
          current_goal_.setCanceled(Result(), "This goal was preempted as a paused goal was resumed.");
          pursuing_current_goal_ = false;
        }
        if (next_goal_available_) {
          next_goal_available_ = false;
          next_goal_.setCanceled(Result(), "This goal was preempted as a paused goal was resumed.");
        }
        current_attempts_ = 0;
        current_goal_ = original_goal_;
        original_goal_available_ = false;
        switch_to_original_goal_ = false;
        pursue_current_goal_ = true;
      } 
      
      // Switch to a new goal if a new goal is available.
      if (next_goal_available_) {
        if (pursuing_current_goal_) {
          current_goal_.setCanceled(Result(), "This goal was preempted by a new goal");
          pursuing_current_goal_ = false;
        }
        current_attempts_ = 0;
        current_goal_ = next_goal_;
        current_goal_.setAccepted();
        if (new_goal_callback_) {
          new_goal_callback_(current_goal_.getGoal());
        }
        pursue_current_goal_ = true;
        next_goal_available_ = false;
      } 
      
      // If we are not pursuing the current goal, and we should be, then start pursuing the current goal.
      if (!pursuing_current_goal_ && pursue_current_goal_) {
        // Send the current goal and hookup the feedback publisher.
        ac_->sendGoal(*(current_goal_.getGoal()),
                      typename actionlib::SimpleActionClient<ActionSpec>::SimpleDoneCallback(),
                      typename actionlib::SimpleActionClient<ActionSpec>::SimpleActiveCallback(),
                      boost::bind(&InterruptableActionServer::publishFeedback, this, _1));
        pursuing_current_goal_ = true;
      } else if (pursuing_current_goal_ && !pursue_current_goal_) {
        // We were pursuing a goal that we do not need to pursue any more.
        ac_->cancelGoal();
        pursuing_current_goal_ = false;
      } else if (pursuing_current_goal_ && pursue_current_goal_) {
        // See if goal is still being pursued.
        actionlib::SimpleClientGoalState state = ac_->getState();
        if (state != actionlib::SimpleClientGoalState::PENDING &&
            state != actionlib::SimpleClientGoalState::ACTIVE) {
          // This means that the the goal is no longer being pursued. Publish the result here from whatever information
          // we've received from the regular action client.
          ResultConstPtr result = ac_->getResult();
          current_attempts_ += 1;
          if (result_callback_) {
            result_callback_(result, state, current_attempts_);
          }
          if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            current_goal_.setSucceeded(*result);
            // The goal should not be pursued again.
            pursuing_current_goal_ = false;
            pursue_current_goal_ = false;
          } else if (state == actionlib::SimpleClientGoalState::ABORTED || 
                     state == actionlib::SimpleClientGoalState::REJECTED) {
            if (current_attempts_ < max_attempts_) {
              ++current_attempts_;
              // This wall cause the same goal to be resent.
              pursuing_current_goal_ = false;
            } else {
              current_goal_.setAborted(*result);
              // The goal should not be pursued again.
              pursuing_current_goal_ = false;
              pursue_current_goal_ = false;
            }
          } else {
            current_goal_.setCanceled(*result);
            // Since the goal got completed, we no longer wish to pursue this goal.
            pursuing_current_goal_ = false;
            pursue_current_goal_ = false;
          }

        }
      }
      r.sleep();
    }
  }

  template <class ActionSpec>
  void InterruptableActionServer<ActionSpec>::publishFeedback(const FeedbackConstPtr& feedback) {
    current_goal_.publishFeedback(*feedback);
  }

};

