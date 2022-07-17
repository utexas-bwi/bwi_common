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
*********************************************************************/

#ifndef BWI_INTERRUPTABLE_ACTION_SERVER_H_
#define BWI_INTERRUPTABLE_ACTION_SERVER_H_

#include <boost/thread/condition.hpp>
#include <ros/ros.h>
#include <actionlib/server/action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/action_definition.h>
#include <std_srvs/Empty.h>

namespace bwi_interruptable_action_server {
  template <class ActionSpec>
  class InterruptableActionServer {
    public:

      ACTION_DEFINITION(ActionSpec);

      typedef typename actionlib::ActionServer<ActionSpec>::GoalHandle GoalHandle;
      typedef boost::function<void(const ResultConstPtr&, const actionlib::SimpleClientGoalState&, int)> ResultCallback;
      typedef boost::function<void(const GoalConstPtr&)> NewGoalCallback;
      
      InterruptableActionServer(ros::NodeHandle n, 
                                std::string name, 
                                int max_attempts = 1, 
                                NewGoalCallback new_goal_callback = 0,
                                ResultCallback result_callback = 0);
      ~InterruptableActionServer();

      void spin();

    protected:

      bool pause(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
      bool resume(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
      void goalCallback(GoalHandle goal);
      void cancelCallback(GoalHandle preempt);

      void publishFeedback(const FeedbackConstPtr& feedback);

      ros::NodeHandle n_;

      ros::ServiceServer pause_server_;
      ros::ServiceServer resume_server_;

      boost::shared_ptr<actionlib::ActionServer<ActionSpec> > as_;
      boost::shared_ptr<actionlib::SimpleActionClient<ActionSpec> > ac_;

      boost::recursive_mutex lock_;

      std::string interruptable_server_name_;

      GoalHandle current_goal_, original_goal_, next_goal_;

      bool original_goal_available_;
      bool switch_to_original_goal_;
      bool next_goal_available_;
      bool pursue_current_goal_;
      bool pursuing_current_goal_;

      int max_attempts_;
      int current_attempts_;
      ResultCallback result_callback_;

      NewGoalCallback new_goal_callback_;
  };
};

//include the implementation here
#include <bwi_interruptable_action_server/interruptable_action_server_imp.h>
#endif
