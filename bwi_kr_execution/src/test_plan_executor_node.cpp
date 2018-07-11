#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <plan_execution/ExecutePlanAction.h>

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_action_executor");
    ros::NodeHandle n;

    actionlib::SimpleActionClient<plan_execution::ExecutePlanAction> ac(n, "plan_executor/execute_plan");

    plan_execution::ExecutePlanGoal goal;
    plan_execution::AspFluent goal_fluent;
    if (argc < 3) {
        cout << "Please pass arguments" << endl;
        cout << "Usage: fluent_name [param1 [...]]" << endl;
        return 1;
    }
    goal_fluent.name = argv[1];
    goal_fluent.name = "not " + goal_fluent.name;
    for (int i = 2; i < argc; ++i) {
        goal_fluent.variables.push_back(argv[i]);
    }
    
    plan_execution::AspRule goal_rule;
    goal_rule.body.push_back(goal_fluent);
    goal.aspGoal.push_back(goal_rule);
    ac.waitForServer();
    ROS_INFO("sending goal");

    ac.sendGoal(goal);

    ros::Rate wait_rate(1);
    while (ros::ok() && !ac.getState().isDone())
        wait_rate.sleep();

    if (!ac.getState().isDone()) {
        ROS_INFO("Canceling goal");
        ac.cancelGoal();
        //and wait for canceling confirmation
        for (int i = 0; !ac.getState().isDone() && i < 50; ++i)
            wait_rate.sleep();
    }
    if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED) {
        ROS_INFO("Aborted");
    } else if (ac.getState() == actionlib::SimpleClientGoalState::PREEMPTED) {
        ROS_INFO("Preempted");
    } else if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Succeeded!");
    } else
        ROS_INFO("Terminated");

    return 0;

}