#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <plan_execution/ExecutePlanAction.h>
#include <actasp/AspFluent.h>
#include "actions/ActionFactory.h"
#include "utils.h"
#include <actasp/Action.h>

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_action_executor");
    ros::NodeHandle n;

    if (argc < 3) {
        cout << "Please pass arguments" << endl;
        cout << "Usage: action_name [param1 [...]]" << endl;
        return 1;
    }
    actasp::Action *action_template;
    try {
        action_template = bwi_krexec::ActionFactory::byName(argv[1]);
    } catch (runtime_error &e) {
        cout << "Couldn't find an action for " << argv[1] << endl;
        cout << "Are you sure it's registered?" << endl;
        return 2;
    }
    vector<string> params(argv + 2, argv + argc);
    actasp::AspFluent action_fluent(argv[1], params);

    auto action = action_template->cloneAndInit(action_fluent);

    ros::Rate r(10);
    while (!action->hasFinished() && ros::ok()) {
        action->run();
        ros::spinOnce();
        r.sleep();
    }
    if (action->hasFailed()) {
        cout << "Failed" << endl;
        return 3;
    }
    cout << "Succeeded" << endl;

    return 0;

}