#include <ros/ros.h>
#include "utils.h"

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "kb_to_asp");
    ros::NodeHandle n;

    cout << "Current knowledge state:" << endl;
    cout <<  bwi_krexec::memoryConduitToAsp();
    return 0;

}