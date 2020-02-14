#include <string.h>
#include <boost/circular_buffer.hpp>
#include <ros/ros.h>
#include <sstream>

using namespace ros;
using namespace std;

int main(int argc, char**argv){
  ros::init(argc, argv, "bypass_test");
  ros::NodeHandle nh("~");

  boost::circular_buffer<int> othersHistory_;
  othersHistory_ = boost::circular_buffer<int>(20);

  for(int i=0; i<100; i++){
    othersHistory_.push_back(i);
    stringstream ss;
    for(int j = 0; j<othersHistory_.size(); j++){
      ss << othersHistory_[j] << ", ";
    }
    cout<<ss.str()<<"\n";
  }
}
