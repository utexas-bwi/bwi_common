#include <signal.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <kinova_msgs/JointAngles.h>
//used for assignment of vector
#include <boost/assign/std/vector.hpp>
//services
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"

#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/JointState.h>

#define foreach BOOST_FOREACH

using namespace boost::assign;
bool g_caught_sigint = false;

geometry_msgs::PoseStamped pose_current;
sensor_msgs::JointState js_cur;

std::vector<double> angles_current;
void sig_handler(int sig){
	g_caught_sigint = true;
	ROS_INFO("caugt sigint, init shutdown seq...");
	ros::shutdown();
	exit(1);
};
void toolpos_cb(const geometry_msgs::PoseStamped &msg){
	pose_current = msg;
}
//Joint state cb
void joint_state_cb(const sensor_msgs::JointState &input){
	angles_current.clear();
	for(int i = 0; i < 6; i++){
		angles_current.push_back(input.position.at(i));
	}
	js_cur = input;
}
int main(int argc, char **argv){
	ros::init(argc, argv, "position_record_node");
	ros::NodeHandle node_handle;
	signal(SIGINT, sig_handler);

	//button position publisher
	ros::Publisher pose_pub = node_handle.advertise<geometry_msgs::PoseStamped>("target_trajectory/pose", 10);

	ros::Subscriber sub_tool = node_handle.subscribe("/m1n6s200_driver/out/tool_pose", 1, toolpos_cb);
	ros::Subscriber sub_angles = node_handle.subscribe ("/m1n6s200_driver/out/joint_state", 1, joint_state_cb);

	// Create a publisher for visualizing plans in Rviz.
	ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

	char in;
	int count = 0;
	std::cout << "For each trajectory, 2 key presses are required. When prompted, move the arm to the desired end position and press the first instructed key." << std::endl;
	std::cout << "Then, move the arm to the desired start position, and key press once more. You will then be prompted for the trajectory name. Usefull names are descriptive!" << std::endl;
	while(in != 'q'){
		std::cout << "MAIN MENU: q for quit, 1 for record, 2 for playback" << std::endl;		
		std::cin >> in;
		if(in == '1'){
			std::string filename;
			std::cout << "Please enter a filename for the trajectory: ";
			std::cin >> filename;
			rosbag::Bag bag;
			ros::spinOnce();
			std::string path = ros::package::getPath("moveit_utils");
			bag.open(path + "/positions/" + filename + ".bag", rosbag::bagmode::Write);
			bag.write("joint_states", ros::Time::now(), js_cur);
			bag.write("tool_position", ros::Time::now(), pose_current);
			bag.close();
			ROS_INFO("Trajectory bag written successfully in the %s/positions/ directory", path.c_str());

		}
		else if(in == '2'){
			rosbag::Bag bag;
			std::string path = ros::package::getPath("moveit_utils");
			std::string filename;
			std::cout << "Please enter a filename for the trajectory: ";
			std::cin >> filename;

			bag.open(path + "/positions/" + filename + ".bag", rosbag::bagmode::Read);
			
			rosbag::View view(bag, rosbag::TopicQuery("joint_states"));

			sensor_msgs::JointState temp;
			geometry_msgs::PoseStamped ps_temp;
			BOOST_FOREACH(rosbag::MessageInstance const m, view){
				sensor_msgs::JointState::ConstPtr js = m.instantiate<sensor_msgs::JointState>();
				if(js != NULL){
					temp = * js;
					ROS_INFO("Loaded bag file successfully: here is j1: %f, j2: %f, j3: %f", temp.position[0],
						temp.position[1], temp.position[2]);
				}
				geometry_msgs::PoseStamped::ConstPtr ps = m.instantiate<geometry_msgs::PoseStamped>();
				if(ps != NULL){
					ps_temp = *ps;
					ROS_INFO("Loaded bag file successfully: here is x: %f, y: %f, z: %f", ps_temp.pose.position.x, ps_temp.pose.position.y, ps_temp.pose.position.z);
					ROS_INFO("Orientation: x: %f, y: %f, z: %f, w: %f", ps_temp.pose.orientation.x, ps_temp.pose.orientation.y, ps_temp.pose.orientation.z, ps_temp.pose.orientation.w);	
				}
			}	
			
			rosbag::View view2(bag, rosbag::TopicQuery("tool_position"));
			BOOST_FOREACH(rosbag::MessageInstance const m, view2){
				geometry_msgs::PoseStamped::ConstPtr ps = m.instantiate<geometry_msgs::PoseStamped>();
				if(ps != NULL){
					ps_temp = *ps;
					ROS_INFO("Loaded bag file successfully: here is x: %f, y: %f, z: %f", ps_temp.pose.position.x, ps_temp.pose.position.y, ps_temp.pose.position.z);
					ROS_INFO("Orientation: x: %f, y: %f, z: %f, w: %f", ps_temp.pose.orientation.x, ps_temp.pose.orientation.y, ps_temp.pose.orientation.z, ps_temp.pose.orientation.w);	
				}
			}	
			bag.close();
			
		}
	}
	ros::shutdown();
	return 0;
}
