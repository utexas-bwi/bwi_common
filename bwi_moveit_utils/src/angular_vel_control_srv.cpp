#include <ros/ros.h>
#include <signal.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <cstdlib>
#include <std_msgs/String.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float32.h>

//actions
#include <actionlib/client/simple_action_client.h>
#include "kinova_msgs/SetFingersPositionAction.h"
#include "kinova_msgs/ArmPoseAction.h"
#include "kinova_msgs/JointAngles.h"
#include "kinova_msgs/JointVelocity.h"

#include "bwi_moveit_utils/AngularVelCtrl.h"

#include <tf/tf.h>

#define PI 3.14159265
#define RAD_TO_DEG 57.2957795

#define TOLERANCE_RADIANS (0.01125*PI)
#define STALL_TIME_MULTIPLIER 1.35 //if we are still moving for 1.35 x the expected time, then stop, 


#define MAX_VELOCITY_RADIANS (2*0.075*PI)

using namespace std;

sensor_msgs::JointState current_state;
sensor_msgs::JointState target_joint_state;
sensor_msgs::JointState starting_joint_state;
bool joints_received = false;

ros::Publisher j_vel_pub;


//Joint state cb
void joint_state_cb (const sensor_msgs::JointStateConstPtr& input) {
	current_state = *input;
	joints_received = true;
	//ROS_INFO("Angles heard.");
}

void waitForJointAngles(){
	ros::Rate r(10);
	joints_received = false;
	while (!joints_received){
		ros::spinOnce();
		r.sleep();
	}
}


kinova_msgs::JointVelocity toJacoJointVelocityMsg(std::vector<float> goal_vector){
	kinova_msgs::JointVelocity jv_goal;
	
	/*jv_goal.joint1 = -180/PI*goal_vector[0];
	jv_goal.joint2 = 180/PI*goal_vector[1];
	jv_goal.joint3 = -180*goal_vector[2];
	jv_goal.joint4 = -180*goal_vector[3];
	jv_goal.joint5 = -180*goal_vector[4];
	jv_goal.joint6 = -180*goal_vector[5];*/
	
	jv_goal.joint1 = -RAD_TO_DEG*goal_vector[0];
	jv_goal.joint2 = RAD_TO_DEG*goal_vector[1];
	jv_goal.joint3 = -RAD_TO_DEG*goal_vector[2];
	jv_goal.joint4 = -RAD_TO_DEG*goal_vector[3];
	jv_goal.joint5 = -RAD_TO_DEG*goal_vector[4];
	jv_goal.joint6 = -RAD_TO_DEG*goal_vector[5];
	
	return jv_goal;
}
	
double distanceToTravel(double current_position, double target_position, int joint_id){
	if (joint_id != 1 && joint_id != 2){ //joints 1 and 2 (starting at 0) have hard limits
		//decide which direction to move in for each joint (either positive or negative)
		if (target_position > current_position){
			if ( target_position - current_position <
						current_position + 2*PI - target_position) {
				//go along the positive direction
					
				return target_position -current_position;
			}
			else {
				return 2*PI - target_position + current_position;
			}
				
		}
		else {
			if ( current_position - target_position > 
					target_position + 2*PI - current_position){
				
				return target_position + 2*PI - current_position;
			}
			else {
				
				return current_position - target_position;
				
			}
		}
	}
	else {
		return target_position - current_position;
	}
}

bool service_cb(bwi_moveit_utils::AngularVelCtrl::Request &req, bwi_moveit_utils::AngularVelCtrl::Response &res){
	//get joint positions
	target_joint_state = req.state;
	
	//get joint positions
	waitForJointAngles();
	starting_joint_state = current_state;
	
	ROS_INFO("Starting state:");
	ROS_INFO("%f, %f, %f, %f, %f, %f",starting_joint_state.position[0],starting_joint_state.position[1],starting_joint_state.position[2],
				starting_joint_state.position[3],starting_joint_state.position[4],starting_joint_state.position[5]);
	
	ROS_INFO("Target state:");
	ROS_INFO("%f, %f, %f, %f, %f, %f",target_joint_state.position[0],target_joint_state.position[1],target_joint_state.position[2],
				target_joint_state.position[3],target_joint_state.position[4],target_joint_state.position[5]);
	//ROS_INFO_STREAM(target_joint_state);
	
	
	//ROS_INFO_STREAM(starting_joint_state);
	
	//now, go to target from joint
	ros::Rate r(40);
	
	
	std::vector<float> j_vel_goal;
	j_vel_goal.resize(6);
	
	std::vector<float> j_vel_directions;
	std::vector<float> distance_to_travel;
	j_vel_directions.resize(6);
	distance_to_travel.resize(6);
	
	for (unsigned int i = 0; i < 6; i ++){
		
		if (i != 1 && i != 2){ //joints 1 and 2 (starting at 0) have hard limits
			//decide which direction to move in for each joint (either positive or negative)
			if (target_joint_state.position[i] > starting_joint_state.position[i]){
				if ( target_joint_state.position[i] - starting_joint_state.position[i] <
						starting_joint_state.position[i] + 2*PI - target_joint_state.position[i]) {
					//go along the positive direction
					j_vel_directions[i] = 1.0;
					distance_to_travel[i] = target_joint_state.position[i] - starting_joint_state.position[i];
				}
				else {
					j_vel_directions[i] = -1.0;
					distance_to_travel[i] = 2*PI - target_joint_state.position[i] + starting_joint_state.position[i];
				}
				
			}
			else {
				if ( starting_joint_state.position[i] - target_joint_state.position[i] > 
						target_joint_state.position[i] + 2*PI - starting_joint_state.position[i]){
					j_vel_directions[i] = 1.0;
					distance_to_travel[i] = target_joint_state.position[i] + 2*PI - starting_joint_state.position[i];
				}
				else {
					j_vel_directions[i] = -1.0;
					distance_to_travel[i] = starting_joint_state.position[i] - target_joint_state.position[i];
				
				}
			}
		}
		else {
			distance_to_travel[i] = target_joint_state.position[i] - starting_joint_state.position[i];
			if (distance_to_travel[i] >= 0)
				j_vel_directions[i] = 1.0;
			else
				j_vel_directions[i] = -1.0;
		}
		//float d_i = target_joint_state.position[i]-starting_joint_state.position[i];
		
	}
	
	ROS_INFO("Directions: %f, %f, %f, %f, %f, %f",j_vel_directions[0],j_vel_directions[1],j_vel_directions[2],
				j_vel_directions[3],j_vel_directions[4],j_vel_directions[5]);
	ROS_INFO("distance to travel: %f, %f, %f, %f, %f, %f",distance_to_travel[0],distance_to_travel[1],distance_to_travel[2],
				distance_to_travel[3],distance_to_travel[4],distance_to_travel[5]);
	
	
	//find which joint has to travel the most; that joint travels at MAX_VELOCITY_RADIANS
	int fastest_joint = -1;
	double d_temp = -1.0;
	for (unsigned int i = 0; i < 6; i ++){
		if (fabs(distance_to_travel[i]) > d_temp){
			d_temp = fabs(distance_to_travel[i]);
			fastest_joint = i;
		}
	}
	
	double expected_duration = (double) d_temp / (double) MAX_VELOCITY_RADIANS;
	ROS_INFO("[%f / %f = %f] Expected duration of movement",d_temp,MAX_VELOCITY_RADIANS,expected_duration);

	//next, decide the speed
	for (unsigned int i = 0; i < 6; i ++){
		if (fabs(distance_to_travel[i]) > TOLERANCE_RADIANS){
			double v_i = (fabs(distance_to_travel[i])/d_temp)*MAX_VELOCITY_RADIANS;
			j_vel_goal[i] = v_i*j_vel_directions[i];
			//j_vel_goal[i] = MAX_VELOCITY_RADIANS*j_vel_directions[i];
		}
		else 
			j_vel_goal[i]=0.0;
	}
	
	ROS_INFO("j_vel: %f, %f, %f, %f, %f, %f",j_vel_goal[0],j_vel_goal[1],j_vel_goal[2],
				j_vel_goal[3],j_vel_goal[4],j_vel_goal[5]);
	
	
	kinova_msgs::JointVelocity jv_goal;
	double sum = 0.0;
	
	double t_start_sec =ros::Time::now().toSec();
	double t_current;
	
	std::vector<float> remaining_distances;
	remaining_distances.resize(6);
	std::vector<float> remaining_distances_last = distance_to_travel;
	bool stalled = false;
	
	//starti looping
	while (ros::ok()){
		ros::spinOnce();
		
		
		sum = 0.0;
		//check if any of the joints have reached their targets
		for (unsigned int i = 0; i < 6; i ++){
			if (fabs(current_state.position[i]- target_joint_state.position[i]) < TOLERANCE_RADIANS){	
				j_vel_goal[i]=0.0;
			}	
			sum+=fabs(j_vel_goal[i]);
		}
		
		//if we're done, just break
		if (sum < 0.01)
			break;
		
		//check if the distance to the target is growing -- this 
		//can be a result of a colision or over-shooting the target
		for (unsigned int i = 0; i < 6; i ++){
			remaining_distances[i]=distanceToTravel(current_state.position[i],target_joint_state.position[i],i);
			
			//check if it's growing
			if (fabs(remaining_distances[i]) > fabs(remaining_distances_last[i]) + TOLERANCE_RADIANS
				&& j_vel_goal[i] != 0.0){
				ROS_INFO("Something must be in the way (joint %i, %f, %f) or the joint has over-shot its target"
							,i,remaining_distances[i],remaining_distances_last[i]);
				
				//stalled = true;
			}
			else {
				remaining_distances_last[i]=remaining_distances[i];
			}
		}
		
		if (stalled)
			break;
		
		
		//ROS_INFO("j vel sum = %f",sum);
		
		//publish joint commands
		/*ROS_INFO("current j_vel: %f, %f, %f, %f, %f, %f",current_state.velocity[0],current_state.velocity[1],
				current_state.velocity[2],
				current_state.velocity[3],current_state.velocity[4],current_state.velocity[5]);*/
		//check how much left the joint has to travel
			
		
	
		ROS_INFO("publishing j_vel: %f, %f, %f, %f, %f, %f",j_vel_goal[0],j_vel_goal[1],j_vel_goal[2],
				j_vel_goal[3],j_vel_goal[4],j_vel_goal[5]);
	
	
		//publish the goal
		jv_goal = toJacoJointVelocityMsg(j_vel_goal);
		j_vel_pub.publish(jv_goal);
		
		//sleep
		r.sleep();
		
		//check if too much time has passed (e.g., 1.5 times expected_duration) and end as well
		t_current = ros::Time::now().toSec();
		
		if (t_current - t_start_sec > expected_duration*STALL_TIME_MULTIPLIER){
			ROS_INFO("Timeout!");
			break;
		}
	}
	
	double t_end_sec =ros::Time::now().toSec();
	double actual_duration = t_end_sec-t_start_sec;
	ROS_INFO("Movement took %f seconds, expected was %f ",actual_duration,expected_duration);
	
	
	//compute final error
	std::vector<float> j_pos_error;
	j_pos_error.resize(6);
	
	for (unsigned int i = 0; i < 6; i ++){
		j_pos_error[i] = fabs(target_joint_state.position[i]-current_state.position[i]);
		ROS_INFO("Joint %i\tTarget: %f\tCurrent: %f\tError: %f",i,target_joint_state.position[i],current_state.position[i],j_pos_error[i]);
	}
	
	res.success = true;
	return res.success;
}
int main(int argc, char **argv) {
	// Intialize ROS with this node name
	ros::init(argc, argv, "joint_vel_test");

	ros::NodeHandle n;

	//create subscriber to joint angles
	ros::Subscriber sub_angles = n.subscribe ("/m1n6s200_driver/out/joint_state", 1, joint_state_cb);
	
	//publisher for velocity commands
    j_vel_pub = n.advertise<kinova_msgs::JointVelocity>("/m1n6s200_driver/in/joint_velocity", 8);
    
    ros::ServiceServer srv = n.advertiseService("angular_vel_control", service_cb);

	ros::spin();
	
	return 0;
}
