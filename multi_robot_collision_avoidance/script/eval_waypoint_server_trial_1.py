#!/usr/bin/env python3.6
import numpy as np

import os
import pandas as pd

import torch
import gpytorch

from multi_robot_collision_avoidance.srv import EvalWaypoint,EvalWaypointResponse
from multi_robot_collision_avoidance.MapHack import MapHack
from geometry_msgs.msg import Pose

import rospy
import rospkg

# gpytorch GP model
class ExactGPModel(gpytorch.models.ExactGP):
	def __init__(self, train_x, train_y, likelihood):
		super(ExactGPModel, self).__init__(train_x, train_y, likelihood)
		self.mean_module = gpytorch.means.ConstantMean()
		self.covar_module = gpytorch.kernels.ScaleKernel(gpytorch.kernels.RBFKernel())

	def forward(self, x):
		mean_x = self.mean_module(x)
		covar_x = self.covar_module(x)
		return gpytorch.distributions.MultivariateNormal(mean_x, covar_x)
def load_data(filename):
	"""Load pre-build data from pkg/data/$filename"""
	rospack = rospkg.RosPack()
	pkgPath = rospack.get_path('multi_robot_collision_avoidance')
	pkgPath = os.path.join(pkgPath, 'script/data')
	filepath = os.path.join(pkgPath, filename)
	data = pd.read_csv(filepath, sep=',', header=[0], engine='python')

	features = data[['d1','d2','d3','d4']]
	reward = data['reward']
	return features, reward

def pose2arr(pose):
	return np.array([pose.position.x,pose.position.y])

def handle_eval_waypoint(req):
	st = rospy.Time.now()
	model = train_GP_model("8m_contextual.txt", likelihood)
	model.eval()
	likelihood.eval()
	print("training took {:.3f}s".format((rospy.Time.now()-st).to_sec() ))
	coward_pose = pose2arr(req.chicken_pose)
	bold_pose  = pose2arr(req.bold_pose)
	# Sample vaild waypoints
	st = rospy.Time.now()
	bold_pix  = mh.pose2pix(bold_pose)
	coward_pix = mh.pose2pix(coward_pose)

	samples = mh.sampling(coward_pix, [[-80,-80],[80,80]], 1000)
	waypoints = mh.filter(samples)
	print("sampling took {:.3f}s".format((rospy.Time.now() - st).to_sec()))
	st = rospy.Time.now()
	# Use position information from request; chicken_pos, bold_pos
	# to measure the features (4 distances) from given map
	features = mh.extractFeature(coward_pix, bold_pix, waypoints)
	print("feature extraction took {:.3f}s".format((rospy.Time.now() - st).to_sec()))

	st = rospy.Time.now()
	with torch.no_grad(), gpytorch.settings.fast_pred_var():
		expected_reward = likelihood(model(torch.from_numpy(features).type(torch.float)))
	expected_reward = expected_reward.mean.numpy()
	print("GP model took {:.3f}s".format((rospy.Time.now() - st).to_sec()))

	if(np.random.rand()<0.10):
		best_idx = np.random.randint(features.shape[0])
	else:
		best_reward = np.max(expected_reward)
		best_idx = np.argwhere(expected_reward == best_reward).reshape(-1)
		best_idx = best_idx[0]
		print("Best expected rewards: {:.3f}".format(best_reward))
	resp = EvalWaypointResponse()
	print(best_idx)
	print(waypoints[best_idx])
	x,y = mh.pix2pose(waypoints[best_idx])
	with open("/home/jinsoo/catkin_ws/src/multi_robot_collision_avoidance/script/data/8m_contextual.txt", 'a') as f:
		f.write("{:.3f},{:.3f},".format(x,y))
		f.write("{:.3f},{:.3f},{:.3f},{:.3f},".format(features[best_idx,0],features[best_idx,1],features[best_idx,2],features[best_idx,3]))
		f.write("{},".format(expected_reward[best_idx]))
	print("{}->[{:.3f},{:.3f}]".format(coward_pose, x,y))

	wpos = Pose()
	wpos.position.x = x
	wpos.position.y = y

	resp.waypoint = wpos
	#resp.E_reward = best_reward
	return resp

def train_GP_model(data_path, likelihood):
	train_x, train_y = load_data("8m_contextual.txt")
	train_y = np.clip(train_y, -100, 0)
	train_x = torch.from_numpy(train_x.to_numpy()).type(torch.float)
	train_y = torch.from_numpy(train_y.to_numpy()).type(torch.float)

	model = ExactGPModel(train_x, train_y, likelihood)
	state_dict = torch.load("/home/jinsoo/catkin_ws/src/multi_robot_collision_avoidance/script/model/ExactGPModel_0_prior_100_epoch.pth")
	model.load_state_dict(state_dict)
	return model

def eval_waypoint_server():
	rospy.init_node('eval_waypoint_server')

	global mh
	global likelihood
	global model

	mh = MapHack(True)
	st = rospy.Time.now()
	likelihood = gpytorch.likelihoods.GaussianLikelihood()
	model = train_GP_model("8m_contextual.txt", likelihood)

    # Activate evaluation mode and
    # Perform dummy evaluation to intialize torch network.
	model.eval()
	likelihood.eval()

	with torch.no_grad(), gpytorch.settings.fast_pred_var():
		likelihood(model( torch.from_numpy(np.random.rand(1,4)).type(torch.float) ))

	s = rospy.Service('eval_waypoint', EvalWaypoint, handle_eval_waypoint)
	print("Ready to evaluate waypoint in map.")
	print("Training took {:.3f}s".format((rospy.Time.now() - st).to_sec()))
	rospy.spin()

if __name__ == "__main__":
    eval_waypoint_server()
