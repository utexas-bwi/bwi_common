#!/usr/bin/env python3.7
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
	global pkgPath
	pkgPath = rospack.get_path('multi_robot_collision_avoidance')
	pkgPath = os.path.join(pkgPath, 'script')
	dataPath = os.path.join(pkgPath, 'data')
	filepath = os.path.join(dataPath, filename)
	data = pd.read_csv(filepath, sep=',', header=[0], engine='python')
	data = data[:-1]

	features = data[['d1','d2','d3','d4']]
	reward = data['reward']
	return features.to_numpy(), reward.to_numpy()

def pose2arr(pose):
	return np.array([pose.position.x,pose.position.y])

def handle_eval_waypoint(req):
	coward_pose = pose2arr(req.chicken_pose)
	bold_pose  = pose2arr(req.bold_pose)
	# Sample vaild waypoints
	#st = rospy.Time.now()
	bold_pix  = mh.pose2pix(bold_pose)
	coward_pix = mh.pose2pix(coward_pose)

	samples = mh.sampling(coward_pix, [[-80,-80],[80,80]], 1000)
	waypoints = mh.filter(samples)
	#print("sampling took {:.3f}s".format((rospy.Time.now() - st).to_sec()))
	#st = rospy.Time.now()
	# Use position information from request; chicken_pos, bold_pos
	# to measure the features (4 distances) from given map
	features = mh.extractFeature(coward_pix, bold_pix, waypoints)
	#print("feature extraction took {:.3f}s".format((rospy.Time.now() - st).to_sec()))

	#st = rospy.Time.now()
	with torch.no_grad(), gpytorch.settings.fast_pred_var():
		expected_reward = likelihood(model(torch.from_numpy(features).type(torch.float)))
	expected_reward = expected_reward.mean.numpy()
	#print("GP model took {:.3f}s".format((rospy.Time.now() - st).to_sec()))

	if(np.random.rand()<epsilon):
		print(epsilon)
		mode = "explore"
		best_idx = np.random.randint(features.shape[0])
	else:
		mode = "exploit"
		best_reward = np.max(expected_reward)
		best_idx = np.argwhere(expected_reward == best_reward).reshape(-1)
		print(best_idx)
		best_idx = best_idx[0]
		#print("Best expected rewards: {:.3f}".format(best_reward))
	resp = EvalWaypointResponse()
	#print(best_idx)
	#print(waypoints[best_idx])
	x,y = mh.pix2pose(waypoints[best_idx])
	with open("{}/data/8m_contextual.txt".format(pkgPath), 'a') as f:
		f.write("{},{:.3f},{:.3f},".format(mode,x,y))
		f.write("{:.3f},{:.3f},{:.3f},{:.3f},".format(features[best_idx,0],features[best_idx,1],features[best_idx,2],features[best_idx,3]))
		f.write("{},".format(expected_reward[best_idx]))
	print("{}->[{:.3f},{:.3f}]".format(coward_pose, x,y))

	global update
	update = True

	wpos = Pose()
	wpos.position.x = x
	wpos.position.y = y

	resp.waypoint = wpos
	#resp.E_reward = best_reward
	return resp

def train_GP_model(data_path, likelihood):
	train_x, train_y = load_data("8m_contextual.txt")
	train_y = np.clip(train_y, -100, 0)
	train_x = torch.from_numpy(train_x).type(torch.float)
	train_y = torch.from_numpy(train_y).type(torch.float)

	model = ExactGPModel(train_x, train_y, likelihood)
	state_dict = torch.load("{}/model/ExactGPModel_0_prior_100_epoch.pth".format(pkgPath))
	model.load_state_dict(state_dict)
	return model

def eval_waypoint_server():
	rospy.init_node('eval_waypoint_server')

	global mh
	global likelihood
	global model
	global update
	global epsilon
	update = True
	epsilon = 0.05

	mh = MapHack(True)
	likelihood = gpytorch.likelihoods.GaussianLikelihood()
	model = train_GP_model("8m_contextual.txt", likelihood)

	s = rospy.Service('eval_waypoint', EvalWaypoint, handle_eval_waypoint)
	print("Ready to evaluate waypoint in map.")

	r = rospy.Rate(10)
	while not rospy.is_shutdown():
		if(update == True):
			st = rospy.Time.now()
			model = train_GP_model("8m_contextual.txt", likelihood)
			# Activate evaluation mode and
			model.eval()
			likelihood.eval()
			print("training took {:.3f}s".format((rospy.Time.now()-st).to_sec() ))
			# Perform dummy evaluation to intialize torch network.
			with torch.no_grad(), gpytorch.settings.fast_pred_var():
				likelihood(model( torch.from_numpy(np.random.rand(1,4)).type(torch.float) ))
			update = False
		r.sleep()
	#s.shutdown()

if __name__ == "__main__":
    eval_waypoint_server()
