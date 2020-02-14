"""
Read Brian's experimental data from file
============Data Structure==============
waypoint_x waypoint_y d1 d2 d3 d4
Reward
Repeat above two lines.
========================================
d1 = distance from waypoint to chicken bot
d2 = distance from waypoint to bold    bot
d3 = distance from waypoint to left   wall
d4 = distance from waypoint to right  wall
in d3 and d4, left and right is defined based on d1-d2 line, L.
if np.cross(L, wall_pt)>0 then it is left wall
otherwise, it is right wall.
Detailed algorithm is implemented in MapHack class.

=================functions=================
readFromFiles(data_dir, filenames, feature_dim)
excludeFailure(features, reward, fail_label)
findMean(features)
findStd(features)
featureNormalize(features, feature_means, feature_std)
rewardClip(reward, clip)
===========================================
"""
import numpy as np
import os

def readFromFiles(data_dir, filenames, feature_dim):
    feature = np.ndarray(shape=(feature_dim, 0))
    rewards = np.array([])
    
    for filename in filenames:
        data_path = os.path.join(data_dir, filename)
        if not os.path.exists(data_path):
            print("{} NOT FOUND".format(data_path))
        with open(data_path) as f:
            philine = f.readline().strip()
            rewardline = f.readline().strip()
            while(rewardline):
                phi = np.fromstring(philine, dtype=float, sep=" ").reshape(feature_dim,1)
                reward = np.fromstring(rewardline, dtype=float, sep=' ')

                feature = np.hstack([feature, phi])
                rewards = np.append(rewards, reward)

                philine = f.readline().strip()
                rewardline = f.readline().strip()
    return feature.T, rewards

def excludeFailure(features, reward, fail_label):
    fail_idx = np.argwhere(np.isin(reward, fail_label)).flatten()
    mask = np.ones(reward.shape, dtype=bool)
    mask[fail_idx] = False
    
    return features[mask], reward[mask]

def findMean(features):
    return np.mean(features, axis=0)

def findStd(features):
    return np.std(features, axis=0)

def featureNormalize(features, feature_means, feature_std):
    # Normalize features
    return (features - feature_means)/feature_std

def rewardClip(reward, clip):
    """Clip the reward so that (clip[0] <= reward <= clip[1])"""
    
    return np.clip(reward, clip[0], clip[1])
