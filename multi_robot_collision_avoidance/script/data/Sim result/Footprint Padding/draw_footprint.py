#!/usr/bin/python3

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

start = -0.04
end   = 0.03
delta = 0.01

def readData(i):
  '''
    Return percentage of collision, turn-around, Reward of successful pass by.
  '''
  df = pd.read_csv("footprint_padding_{:.2f}.txt".format(i), engine='python', sep=',')
  n = df.shape[0]
  idx = (df.reward<-1000)
  num_collision = np.sum(idx)
  df = df[~idx]
  idx = (df.reward<-100)
  num_turn = np.sum(idx)
  df = df[~idx]
  return num_collision/n, num_turn/n, np.mean(df.reward)

if __name__ == "__main__":
  col, turn, reward = [], [], []
  for i in np.arange(start, end+delta, delta):
    c,t,r = readData(i)
    col.append(c)
    turn.append(t)
    reward.append(r)

  x = np.arange(start,end+delta, delta)
  plt.plot(x, np.array(col) * 100, label='collision')
  plt.plot(x, np.array(turn) * 100, label='turn around')

  plt.xlabel("footprint padding")
  plt.ylabel("%")
  plt.legend()
  plt.title("Social Navigation\nPercent Failure (collision, turnaround)")
  plt.savefig("Social_Navigation_Stats.png", dpi=300)
  plt.close()

  plt.plot(x, reward)
  plt.xlabel("footprint padding")
  plt.ylabel("reward")
  plt.title("Social Navigation\nAvg reward of successful pass-by")
  plt.savefig("Social_Navigation_success_case_reward.png", dpi=300)
