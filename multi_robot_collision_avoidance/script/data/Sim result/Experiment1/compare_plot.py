import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

filenames = ["social_navigation", "vanilla", 'chicken', "proposed"]
if __name__ == "__main__":
  p_collision = np.empty(shape=(4))
  p_turnaround = np.empty(shape=(4))
  avg_reward = np.empty(shape=(4))

  for i, filename in enumerate(filenames):
    df = pd.read_csv(f"{filename}.txt")
    if(filename == "proposed"):
        df = df[-100:]
    collision = (df.reward <= -2000)
    p_collision[i] = np.sum(collision) / df.shape[0]

    data = df[~collision]
    turnaround = (data.reward <= -70)
    p_turnaround = np.sum(turnaround) / df.shape[0]

    avg_reward[i] = df.reward.mean()

  # Draw reward plot
  plt.figure(figsize=(9,6))
  plt.bar(filenames, p_collision * 100)
  plt.xlabel("Policy")
  plt.ylabel("reward")
  plt.title("Collision rate of policies")
  plt.savefig("percent_collision.png", dpi=300)

  plt.figure(figsize=(9,6))
  plt.bar(filenames, avg_reward)
  plt.xlabel("Policy")
  plt.ylabel("%")
  plt.title("Average Reward of last 100 episodes")
  plt.savefig("avg_reward.png", dpi=300)
