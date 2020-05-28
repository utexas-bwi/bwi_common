import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("8m_contextual.txt")
idx = (df.type=="exploit")
plt.plot(df[idx].reward.rolling(4).mean())
plt.plot(df[idx].E_reward.rolling(1).mean())
plt.savefig("rolling_4.png", dpi=300)
