import numpy as np
import matplotlib.pyplot as plt

# change filename to record file 
x, y = np.loadtxt('../data/record.txt', usecols = (1,2), unpack=True)

heatmap, xedges, yedges = np.histogram2d(x, y, bins=500)
extent = [xedges[0], xedges[-1], yedges[0], yedges[-1]]

plt.imshow(heatmap.T, extent=extent, origin='lower')
plt.show()
