import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpathes
import os
import pickle

np.random.seed(777)

f = open(os.path.join(os.path.expanduser('~'),"Rfly_Attack/src","CEP_with_rate.pkl"), 'r')
data_deal = pickle.load(f)
f.close()

fig= plt.figure(1)
ax = fig.add_subplot(111)
raw = data_deal["10Hz"]
CEP_10 = raw[(len(raw)+1)//2-1]
print("CEP_10: {}".format(CEP_10))
data = {"x": [], "y": []}
for d in data_deal["10Hz"]:
    theta = np.random.uniform(0, 2*np.pi)
    data["x"].append(d*np.cos(theta))
    data["y"].append(d*np.sin(theta))
plt.scatter(data["x"], data["y"])
circle = mpathes.Circle([0,0], CEP_10, fill=False)
ax.add_patch(circle)
plt.axis("square")
plt.axis([-0.4, 0.4, -0.4, 0.4])
plt.savefig("scatter_with_rate_10Hz.png", dpi=1200)
plt.show()

fig= plt.figure(2)
ax = fig.add_subplot(111)
raw = data_deal["30Hz"]
CEP_30 = raw[(len(raw)+1)//2-1]
print("CEP_30: {}".format(CEP_30))
data = {"x": [], "y": []}
for d in data_deal["30Hz"]:
    theta = np.random.uniform(0, 2*np.pi)
    data["x"].append(d*np.cos(theta))
    data["y"].append(d*np.sin(theta))
plt.scatter(data["x"], data["y"])
circle = mpathes.Circle([0,0], CEP_30, fill=False)
ax.add_patch(circle)
plt.axis("square")
plt.axis([-0.4, 0.4, -0.4, 0.4])
plt.savefig("scatter_with_rate_30Hz.png", dpi=1200)
plt.show()

fig= plt.figure(3)
ax = fig.add_subplot(111)
raw = data_deal["50Hz"]
CEP_50 = raw[(len(raw)+1)//2-1]
print("CEP_50: {}".format(CEP_50))
data = {"x": [], "y": []}
for d in data_deal["50Hz"]:
    theta = np.random.uniform(0, 2*np.pi)
    data["x"].append(d*np.cos(theta))
    data["y"].append(d*np.sin(theta))
plt.scatter(data["x"], data["y"])
circle = mpathes.Circle([0,0], CEP_50, fill=False)
ax.add_patch(circle)
plt.axis("square")
plt.axis([-0.4, 0.4, -0.4, 0.4])
plt.savefig("scatter_with_rate_50Hz.png", dpi=1200)
plt.show()
