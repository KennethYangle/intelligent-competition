import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpathes
import os
import pickle

np.random.seed(777)

f = open(os.path.join(os.path.expanduser('~'),"Rfly_Attack/src","CEP_with_ekf.pkl"), 'r')
data_deal = pickle.load(f)
f.close()

fig= plt.figure(1)
ax = fig.add_subplot(111)
raw = data_deal["delay-ekf"]
CEP_ekf = raw[(len(raw)+1)//2-1]
print("CEP_ekf: {}".format(CEP_ekf))
data = {"x": [], "y": []}
for d in data_deal["delay-ekf"]:
    theta = np.random.uniform(0, 2*np.pi)
    data["x"].append(d*np.cos(theta))
    data["y"].append(d*np.sin(theta))
plt.scatter(data["x"], data["y"])
circle = mpathes.Circle([0,0], CEP_ekf, fill=False)
ax.add_patch(circle)
plt.axis("square")
plt.axis([-0.4, 0.4, -0.4, 0.4])
plt.savefig("scatter_with_ekf.png", dpi=1200)
plt.show()
