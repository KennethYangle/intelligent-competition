import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import os
import pickle

# fontsize = 16
data = []
# 50Hz
f = open(os.path.join(os.path.expanduser('~'),"Rfly_Attack/src","CEP_raw_50Hz.pkl"), 'r')
CEP_raw = pickle.load(f)
f.close()
CEP_raw.sort()
data.append(CEP_raw[:50])
# 30Hz
f = open(os.path.join(os.path.expanduser('~'),"Rfly_Attack/src","CEP_raw_30Hz.pkl"), 'r')
CEP_raw = pickle.load(f)
f.close()
CEP_raw.sort()
data.append(CEP_raw[:50])
# 10Hz
f = open(os.path.join(os.path.expanduser('~'),"Rfly_Attack/src","CEP_raw_10Hz.pkl"), 'r')
CEP_raw = pickle.load(f)
f.close()
CEP_raw.sort()
data.append(CEP_raw[:50])

labels = [10, 30, 50]
ave = [np.average(a) for a in data]
print(ave)

fig = plt.figure()
ax = fig.add_subplot(111)
ax.boxplot(data, labels=labels, showmeans=True, patch_artist=True)
ax.grid(linestyle="--", alpha=0.3)
# ax.set_xlabel("The number of WLs", size=fontsize)
# ax.set_ylabel("Coverage rate (%)", size=fontsize)
# ax.set_ylim(78, 102)


plt.tight_layout()
plt.savefig("CEP_with_rate.png", dpi=1200)
plt.show()


data_deal = {"10Hz": data[0], "30Hz": data[1], "50Hz": data[2]}
f = open(os.path.join(os.path.expanduser('~'),"Rfly_Attack/src","CEP_with_rate.pkl"), 'w')
pickle.dump(data_deal, f)
f.close()