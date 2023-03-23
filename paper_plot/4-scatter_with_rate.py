#! coding=utf-8

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpathes
from mpl_toolkits import mplot3d
import os, io
import pickle
import json5
from plot_tools import get_line_style, set_size, svg_to_emf

# 全局美化格式
with io.open("params.json", "r", encoding="utf-8") as fp:
    params = json5.load(fp)
print(params)
plt.rcParams.update(params)

# 载入线型
style_dict = get_line_style()

# plt.figure(1)
figsize=set_size(83, fraction=1.2)
fig, ax = plt.subplots(1, 1, figsize=figsize)
f = open(os.path.join("../datas","datas_10Hz.pkl"), 'r')
datas = pickle.load(f)
f.close()
datas.sort(key=lambda x: x["min_distance"])

raw = [d["min_distance"] for d in datas]
CEP_10 = raw[(len(raw)+1)//2-1]
print("CEP_10Hz: {}".format(CEP_10))

pts = []
for d in datas:
    for i in range(len(d["sphere_traj"])):
        dlt_pos = np.array(d["mav_traj"][i]) - np.array(d["sphere_traj"][i])
        if abs(np.linalg.norm(dlt_pos) - d["min_distance"]) < 1e-5:
            pts.append(dlt_pos)
            break

plt.scatter([p[0] for p in pts], [p[2] for p in pts])
circle = mpathes.Circle([0,0], CEP_10, fill=False)
ax.add_patch(circle)

new_ticks = np.linspace(-1, 1, 5)
plt.xticks(new_ticks)
plt.yticks(new_ticks)
plt.axis("square")
plt.axis([-1, 1, -1, 1])
fig.savefig("../output/scatter_with_rate_10Hz.svg", format='svg', bbox_inches='tight')
svg_to_emf("../output/scatter_with_rate_10Hz.svg")
plt.show()


# plt.figure(2)
figsize=set_size(83, fraction=1.2)
fig, ax = plt.subplots(1, 1, figsize=figsize)
f = open(os.path.join("../datas","datas_30Hz.pkl"), 'r')
datas = pickle.load(f)
f.close()
datas.sort(key=lambda x: x["min_distance"])

raw = [d["min_distance"] for d in datas]
CEP_30 = raw[(len(raw)+1)//2-1]
print("CEP_30Hz: {}".format(CEP_30))

pts = []
for d in datas:
    for i in range(len(d["sphere_traj"])):
        dlt_pos = np.array(d["mav_traj"][i]) - np.array(d["sphere_traj"][i])
        if abs(np.linalg.norm(dlt_pos) - d["min_distance"]) < 1e-5:
            pts.append(dlt_pos)
            break

plt.scatter([p[0] for p in pts], [p[2] for p in pts])
circle = mpathes.Circle([0,0], CEP_30, fill=False)
ax.add_patch(circle)

new_ticks = np.linspace(-1, 1, 5)
plt.xticks(new_ticks)
plt.yticks(new_ticks)
plt.axis("square")
plt.axis([-1, 1, -1, 1])
fig.savefig("../output/scatter_with_rate_30Hz.svg", format='svg', bbox_inches='tight')
svg_to_emf("../output/scatter_with_rate_30Hz.svg")
plt.show()


# plt.figure(3)
figsize=set_size(83, fraction=1.2)
fig, ax = plt.subplots(1, 1, figsize=figsize)
f = open(os.path.join("../datas","datas_50Hz.pkl"), 'r')
datas = pickle.load(f)
f.close()
datas.sort(key=lambda x: x["min_distance"])

raw = [d["min_distance"] for d in datas]
CEP_50 = raw[(len(raw)+1)//2-1]
print("CEP_50Hz: {}".format(CEP_50))

pts = []
for d in datas:
    for i in range(len(d["sphere_traj"])):
        dlt_pos = np.array(d["mav_traj"][i]) - np.array(d["sphere_traj"][i])
        if abs(np.linalg.norm(dlt_pos) - d["min_distance"]) < 1e-5:
            pts.append(dlt_pos)
            break

plt.scatter([p[0] for p in pts], [p[2] for p in pts])
circle = mpathes.Circle([0,0], CEP_50, fill=False)
ax.add_patch(circle)

new_ticks = np.linspace(-1, 1, 5)
plt.xticks(new_ticks)
plt.yticks(new_ticks)
plt.axis("square")
plt.axis([-1, 1, -1, 1])
fig.savefig("../output/scatter_with_rate_50Hz.svg", format='svg', bbox_inches='tight')
svg_to_emf("../output/scatter_with_rate_50Hz.svg")
plt.show()

