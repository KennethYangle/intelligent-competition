# 运行
## 1. 仿真
修改`settings.json`中`"MODE": "Simulation"`

运行
`./simulation/shell/rfly-sitl.sh`
或者
`./simulation/shell/rfly-hitl.sh`

## 2. 实飞
修改`settings.json`中`"MODE": "RealFlight"`

运行
`./offboard_pkg/shell/all.sh`
或者
`./offboard_pkg/shell/all_high.sh`


# 画图

**图1-5在`simulation\statistics\`路径下运行。**

均使用`Python3`运行，保证多平台通用。Linux默认python2，Windows中先运行`conda activate py2_env`切换环境。

## 1. The trajectory of the interceptor and the target when the image measurement is filtered at 30Hz.
画50组实验的全体轨迹图，可以修改`datas_30Hz_ekf.pkl`为其他统计结果
```
python draw_trajectory.py
```
![](simulation\statistics\trajectory.png)

## 2. Boxplots of control error for EKF 
画30hz下有无EKF的误差对比箱线图
```
python CEP_with_ekf.py
```
![](simulation\statistics\CEP_with_ekf.png)

## 3. Boxplots of control error for different image frequencies 
画不同图像频率下误差对比箱线图
```
python CEP_with_rate.py
```
![](simulation\statistics\CEP_with_rate.png)

## 4. Error distribution at 10Hz、30Hz、50Hz 
画不同图像频率下撞击点散点图和圆概率误差
```
python scatter_with_rate.py
```
![](simulation\statistics\scatter_with_rate_10Hz.png)
CEP_10Hz: 0.466656341936

![](simulation\statistics\scatter_with_rate_30Hz.png)
CEP_30Hz: 0.470425914344

![](simulation\statistics\scatter_with_rate_50Hz.png)
CEP_50Hz: 0.4568409922

## 5. Error distribution with EKF 
画有EKF下撞击点散点图和圆概率误差
```
python scatter_with_ekf.py
```
![](simulation\statistics\scatter_with_ekf.png)
CEP_30Hz_ekf: 0.39116126392


**图6-11在`analyse\`路径下运行。**
## 6. Results for HITL simulation. (a) trajectory
画仿真中相对目标的全局坐标轨迹图
```
python plot_data.py ../datas/377504aa-b91c-11eb-9cde-000c29e163c9/main_node-3-stdout.log mav_pos -p -t "0 10"
```

## 7. Results for HITL simulation. (b) image coordinates
画仿真中原始和滤波后图像坐标随时间变化图
```
python plot_data.py ../datas/377504aa-b91c-11eb-9cde-000c29e163c9/ekf_node-2-stdout.log IMG_x ekf_x -t "10 37.5"
python plot_data.py ../datas/377504aa-b91c-11eb-9cde-000c29e163c9/ekf_node-2-stdout.log IMG_y ekf_y -t "10 37.5"
```

## 8. Results for HITL simulation. (c) local image coordinates
画仿真中原始和滤波后图像坐标随时间变化图局部放大图
```
python plot_data.py ../datas/377504aa-b91c-11eb-9cde-000c29e163c9/ekf_node-2-stdout.log IMG_y ekf_y -t "30.2 32.25"
```

## 9. Results for real flight experiments. (a) trajectory
画实飞中相对目标的全局坐标轨迹图
```
python plot_data.py ../datas/20210521_172502_sim.log mav_pos -t "51 61" -p
```

## 10. Results for HITL simulation. (b) image coordinates
画实飞中原始和滤波后图像坐标随时间变化图
```
python plot_data.py ../datas/20210521_172502_sim.log IMG_x ekf_x -t "2 68.4"
python plot_data.py ../datas/20210521_172502_sim.log IMG_y ekf_y -t "2 68.4"
```

## 11. Results for HITL simulation. (c) local image coordinates
画实飞中原始和滤波后图像坐标随时间变化图局部放大图
```
python plot_data.py ../datas/20210521_172502_sim.log IMG_y ekf_y -t "36.5 41"
python plot_data.py ../datas/20210521_172502_sim.log IMG_y ekf_y -t "62.5 68.4"
```
