# 一、运行
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


# 二、画图

**[new] 论文图均在`paper_plot\`文件夹下运行**

**图1-5在`simulation\statistics\`路径下运行。**

均使用`Python3`运行，保证多平台通用。Linux默认python2，Windows中先运行`conda activate py2_env`切换环境。

## 1. The trajectory of the interceptor and the target when the image measurement is filtered at 30Hz.
画50组实验的全体轨迹图，可以修改`datas_30Hz_ekf.pkl`为其他统计结果
```
# python draw_trajectory.py
python 1-draw_trajectory.py
```
![](output\trajectory.svg)

## 2. Boxplots of control error for EKF 
画30hz下有无EKF的误差对比箱线图
```
# python CEP_with_ekf.py
python 2-CEP_with_ekf.py
```
![](output\CEP_with_ekf.svg)

## 3. Boxplots of control error for different image frequencies 
画不同图像频率下误差对比箱线图
```
# python CEP_with_rate.py
python 3-CEP_with_rate.py
```
![](output\CEP_with_rate.svg)

## 4. Error distribution at 10Hz、30Hz、50Hz 
画不同图像频率下撞击点散点图和圆概率误差
```
# python scatter_with_rate.py
python 4-scatter_with_rate.py
```
![](output\scatter_with_rate_10Hz.svg)
CEP_10Hz: 0.466656341936

![](output\scatter_with_rate_30Hz.svg)
CEP_30Hz: 0.470425914344

![](output\scatter_with_rate_50Hz.svg)
CEP_50Hz: 0.4568409922

## 5. Error distribution with EKF 
画有EKF下撞击点散点图和圆概率误差
```
# python scatter_with_ekf.py
python 5-scatter_with_ekf.py
```
![](output\scatter_with_30Hz_ekf.svg)
CEP_30Hz_ekf: 0.39116126392


**图6-11在`analyse\`路径下运行。**
## 6. Results for HITL simulation. (a) trajectory
画仿真中相对目标的全局坐标轨迹图
```
# python plot_data.py ../datas/377504aa-b91c-11eb-9cde-000c29e163c9/main_node-3-stdout.log mav_pos -p -t "0 10"
python 6-plot_data.py ../datas/377504aa-b91c-11eb-9cde-000c29e163c9/main_node-3-stdout.log mav_pos -p -t "0 10"
```
![](output\plotxy-sim.svg)

## 7. Results for HITL simulation. (b) image coordinates
画仿真中原始和滤波后图像坐标随时间变化图
```
# python plot_data.py ../datas/377504aa-b91c-11eb-9cde-000c29e163c9/ekf_node-2-stdout.log IMG_x ekf_x IMG_y ekf_y -t "10 37.5" --subplot 2
python 7-plot_data.py ../datas/377504aa-b91c-11eb-9cde-000c29e163c9/ekf_node-2-stdout.log IMG_x ekf_x IMG_y ekf_y -t "10 37.5" --subplot 2
```
![](output\subplot-sim.svg)

## 8. Results for HITL simulation. (c) local image coordinates
画仿真中原始和滤波后图像坐标随时间变化图局部放大图
```
# python plot_data.py ../datas/377504aa-b91c-11eb-9cde-000c29e163c9/ekf_node-2-stdout.log IMG_y ekf_y -t "30.2 32.25"
python 8-plot_data.py ../datas/377504aa-b91c-11eb-9cde-000c29e163c9/ekf_node-2-stdout.log IMG_y ekf_y -t "30.2 32.25"
```
![](output\local-sim.svg)

## 9. Results for real flight experiments. (a) trajectory
画实飞中相对目标的全局坐标轨迹图
```
# python plot_data.py ../datas/20210521_172502_sim.log mav_pos -t "51 61" -p
python 9-plot_data.py ../datas/20210521_172502_sim.log mav_pos -t "51 61" -p
```
![](output\plotxy-real.svg)

## 10. Results for HITL simulation. (b) image coordinates
画实飞中原始和滤波后图像坐标随时间变化图
```
# python plot_data.py ../datas/20210521_172502_sim.log IMG_x ekf_x IMG_y ekf_y -t "2 68.4" --subplot 2
python 10-plot_data.py ../datas/20210521_172502_sim.log IMG_x ekf_x IMG_y ekf_y -t "2 68.4" --subplot 2
```
![](output\subplot-real.svg)

## 11. Results for HITL simulation. (c) local image coordinates
画实飞中原始和滤波后图像坐标随时间变化图局部放大图
```
# python plot_data.py ../datas/20210521_172502_sim.log IMG_y ekf_y -t "36.5 41"
# python plot_data.py ../datas/20210521_172502_sim.log IMG_y ekf_y -t "62.5 68.4"
python 11-plot_data.py ../datas/20210521_172502_sim.log IMG_y ekf_y -t "36.5 41"
python 11-plot_data.py ../datas/20210521_172502_sim.log IMG_y ekf_y -t "62.5 68.4"
```
![](output\local-real-1.svg)
![](output\local-real-2.svg)

# 三、更多
## 1. 总体绘图逻辑
* 绘图前设置全局参数，保证所有图片风格一致。
* 然后设置画布大小，这样生成的图片无论什么格式，后续都不需要缩放。
* 编写数据处理和绘图程序，可以使用默认风格或自定义风格`style_dict`
* 最后按照需要自定义本图片的修改，通常修改坐标轴范围和刻度数量

## 2. 论文和文档使用图片
Python -> svg -> emf -> Visio -> PDF -> eps -> LaTex

常用尺寸：单栏83mm，双栏176mm

新电脑里需要安装`inkscape`，安装到`D:\Inkscape`，添加环境变量。
* `Python`中程序出图，得到`svg`和`emf`格式图片。其中`svg`或`emf`格式可以用于`Word`文档中使用。是可以无限放大的矢量格式。
* 注意`Python`画图时就应该确定图片尺寸，避免后续缩放
* 在`Visio`中进行图片拼接、加坐标注释、小标题等操作。预先设定`Visio`页面尺寸
* `PDF`中裁剪多余空白，转换为`eps`格式用于`LaTeX`和投稿。

## 3. 模板
* `paper_plot\1-draw_trajectory.py`可以作为3d图样板
* `paper_plot\5-scatter_with_ekf.py`可以作为常规图样板
* `paper_plot\params.json`定义了全局模式
* `paper_plot\plot_tools.py`定义了一些线型和一些工具
* `paper_plot\Matplotlib 优雅作图笔记 - 可乐学人.ipynb`相当全的教程