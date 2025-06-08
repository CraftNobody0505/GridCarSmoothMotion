# GridCarSmoothMotion

🛻 平滑轨迹生成 + 避障 + 动画可视化 —— 用于栅格地图中小车路径规划的 Python 项目。

## 简介

本项目实现了一个在二维栅格地图中行驶的小车，其轨迹由 DFS 遍历生成控制点，通过样条插值构建平滑曲线，并进行碰撞检测。支持路径动态可视化，并保存为 MP4/GIF 动画，适用于路径规划、仿真教学与算法演示。

## 特性

- ✅ **平滑路径生成**：基于样条曲线 (B-Spline)，轨迹自然、连续。
- 🚫 **障碍物碰撞检测**：模拟真实小车宽度/长度，检测与障碍物的几何交叉。
- 🎞️ **动画可视化**：利用 `matplotlib.animation`，输出带角度变化的小车运动轨迹。
- 🧭 **转向优先遍历策略**：在 DFS 中引入转向偏好，提升路径自然度。
- 🔍 **进度显示**：动画导出过程可视化百分比进度，适配 ffmpeg/Pillow。

## 快速开始

```bash
git clone https://github.com/your-username/GridCarSmoothMotion.git
cd GridCarSmoothMotion
python GridCarSmoothMotion.py
依赖：
pip install numpy matplotlib scipy
运行成功后，默认输出：

smooth_car_progress.mp4：可视化动画文件

smooth_car_progress_static.png：如路径点太少或失败，则输出静态图

使用说明
可通过 SmoothCurveCarTraversal 类快速集成：
traversal = SmoothCurveCarTraversal(
    grid_map=[[...]], 
    start_pos_grid=(r, c), 
    car_length=0.7, 
    car_width=0.35,
    spline_smoothness=0.0,
    points_per_segment=10
)
traversal.visualize("your_output.mp4")
参数说明：

grid_map：二维整数矩阵，0 表示可通行，1 表示障碍物。

start_pos_grid：起点坐标 (行, 列)。

car_length, car_width：小车长宽，用于碰撞检测。

spline_smoothness：样条平滑度控制参数。

points_per_segment：每段样条插值的细分点数。

效果预览
起点设定	规划路径	小车动画
✅ 支持	✅ 支持	✅ 支持

若起点落在障碍物上或路径生成失败，会回退输出静态图。

TODO
 支持动态终点与多段路径连接

 接入 A* / D* 替代 DFS 控制点生成

 单元测试覆盖

许可
MIT License
