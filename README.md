GridCarSmoothMotion
🛻 平滑轨迹生成 + 避障 + 动画可视化 —— 在二维栅格地图中模拟小车自然行驶轨迹的 Python 项目。

简介
本项目实现了一个在二维栅格地图中行驶的小车：

利用 DFS 遍历生成控制点；

使用 B-Spline 样条插值构建平滑路径；

进行真实尺寸的小车碰撞检测；

输出完整的动画轨迹并保存为 MP4/GIF；

适用于路径规划、仿真教学与算法演示等场景。

特性
✅ 平滑路径生成：轨迹自然流畅；

🚫 障碍物碰撞检测：真实小车尺寸模拟；

🎞️ 动画可视化：含方向角的动态轨迹；

🧭 转向优先遍历：更拟人的行驶路线；

📊 导出进度提示：动画导出时显示百分比。

快速开始
克隆项目并运行：
git clone https://github.com/your-username/GridCarSmoothMotion.git
cd GridCarSmoothMotion
python GridCarSmoothMotion.py
安装依赖：
pip install numpy matplotlib scipy
输出结果：

smooth_car_progress.mp4：轨迹动画；

smooth_car_progress_static.png：失败时的静态图。
使用示例
也可在其他项目中调用：
traversal = SmoothCurveCarTraversal(
    grid_map=[[...]],
    start_pos_grid=(r, c),
    car_length=0.7,
    car_width=0.35,
    spline_smoothness=0.0,
    points_per_segment=10
)
traversal.visualize("your_output.mp4")
参数说明
grid_map：二维数组，0 为通行，1 为障碍；

start_pos_grid：起点位置 (row, col)；

car_length / car_width：小车长宽；

spline_smoothness：曲线平滑度；

points_per_segment：插值密度。

效果预览
起点设定	平滑路径	小车动画
✅ 支持	✅ 支持	✅ 支持

若路径生成失败，自动输出静态图作为回退。

TODO
 支持终点设定与路径闭环；

 替换 DFS 为 A*/D* 等更智能算法；

 参数可视化与交互界面；

 添加单元测试与模块分离。

License
MIT License
