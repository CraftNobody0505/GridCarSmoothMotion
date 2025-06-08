GridCarSmoothMotion
平滑轨迹生成 + 避障 + 动画可视化 —— 在二维栅格地图中模拟小车自然行驶轨迹的 Python 项目。

【简介】

本项目实现了一个在二维栅格地图中行驶的小车：

利用 DFS 遍历生成控制点；

使用 B-Spline 样条插值构建平滑路径；

进行真实尺寸的小车碰撞检测；

输出完整的动画轨迹并保存为 MP4/GIF；

适用于路径规划、仿真教学与算法演示等场景。

【特性】

平滑路径生成：通过样条曲线插值，轨迹自然流畅；

障碍物碰撞检测：模拟真实小车尺寸；

动画可视化：轨迹可保存为 mp4/gif，含角度与颜色变化；

方向优先遍历：路径生成更自然；

支持进度回调：动画保存时显示百分比。

【快速开始】

克隆仓库：
git clone https://github.com/your-username/GridCarSmoothMotion.git

安装依赖：
pip install numpy matplotlib scipy

运行：
python GridCarSmoothMotion.py

默认输出 smooth_car_progress.mp4 或 fallback 的静态图。

【如何调用主类】

支持在其他项目中使用核心类：

SmoothCurveCarTraversal(
grid_map=[[...]],
start_pos_grid=(r, c),
car_length=0.7,
car_width=0.35,
spline_smoothness=0.0,
points_per_segment=10
).visualize("your_output.mp4")

【参数说明】

grid_map：二维列表，0 表示空地，1 为障碍物；

start_pos_grid：起点坐标 (行, 列)；

car_length / car_width：小车尺寸；

spline_smoothness：曲线平滑程度；

points_per_segment：每段细化的点数。

【效果预览】

起点设定 ✅
平滑路径 ✅
小车动画 ✅

失败时自动输出静态图作为回退。

【TODO】

支持终点设定；

引入 A* 等替代 DFS；

增加交互与调试功能；

添加测试与验证。

【License】

MIT License
