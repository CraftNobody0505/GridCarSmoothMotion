GridCarSmoothMotion (网格小车平滑运动)
GridCarSmoothMotion 是一个 Python 项目，用于演示在二维网格地图上为类车智能体生成并可视化平滑遍历路径的过程。其主要特性包括：
基于网格的路径探索：使用一种优先转向的深度优先搜索 (DFS) 变体来生成控制点。
路径平滑处理：利用 B-样条插值 (通过 SciPy 实现)。
车辆朝向计算：沿着平滑路径计算车辆的精确朝向。
碰撞检测：为矩形小车模型提供与网格障碍物的碰撞检测功能。
动画可视化：使用 Matplotlib 动态展示小车的移动过程。
碰撞高亮：在路径上高亮显示检测到碰撞的路段。
动画保存：支持将动画保存为 MP4 (需要 FFmpeg) 或 GIF (使用 Pillow)，并在控制台提供保存进度反馈。
(图片占位：请在此处放置一个演示动画的 GIF 图片，例如命名为 placeholder_animation.gif)
(请将 placeholder_animation.gif 替换为你的程序实际运行效果的 GIF 动图!)
工作原理
网格地图输入: 系统接收一个二维 NumPy 数组作为环境表示，其中单元格可以是空格或障碍物。同时定义小车在网格上的起始位置。
控制点生成: 一个类似 DFS 的算法从起点开始探索可到达的空格。该算法被设计为优先进行转向操作，以便覆盖更大区域或创建更有趣的路径，最终生成一个由 (行, 列) 网格坐标组成的控制点列表。
路径简化与转换: 原始的网格控制点会被简化 (例如，移除过于接近的点) 并转换为连续的 (x, y) 坐标 (通常是单元格中心点)。
平滑路径创建:
使用简化后的 (x, y) 航点，通过 scipy.interpolate.splprep 和 splev 创建 B-样条曲线。
利用样条曲线的导数计算路径上每个点处小车的朝向 (角度)。
如果样条创建失败 (例如，点数过少)，则回退到线性插值。
碰撞校验: 对于平滑路径上的每一个位姿 (x, y, 角度)，都会检查矩形小车模型是否与网格地图中的障碍物发生碰撞。
可视化与动画:
使用 Matplotlib 显示网格、规划路径以及小车的动画。
如果小车在当前路径位姿上检测到碰撞，其颜色会发生变化。
动画可以保存为 MP4 或 GIF 文件，保存过程中会在控制台更新进度。
环境要求
Python 3.7+
NumPy
Matplotlib
SciPy
FFmpeg (推荐用于保存 MP4 视频，请确保已将其添加到系统的 PATH 环境变量中)
Pillow (用于保存 GIF 视频，若 FFmpeg 失败或不希望使用时)
你可以使用 pip 安装 Python 库:
(代码块开始)
pip install numpy matplotlib scipy Pillow
(代码块结束)
使用方法
克隆仓库 (或下载脚本):
(代码块开始)
git clone https://github.com/YOUR_USERNAME/GridCarSmoothMotion.git
cd GridCarSmoothMotion
(代码块结束)
(请将 YOUR_USERNAME 替换为你的 GitHub 用户名)
配置地图和起点:
打开 Python 脚本 (例如 smooth_car_traversal.py)，在 if __name__ == '__main__': 代码块中修改 grid_map_for_smooth 和 start_pos_grid_smooth 变量，以定义你想要的地图和起始位置。
(代码块开始 - Python示例)
if name == 'main':
grid_map_for_smooth = [
[0,0,0,0,0,0,0,0,0,0],
[0,1,1,0,0,0,0,1,1,0],
# ... 更多地图数据 ...
[0,0,0,0,0,0,0,0,0,0],
]
start_pos_grid_smooth = (0,0) # (行, 列)
# ... 主执行块的其余部分
Use code with caution.
(代码块结束 - Python示例)
调整参数 (可选):
你可以更改小车尺寸、样条平滑度、每段点数以及动画输出设置：
(代码块开始 - Python示例)
traversal_system = SmoothCurveCarTraversal(current_grid, current_start,
car_length=0.7,
car_width=0.35,
spline_smoothness=0.0,
points_per_segment=10)
traversal_system.visualize(filename="my_animation.mp4", interval=40)
(代码块结束 - Python示例)
运行脚本:
(代码块开始)
python your_script_name.py
(代码块结束)
(请将 your_script_name.py 替换为你的 Python 文件实际名称)。
一个动画窗口将会出现，并且视频/GIF 将被保存到指定的文件名 (例如 smooth_car_progress.mp4)。
代码结构
SmoothCurveCarTraversal 类:
__init__: 初始化地图、小车和路径生成过程。
_generate_grid_control_points: 实现用于生成控制点的类 DFS 探索算法。
_get_turn_prioritized_grid_moves: _generate_grid_control_points 的辅助函数，用于排序下一步的移动。
_simplify_and_convert_control_points: 清理原始控制点。
_create_smooth_path_from_waypoints: 生成样条或线性插值路径。
_validate_smooth_path_for_collision: 检查生成的平滑路径是否存在碰撞。
visualize: 处理 Matplotlib 动画和保存。
_visualize_static_obstacle_start: 当路径生成早期失败时的回退可视化方法。
SaveProgressCallback 类: 一个辅助类，用于在动画保存期间提供进度更新。
辅助函数:
get_car_corners: 计算小车四个角点的世界坐标。
check_car_collision_at_pose: 对给定的小车位姿执行碰撞检测。
未来可能的增强功能
实现更高级的路径规划算法 (例如 A星, RRT星)。
路径优化 (例如，最短路径、能量最优等)。
支持更复杂的车辆模型 (例如，差速驱动、阿克曼转向)。
交互式地图编辑器。
集成到更大的机器人仿真环境或框架中。
贡献
欢迎通过 Pull Requests 提出改进建议或修复错误。对于重大的更改，请先开一个 Issue 来讨论你想要改变的内容。
许可证
MIT (链接到 https://choosealicense.com/licenses/mit/)
