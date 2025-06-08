GridCarSmoothMotion
🛻 平滑轨迹生成 + 避障 + 动画可视化 —— 在二维栅格地图中模拟小车自然行驶轨迹的 Python 项目。

简介
本项目实现了一个小车在栅格地图中自动平滑运动的系统：

基于 DFS 遍历生成控制点；

样条插值（B-Spline）构建自然轨迹；

支持真实尺寸小车的障碍物碰撞检测；

自动生成轨迹动画并导出 MP4/GIF；

适合路径规划、算法教学与仿真演示。

（2）特性与快速开始：
特性
✅ 平滑路径生成：样条曲线插值，连贯自然；

🚫 碰撞检测：小车尺寸可调，检测障碍交叠；

🎞️ 动画导出：支持方向角变化、碰撞点高亮；

🧭 转向优先遍历：路线更符合实际驾驶习惯；

📊 进度反馈：保存动画时输出进度条。

快速开始
bash
复制
编辑
git clone https://github.com/your-username/GridCarSmoothMotion.git
cd GridCarSmoothMotion
pip install numpy matplotlib scipy
python GridCarSmoothMotion.py
（3）使用说明（含代码）：
使用示例
你可以在自己的项目中调用主类：

python
复制
编辑
from GridCarSmoothMotion import SmoothCurveCarTraversal

traversal = SmoothCurveCarTraversal(
    grid_map=[[...]], 
    start_pos_grid=(r, c), 
    car_length=0.7, 
    car_width=0.35,
    spline_smoothness=0.0,
    points_per_segment=10
)
traversal.visualize("output.mp4")
参数说明
参数名	含义
grid_map	二维数组，0 为通行，1 为障碍
start_pos_grid	起点位置 (行, 列)
car_length	小车长度（米）
car_width	小车宽度（米）
spline_smoothness	样条曲线平滑度（建议设为 0）
points_per_segment	每段插值的点数，决定平滑精度

（4）效果预览、计划与许可证：
效果预览
起点设定	平滑轨迹	碰撞检测	动画导出
✅ 支持	✅ 支持	✅ 支持	✅ 支持

如轨迹无法生成或起点为障碍，自动回退为静态图输出。

TODO
 支持自定义终点；

 引入 A* / D* 替代 DFS；

 增加参数调节界面；

 添加单元测试模块。

License
MIT License
