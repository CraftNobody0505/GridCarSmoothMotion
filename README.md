# 栅格地图中的平滑车辆路径遍历与动画演示

本项目是一个Python程序，用于在二维栅格地图中为模拟小车生成平滑的遍历路径。它包括使用优先转弯的深度优先搜索（DFS）进行路径规划，使用B样条进行路径平滑，针对矩形小车模型的碰撞检测，以及将小车运动可视化为MP4或GIF动画。

![演示动画占位符](placeholder_animation.gif)
*(请将 `placeholder_animation.gif` 替换为脚本生成的实际GIF动画，例如 `smooth_car_progress.gif`)*

## 主要功能

*   **栅格地图表示:** 使用简单的二维NumPy数组表示地图（0为空地，1为障碍物）。
*   **路径规划:** 实现了一种深度优先搜索（DFS）的变体，该变体优先考虑转弯操作，以探索更多可用空间。
*   **路径平滑:** 利用SciPy的B样条插值（`splprep`, `splev`）从原始栅格控制点创建平滑轨迹。
*   **运动学小车模型:** 将小车表示为具有可定义长度和宽度的矩形。
*   **碰撞检测:**
    *   检查小车（在给定姿态：x, y, 角度）是否与栅格地图中的障碍物发生碰撞。
    *   验证整个平滑路径是否存在碰撞，并在动画中高亮显示发生碰撞的路段。
*   **可视化:**
    *   使用Matplotlib显示栅格地图、规划路径和小车。
    *   将小车沿平滑路径的行驶过程制作成动画。
    *   如果小车进入预先检测到的碰撞状态，其颜色会发生变化（例如，变为橙色）。
    *   在栅格上显示起点。
    *   确保在绘图中正确显示中文字符和负号。
*   **动画导出:**
    *   使用 `ffmpeg` 将动画保存为MP4视频文件。
    *   在保存MP4期间提供进度回调，向控制台打印更新信息。
    *   如果MP4保存失败，则包含使用 `Pillow` 保存为GIF格式的备选方案。
    *   如果路径点过少无法生成动画或路径生成失败，则保存静态图像。
*   **可定制参数:**
    *   小车尺寸（长度、宽度）。
    *   样条平滑度。
    *   平滑路径中每段的点数。
    *   动画帧间隔。

## 环境要求

*   Python 3.x
*   NumPy
*   Matplotlib
*   SciPy
*   **FFmpeg:** 保存MP4动画所必需。
    *   Debian/Ubuntu: `sudo apt-get install ffmpeg`
    *   macOS (使用 Homebrew): `brew install ffmpeg`
    *   Windows: 从 [ffmpeg.org](https://ffmpeg.org/download.html) 下载并将其添加到系统的PATH环境变量中。
*   **Pillow:** 保存GIF动画所必需（作为备选方案）。
    *   `pip install Pillow`

## 安装步骤

1.  **克隆仓库:**
    ```bash
    git clone <您的仓库URL>
    cd <您的仓库名称>
    ```

2.  **安装Python依赖:**
    ```bash
    pip install numpy matplotlib scipy Pillow
    ```
    （请确保已按照“环境要求”部分单独安装了FFmpeg）。

## 如何运行

主脚本可以直接执行。地图、起始位置和小车参数的配置在Python脚本的 `if __name__ == '__main__':` 代码块中进行。

1.  **（可选）修改参数:**
    打开Python脚本（例如 `main.py` 或您命名的文件），在 `if __name__ == '__main__':` 代码块中根据需要调整以下内容：
    *   `grid_map_for_smooth`: 定义您的栅格地图。
    *   `start_pos_grid_smooth`: 设置小车的起始 `(行, 列)`。
    *   `SmoothCurveCarTraversal` 构造函数参数：
        *   `car_length` (小车长度)
        *   `car_width` (小车宽度)
        *   `spline_smoothness` (样条平滑度)
        *   `points_per_segment` (每段点数)
    *   `traversal_system.visualize` 方法参数：
        *   `filename`: 输出动画文件名。
        *   `interval`: 动画帧间隔（毫秒）。

2.  **运行脚本:**
    ```bash
    python your_script_name.py
    ```
    (请将 `your_script_name.py` 替换为您的Python脚本文件名)

## 输出结果

*   **动画文件:** 一个 `.mp4` 文件（如果MP4保存失败，则为 `.gif` 文件，例如 `smooth_car_progress.mp4`）将被保存在同一目录中，显示小车沿路径行驶的过程。
*   **静态图像:** 如果路径太短或无效，将保存一个静态 `.png` 图像（例如 `smooth_car_progress_static.png` 或 `smooth_car_progress_static_fallback.png`）。
*   **控制台输出:**
    *   关于路径生成和碰撞验证的状态消息。
    *   动画保存期间的进度更新（例如，“保存动画进度: X%”）。
    *   任何错误消息。

## 代码结构简介

*   **`SmoothCurveCarTraversal` 类:**
    *   `__init__(...)`: 初始化地图、小车、路径规划和平滑处理。
    *   `_generate_grid_control_points(...)`: 基于DFS的原始路径点生成。
    *   `_simplify_and_convert_control_points(...)`: 简化原始路径点。
    *   `_create_smooth_path_from_waypoints(...)`: 使用B样条进行路径平滑。
    *   `_validate_smooth_path_for_collision()`: 检查整个平滑路径的碰撞情况。
    *   `visualize(...)`: 处理Matplotlib动画的设置和保存。
    *   `_visualize_static_obstacle_start(...)`: 用于生成静态图像的备选方法。
    *   `_is_grid_valid(...)`, `_get_turn_prioritized_grid_moves(...)`: DFS的辅助方法。
*   **碰撞检测函数:**
    *   `get_car_corners(...)`: 计算小车四个角点的世界坐标。
    *   `check_car_collision_at_pose(...)`: 在特定姿态下检查小车与栅格障碍物的碰撞。
*   **`SaveProgressCallback` 类:**
    *   一个辅助类，传递给 `FuncAnimation.save` 方法，用于在控制台提供保存进度的反馈。
*   **常量与设置:**
    *   Matplotlib关于中文字体和视觉样式的配置。
    *   栅格单元类型常量 (EMPTY, OBSTACLE, START_POINT_GRID)。
*   **`if __name__ == '__main__':` 代码块:**
    *   示例用法：定义示例栅格地图和起始位置，实例化 `SmoothCurveCarTraversal`，并调用 `visualize()`。

## 未来可尝试的增强功能

*   实现更高级的路径规划算法（如A*, RRT*等）。
*   集成车辆动力学和控制约束。
*   支持动态障碍物。
*   交互式地图定义或从文件加载地图。
*   更精细的碰撞检测算法（例如，用于多边形间碰撞的SAT分离轴定理）。

## 许可证

（在此处指定您的许可证，例如MIT许可证。如果您还没有，请考虑添加一个。一个常见的简单选择是MIT。）
