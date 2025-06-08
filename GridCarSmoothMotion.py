import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.colors import ListedColormap
from matplotlib.patches import Rectangle
from matplotlib.transforms import Affine2D
import random
from scipy.interpolate import splprep, splev
from collections import deque
import math
import sys  # 用于 flush 输出

# ... (所有之前的常量、辅助函数、类定义等保持不变) ...
# 包括 get_car_corners, check_car_collision_at_pose
# 以及 SmoothCurveCarTraversal 类的 __init__, _validate_smooth_path_for_collision,
# _is_grid_valid, _get_turn_prioritized_grid_moves, _generate_grid_control_points,
# _simplify_and_convert_control_points, _create_smooth_path_from_waypoints,
# _visualize_static_obstacle_start

# --- Matplotlib 中文显示和负号显示配置 ---
plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False

# --- 定义可视化常量 ---
EMPTY = 0
OBSTACLE = 1
START_POINT_GRID = 2

cmap_colors = [
    'white',  # 0: EMPTY
    'black',  # 1: OBSTACLE
    'green'  # 2: START_POINT_GRID
]
custom_cmap = ListedColormap(cmap_colors)
norm = plt.Normalize(vmin=0, vmax=len(cmap_colors) - 1)


# --- 碰撞检测函数 (从之前代码复制) ---
def get_car_corners(center_x, center_y, angle_degrees, length, width):
    angle_rad = math.radians(angle_degrees)
    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)
    half_l = length / 2
    half_w = width / 2
    corners_local = [
        (-half_l, -half_w), (+half_l, -half_w),
        (+half_l, +half_w), (-half_l, +half_w)
    ]
    corners_world = []
    for x_local, y_local in corners_local:
        x_rot = x_local * cos_a - y_local * sin_a
        y_rot = x_local * sin_a + y_local * cos_a
        corners_world.append((center_x + x_rot, center_y + y_rot))
    return corners_world


def check_car_collision_at_pose(center_x, center_y, angle_degrees, car_length, car_width, grid_map_data):
    rows, cols = grid_map_data.shape
    corners = get_car_corners(center_x, center_y, angle_degrees, car_length, car_width)
    all_x = [p[0] for p in corners]
    all_y = [p[1] for p in corners]
    min_gx = math.floor(min(all_x))
    max_gx = math.ceil(max(all_x))
    min_gy = math.floor(min(all_y))
    max_gy = math.ceil(max(all_y))

    for gx_test in range(int(min_gx), int(max_gx)):
        for gy_test in range(int(min_gy), int(max_gy)):
            if not (0 <= gy_test < rows and 0 <= gx_test < cols):
                continue
            if grid_map_data[gy_test, gx_test] == OBSTACLE:
                obs_rect_min_x, obs_rect_max_x = gx_test, gx_test + 1
                obs_rect_min_y, obs_rect_max_y = gy_test, gy_test + 1
                car_min_x, car_max_x = min(all_x), max(all_x)
                car_min_y, car_max_y = min(all_y), max(all_y)
                if not (car_max_x <= obs_rect_min_x or car_min_x >= obs_rect_max_x or \
                        car_max_y <= obs_rect_min_y or car_min_y >= obs_rect_max_y):
                    return True  # Simplified AABB collision
    return False


class SmoothCurveCarTraversal:
    # ... (__init__ 和其他方法到 _create_smooth_path_from_waypoints 保持不变)
    def __init__(self, grid_map, start_pos_grid, car_length=0.8, car_width=0.4, spline_smoothness=0,
                 points_per_segment=20):
        self.base_grid_map = np.array(grid_map, dtype=int)
        self.rows, self.cols = self.base_grid_map.shape
        self.start_pos_grid = tuple(start_pos_grid)

        self.car_length = car_length
        self.car_width = car_width
        self.spline_smoothness = spline_smoothness
        self.points_per_segment = points_per_segment
        self.collided_path_indices = []

        if not (0 <= self.start_pos_grid[0] < self.rows and 0 <= self.start_pos_grid[1] < self.cols):
            raise ValueError("起点位置超出地图边界。")
        if self.base_grid_map[self.start_pos_grid] == OBSTACLE:
            print(f"警告: 起点 {self.start_pos_grid} 是一个障碍物。")
            self.smooth_path_points = []
            return

        self.grid_control_points = self._generate_grid_control_points(self.start_pos_grid[0], self.start_pos_grid[1])

        if not self.grid_control_points:
            print("未能生成栅格控制点。")
            self.smooth_path_points = []
            return

        simplified_control_points_xy = self._simplify_and_convert_control_points(self.grid_control_points)

        if len(simplified_control_points_xy) < 2:
            print("控制点太少，无法生成平滑路径。")
            if len(simplified_control_points_xy) == 1:
                x, y = simplified_control_points_xy[0]
                self.smooth_path_points = [(x, y, 0)]
            else:
                self.smooth_path_points = []
            if self.smooth_path_points:
                self._validate_smooth_path_for_collision()
            return

        self.smooth_path_points = self._create_smooth_path_from_waypoints(simplified_control_points_xy)

        if self.smooth_path_points:
            self._validate_smooth_path_for_collision()

    def _validate_smooth_path_for_collision(self):
        self.collided_path_indices = []
        if not self.smooth_path_points:
            return
        print("开始进行路径碰撞校验...")
        for i, (x, y, angle) in enumerate(self.smooth_path_points):
            if check_car_collision_at_pose(x, y, angle, self.car_length, self.car_width, self.base_grid_map):
                self.collided_path_indices.append(i)
        if self.collided_path_indices:
            print(f"警告: 检测到路径上有 {len(self.collided_path_indices)} 个点与障碍物发生碰撞！")
        else:
            print("路径碰撞校验通过，未发现碰撞。")

    def _is_grid_valid(self, r, c):
        return 0 <= r < self.rows and \
            0 <= c < self.cols and \
            self.base_grid_map[r, c] == EMPTY

    def _get_turn_prioritized_grid_moves(self, r, c, prev_r, prev_c, visited_dfs_local):
        if prev_r is None:
            dr_in, dc_in = 0, 1
        else:
            dr_in, dc_in = r - prev_r, c - prev_c
        abs_moves_vectors = [(-1, 0), (0, 1), (1, 0), (0, -1)]
        try:
            current_abs_dir_idx = abs_moves_vectors.index((dr_in, dc_in))
        except ValueError:
            current_abs_dir_idx = 1
        straight_idx = current_abs_dir_idx
        left_idx = (current_abs_dir_idx - 1 + 4) % 4
        right_idx = (current_abs_dir_idx + 1) % 4
        back_idx = (current_abs_dir_idx + 2) % 4
        priority_indices = [left_idx, straight_idx, right_idx, back_idx]
        categorized = {'left': [], 'straight': [], 'right': [], 'back': []}
        category_map = ['left', 'straight', 'right', 'back']
        for i, move_idx in enumerate(priority_indices):
            m_dr, m_dc = abs_moves_vectors[move_idx]
            nr, nc = r + m_dr, c + m_dc
            if self._is_grid_valid(nr, nc) and not visited_dfs_local[nr, nc]:
                categorized[category_map[i]].append((nr, nc))
        final_ordered_moves = []
        for cat_name in category_map:
            random.shuffle(categorized[cat_name])
            final_ordered_moves.extend(categorized[cat_name])
        return final_ordered_moves

    def _generate_grid_control_points(self, r_start, c_start):
        _path_temp = []
        _visited_temp = np.zeros_like(self.base_grid_map, dtype=bool)

        def _dfs_recursive_for_control(r_curr, c_curr, pr_curr, pc_curr, path_list, visited_arr):
            path_list.append((r_curr, c_curr))
            visited_arr[r_curr, c_curr] = True
            ordered_next = self._get_turn_prioritized_grid_moves(r_curr, c_curr, pr_curr, pc_curr, visited_arr)
            for nr_next, nc_next in ordered_next:
                if not visited_arr[nr_next, nc_next]:
                    _dfs_recursive_for_control(nr_next, nc_next, r_curr, c_curr, path_list, visited_arr)
                    path_list.append((r_curr, c_curr))

        _dfs_recursive_for_control(r_start, c_start, None, None, _path_temp, _visited_temp)
        return _path_temp

    def _simplify_and_convert_control_points(self, grid_points_rc, min_distance_sq=0.5 ** 2):
        if not grid_points_rc: return []
        unique_waypoints_xy = []
        last_r_processed, last_c_processed = grid_points_rc[0]
        unique_waypoints_xy.append((last_c_processed + 0.5, last_r_processed + 0.5))
        for r_curr, c_curr in grid_points_rc[1:]:
            current_x, current_y = c_curr + 0.5, r_curr + 0.5
            last_added_x, last_added_y = unique_waypoints_xy[-1]
            dist_to_last_added_sq = (current_x - last_added_x) ** 2 + (current_y - last_added_y) ** 2
            if dist_to_last_added_sq >= min_distance_sq:
                unique_waypoints_xy.append((current_x, current_y))
            last_r_processed, last_c_processed = r_curr, c_curr
        final_points = []
        if unique_waypoints_xy:
            final_points.append(unique_waypoints_xy[0])
            for i in range(1, len(unique_waypoints_xy)):
                if unique_waypoints_xy[i][0] != final_points[-1][0] or \
                        unique_waypoints_xy[i][1] != final_points[-1][1]:
                    final_points.append(unique_waypoints_xy[i])
        return final_points

    def _create_smooth_path_from_waypoints(self, waypoints_xy):
        if len(waypoints_xy) < 2:
            if waypoints_xy:
                x, y = waypoints_xy[0]
                return [(x, y, 0)]
            return []
        waypoints_xy_np = np.array(waypoints_xy)
        x_wp = waypoints_xy_np[:, 0]
        y_wp = waypoints_xy_np[:, 1]
        try:
            k_spline = min(3, len(x_wp) - 1)
            if k_spline < 1:
                if len(x_wp) == 1: return [(x_wp[0], y_wp[0], 0)]
                return []
            tck, u = splprep([x_wp, y_wp], s=self.spline_smoothness, k=k_spline)
        except Exception as e:
            print(f"样条插值失败: {e}. Waypoints len: {len(waypoints_xy_np)}. Fallback to linear.")
            path_points = []
            if len(waypoints_xy_np) == 1:
                return [(waypoints_xy_np[0][0], waypoints_xy_np[0][1], 0)]
            for i in range(len(waypoints_xy_np) - 1):
                p1, p2 = waypoints_xy_np[i], waypoints_xy_np[i + 1]
                num_pts_seg = self.points_per_segment
                for j in range(num_pts_seg):
                    t_seg = j / self.points_per_segment
                    pt_x = p1[0] * (1 - t_seg) + p2[0] * t_seg
                    pt_y = p1[1] * (1 - t_seg) + p2[1] * t_seg
                    dx = p2[0] - p1[0];
                    dy = p2[1] - p1[1]
                    angle = np.degrees(np.arctan2(dy, dx)) if not (abs(dx) < 1e-6 and abs(dy) < 1e-6) else 0
                    path_points.append((pt_x, pt_y, angle))
            if waypoints_xy_np.size > 0 and path_points:
                last_wp_x, last_wp_y = waypoints_xy_np[-1]
                if not (abs(path_points[-1][0] - last_wp_x) < 1e-6 and abs(path_points[-1][1] - last_wp_y) < 1e-6):
                    angle_last = path_points[-1][2] if path_points else 0
                    path_points.append((last_wp_x, last_wp_y, angle_last))
            if not path_points and waypoints_xy_np.size > 0:
                path_points.append((waypoints_xy_np[0][0], waypoints_xy_np[0][1], 0))
            return path_points
        num_fine_points = max(2, (len(x_wp) - 1) * self.points_per_segment + 1)
        u_fine = np.linspace(u.min(), u.max(), num_fine_points)
        x_fine, y_fine = splev(u_fine, tck)
        dx_fine, dy_fine = splev(u_fine, tck, der=1)
        angles_rad = np.arctan2(dy_fine, dx_fine)
        angles_deg = np.degrees(angles_rad)
        smooth_path = []
        for i in range(len(x_fine)):
            smooth_path.append((x_fine[i], y_fine[i], angles_deg[i]))
        return smooth_path

    # --- 新增：进度回调辅助类 ---
    # 使用类来存储上一次打印的百分比，避免使用全局变量
    class SaveProgressCallback:
        def __init__(self, total_frames, update_increment_percent=5):
            self.total_frames = total_frames
            self.update_increment_percent = update_increment_percent
            self.last_reported_percent = -1

        def __call__(self, current_frame, total_frames):
            # total_frames 参数由 FuncAnimation.save 提供，可能与 self.total_frames 不同
            # 通常它们是一样的，但以 save 方法提供的为准
            percent_done = int((current_frame / total_frames) * 100)

            # 只有当达到新的增量阈值时才打印
            if percent_done >= self.last_reported_percent + self.update_increment_percent:
                # 使用 \r 和 end='' 来实现同一行更新进度条
                # sys.stdout.write(f"\r保存动画进度: {percent_done}% (帧 {current_frame}/{total_frames})")
                # sys.stdout.flush()
                # 在某些环境（如IDE控制台）\r可能效果不好，直接打印新行更可靠
                print(f"保存动画进度: {percent_done}% (帧 {current_frame}/{total_frames})")
                self.last_reported_percent = percent_done
            elif current_frame == total_frames - 1:  # 确保最后100%会显示 (如果之前没到)
                # sys.stdout.write(f"\r保存动画进度: 100% (帧 {current_frame+1}/{total_frames})\n")
                # sys.stdout.flush()
                print(f"保存动画进度: 100% (帧 {current_frame + 1}/{total_frames})")

    def visualize(self, filename="smooth_car_progress.mp4", interval=50):
        if not self.smooth_path_points:
            print("没有平滑路径可供可视化。")
            self._visualize_static_obstacle_start(filename)
            return

        fig, ax = plt.subplots(figsize=(max(8, self.cols / 1.5), max(8, self.rows / 1.5)))
        plt.tight_layout(pad=3.0)

        display_grid_static = self.base_grid_map.copy().astype(float)
        if self.base_grid_map[self.start_pos_grid] == EMPTY:
            display_grid_static[self.start_pos_grid] = START_POINT_GRID
        ax.imshow(display_grid_static, cmap=custom_cmap, norm=norm, origin='upper',
                  extent=[-0.5, self.cols - 0.5, self.rows - 0.5, -0.5])
        ax.set_xticks(np.arange(-.5, self.cols, 1), minor=True)
        ax.set_yticks(np.arange(-.5, self.rows, 1), minor=True)
        ax.grid(which="minor", color="dimgray", linestyle='-', linewidth=0.7)
        ax.set_xticks(np.arange(0, self.cols, 1))
        ax.set_yticks(np.arange(0, self.rows, 1))
        ax.set_aspect('equal', adjustable='box')

        if self.smooth_path_points:
            path_x = [p[0] for p in self.smooth_path_points]
            path_y = [p[1] for p in self.smooth_path_points]
            ax.plot(path_x, path_y, color='deepskyblue', linestyle='--', linewidth=1.5, alpha=0.8, zorder=5,
                    label="规划轨迹")
            if self.collided_path_indices:
                print("在可视化中标红碰撞路径段...")
                coll_x = [self.smooth_path_points[i][0] for i in self.collided_path_indices]
                coll_y = [self.smooth_path_points[i][1] for i in self.collided_path_indices]
                ax.scatter(coll_x, coll_y, color='magenta', s=10, zorder=6, label="碰撞点")
            if len(self.smooth_path_points) > 1:
                ax.legend(fontsize='small', loc='upper right')

        car_patch = Rectangle((-self.car_length / 2, -self.car_width / 2),
                              self.car_length, self.car_width,
                              facecolor='crimson', edgecolor='black', lw=0.5, zorder=10)
        ax.add_patch(car_patch)

        title = ax.text(0.5, 1.02, "", bbox={'facecolor': 'white', 'alpha': 0.7, 'pad': 3},
                        transform=ax.transAxes, ha="center", fontsize=9)

        def init_animation():
            first_x, first_y, first_angle = self.smooth_path_points[0]
            transform = Affine2D().rotate_deg_around(0, 0, first_angle).translate(first_x, first_y)
            car_patch.set_transform(transform + ax.transData)
            if self.collided_path_indices and 0 in self.collided_path_indices:
                car_patch.set_facecolor('orange')
            else:
                car_patch.set_facecolor('crimson')
            title.set_text("开始平滑行驶 (带碰撞检测)...")
            return [car_patch, title]

        def update_animation(frame_idx):
            x, y, angle = self.smooth_path_points[frame_idx]
            transform = Affine2D().rotate_deg_around(0, 0, angle).translate(x, y)
            car_patch.set_transform(transform + ax.transData)
            if self.collided_path_indices and frame_idx in self.collided_path_indices:
                car_patch.set_facecolor('orange')
            else:
                car_patch.set_facecolor('crimson')
            title.set_text(
                f"帧: {frame_idx + 1}/{len(self.smooth_path_points)}, 位置: ({x:.1f},{y:.1f}), 朝向: {angle:.0f}°")
            return [car_patch, title]

        num_frames = len(self.smooth_path_points)
        if num_frames <= 1:
            if self.smooth_path_points:
                init_animation()
            else:
                ax.set_title("无法生成平滑路径")
            plt.savefig(filename.replace(".mp4", "_static.png"))
            print(f"路径点过少({num_frames})。静态图像已保存。")
            plt.show()
            return

        ani = animation.FuncAnimation(fig, update_animation, frames=num_frames,
                                      init_func=init_animation, blit=True,
                                      interval=interval, repeat=False)

        # --- 使用进度回调 ---
        print(f"准备保存动画到 {filename} ({num_frames} 帧)...")
        # progress_callback_obj = self.SaveProgressCallback(num_frames, update_increment_percent=10) # 每10%更新
        #  如果希望 SaveProgressCallback 是一个独立的函数而不是类方法:
        global_progress_callback_obj = SaveProgressCallback(num_frames, update_increment_percent=10)

        try:
            # ani.save(filename, writer='ffmpeg', fps=max(1, 1000 // interval), dpi=150,
            #          progress_callback=progress_callback_obj)
            ani.save(filename, writer='ffmpeg', fps=max(1, 1000 // interval), dpi=150,
                     progress_callback=global_progress_callback_obj)  # 使用全局版本的对象
            print(f"\n动画已保存到: {filename}")  # 加一个换行符，因为进度可能不换行
        except Exception as e:
            print(f"\n保存动画失败 (ffmpeg): {e}")
            print("尝试使用 Pillow 保存为 GIF 格式...")
            try:
                gif_filename = filename.replace(".mp4", ".gif")
                # PillowWriter 可能不支持 progress_callback，或支持方式不同
                # 这里我们先不给PillowWriter传progress_callback，或者你需要查阅其文档
                ani.save(gif_filename, writer=animation.PillowWriter(fps=max(1, 1000 // interval)), dpi=150)
                print(f"GIF动画已保存到: {gif_filename}")
            except Exception as e_gif:
                print(f"保存GIF失败 (Pillow): {e_gif}.")
        plt.show()

    def _visualize_static_obstacle_start(self, filename):  # (保持不变)
        fig, ax = plt.subplots(figsize=(max(8, self.cols / 1.5), max(8, self.rows / 1.5)))
        display_grid_static = self.base_grid_map.copy().astype(float)
        title_text = "无法生成平滑路径"
        if hasattr(self, 'start_pos_grid') and self.base_grid_map[self.start_pos_grid] == OBSTACLE:
            title_text = "起点是障碍物"
            car_patch_static = Rectangle((self.start_pos_grid[1] + 0.5 - self.car_length / 2,
                                          self.start_pos_grid[0] + 0.5 - self.car_width / 2),
                                         self.car_length, self.car_width,
                                         facecolor='crimson', edgecolor='black', zorder=10)
            ax.add_patch(car_patch_static)
        elif hasattr(self, 'start_pos_grid') and self.base_grid_map[self.start_pos_grid] == EMPTY:
            display_grid_static[self.start_pos_grid] = START_POINT_GRID
        ax.imshow(display_grid_static, cmap=custom_cmap, norm=norm, origin='upper',
                  extent=[-0.5, self.cols - 0.5, self.rows - 0.5, -0.5])
        ax.set_title(title_text + " (无栅格轨迹)")
        ax.set_xticks(np.arange(-.5, self.cols, 1), minor=True);
        ax.set_yticks(np.arange(-.5, self.rows, 1), minor=True)
        ax.grid(which="minor", color="dimgray", linestyle='-', linewidth=0.7)
        ax.tick_params(which="minor", size=0)
        ax.set_xticks(np.arange(0, self.cols, 1));
        ax.set_yticks(np.arange(0, self.rows, 1))
        ax.set_aspect('equal', adjustable='box')
        plt.savefig(filename.replace(".mp4", "_static_fallback.png"))
        print(f"静态回退图像已保存到: {filename.replace('.mp4', '_static_fallback.png')}")
        plt.show()


# --- 进度回调辅助类 (定义在全局作用域，以便 FuncAnimation.save 可以访问) ---
class SaveProgressCallback:
    def __init__(self, total_frames, update_increment_percent=5):
        self.total_frames = total_frames
        self.update_increment_percent = update_increment_percent
        # 初始化为-1，确保0%（如果update_increment_percent允许）能被打印
        self.last_reported_percent = - (update_increment_percent)

    def __call__(self, current_frame, total_frames_from_save):
        # total_frames_from_save 是 FuncAnimation.save 内部认为的总帧数
        # 通常它等于我们传入的 frames=num_frames
        percent_done = int(((current_frame + 1) / total_frames_from_save) * 100)  # current_frame 从0开始

        # 只有当达到新的增量阈值时才打印
        # 或者当它是最后一帧时确保打印100%
        if percent_done >= self.last_reported_percent + self.update_increment_percent or \
                (current_frame + 1) == total_frames_from_save:

            # 确保百分比不超过100%
            actual_percent_to_print = min(percent_done, 100)

            # 使用 \r 和 end='' 来尝试在同一行更新进度
            # 注意：在某些IDE（如PyCharm的默认运行控制台）中，\r 可能不会按预期工作，
            # 而是每条都打印在新行。在标准终端中通常可以。
            # 为了更好的兼容性，我们这里选择直接打印新行。
            # 如果想尝试单行更新：
            # sys.stdout.write(f"\r保存动画进度: {actual_percent_to_print}% (帧 {current_frame + 1}/{total_frames_from_save})")
            # sys.stdout.flush()
            print(f"保存动画进度: {actual_percent_to_print}% (帧 {current_frame + 1}/{total_frames_from_save})")

            # 更新已报告的百分比，注意这里更新的是阈值，而不是实际打印的百分比
            # 例如，如果 increment 是 10，当达到 8% 时不打印，达到 12% 时打印 12%，下次报告点是 20%
            self.last_reported_percent = (
                                                     actual_percent_to_print // self.update_increment_percent) * self.update_increment_percent
            if (current_frame + 1) == total_frames_from_save:  # 如果是最后一帧
                self.last_reported_percent = 100  # 确保之后不会再因舍入问题打印


if __name__ == '__main__':
    grid_map_for_smooth = [
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [0, 1, 1, 0, 0, 0, 0, 1, 1, 0],
        [0, 0, 0, 0, 1, 0, 0, 0, 0, 0],
        [0, 0, 1, 0, 1, 0, 0, 1, 0, 0],
        [0, 0, 1, 0, 0, 0, 0, 1, 0, 0],
        [0, 0, 1, 1, 1, 0, 1, 1, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 1, 0],
        [0, 1, 1, 1, 1, 1, 1, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    ]
    start_pos_grid_smooth = (0, 0)

    current_grid, current_start = grid_map_for_smooth, start_pos_grid_smooth

    print("正在初始化平滑路径小车 (带碰撞检测和进度条)...")
    try:
        traversal_system = SmoothCurveCarTraversal(current_grid, current_start,
                                                   car_length=0.7, car_width=0.35,
                                                   spline_smoothness=0.0,
                                                   points_per_segment=10)  # 减少点以加快测试

        print("开始生成可视化动画...")
        traversal_system.visualize(filename="smooth_car_progress.mp4", interval=40)

    except ValueError as e:
        print(f"值错误: {e}")
    except Exception as e:
        import traceback

        print("发生意外错误:")
        traceback.print_exc()