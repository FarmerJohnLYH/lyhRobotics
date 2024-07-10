import matplotlib.pyplot as plt
import math
import numpy as np
import sys
import pathlib
import matplotlib.pyplot as plt
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))
print("PATH=" + str(pathlib.Path(__file__).parent.parent.parent)) # 打印路径
from utils.angle import angle_mod
from PathPlanning.CubicSpline import cubic_spline_planner


show_animation = True
k = 0.5  # 控制增益
Kp = 1.0  # 速度比例增益
dt = 0.1  # [s] 时间差
L = 2.9  # [m] 车辆轴距
max_steer = np.radians(30.0)  # [rad] 最大转向角度

def normalize_angle(angle):
    """
    将角度归一化到[-pi, pi]。
    :param angle: (float) 角度
    :return: (float) 归一化后的角度，范围在[-pi, pi]
    """
    return angle_mod(angle)
def calc_target_index(state, cx, cy):
    """
    计算目标点在轨迹列表中的索引。
    :param state: (State对象) 车辆状态
    :param cx: [float] x坐标列表
    :param cy: [float] y坐标列表
    :return: (int, float) 目标点的索引和前轴误差
    """
    # 计算前轴位置
    fx = state.x + L * np.cos(state.yaw)
    fy = state.y + L * np.sin(state.yaw)
    # 搜索最近点的索引
    dx = [fx - icx for icx in cx]
    dy = [fy - icy for icy in cy]
    d = np.hypot(dx, dy)
    target_idx = np.argmin(d)
    # 将RMS误差投影到前轴向量上
    front_axle_vec = [-np.cos(state.yaw + np.pi / 2),
                    -np.sin(state.yaw + np.pi / 2)]
    error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)
    return target_idx, error_front_axle
class State:
    """
    表示车辆状态的类。
    :param x: (float) x坐标
    :param y: (float) y坐标
    :param yaw: (float) 偏航角
    :param v: (float) 速度
    """
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        """实例化对象。"""
        super().__init__()
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
    def update(self, acceleration, delta):
        """
        更新车辆状态。
        Stanley控制使用自行车模型。
        :param acceleration: (float) 加速度
        :param delta: (float) 转向角
        """
        delta = np.clip(delta, -max_steer, max_steer)
        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt
        self.yaw += self.v / L * np.tan(delta) * dt
        self.yaw = normalize_angle(self.yaw)
        self.v += acceleration * dt
class Dijkstra:

    def __init__(self, ox, oy, resolution, robot_radius):
        """
        Initialize map for a star planning

        ox: x position list of Obstacles [m]
        oy: y position list of Obstacles [m]
        resolution: grid resolution [m]
        rr: robot radius[m]
        """

        self.min_x = None
        self.min_y = None
        self.max_x = None
        self.max_y = None
        self.x_width = None
        self.y_width = None
        self.obstacle_map = None

        self.resolution = resolution
        self.robot_radius = robot_radius
        self.calc_obstacle_map(ox, oy)
        self.motion = self.get_motion_model()

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index  # index of previous Node

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def planning(self, sx, sy, gx, gy):
        """
        dijkstra path search

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gx: goal x position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """

        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_index(start_node)] = start_node

        while True:
            c_id = min(open_set, key=lambda o: open_set[o].cost)
            current = open_set[c_id]

            # show graph
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_position(current.x, self.min_x),
                         self.calc_position(current.y, self.min_y), "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect(
                    'key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current.x == goal_node.x and current.y == goal_node.y:
                print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand search grid based on motion model
            for move_x, move_y, move_cost in self.motion:
                node = self.Node(current.x + move_x,
                                 current.y + move_y,
                                 current.cost + move_cost, c_id)
                n_id = self.calc_index(node)

                if n_id in closed_set:
                    continue

                if not self.verify_node(node):
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # Discover a new node
                else:
                    if open_set[n_id].cost >= node.cost:
                        # This path is the best until now. record it!
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_position(goal_node.x, self.min_x)], [
            self.calc_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_position(n.x, self.min_x))
            ry.append(self.calc_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    def calc_position(self, index, minp):
        pos = index * self.resolution + minp
        return pos

    def calc_xy_index(self, position, minp):
        return round((position - minp) / self.resolution)

    def calc_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_position(node.x, self.min_x)
        py = self.calc_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        if py < self.min_y:
            return False
        if px >= self.max_x:
            return False
        if py >= self.max_y:
            return False

        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, ox, oy):

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        print("min_x:", self.min_x)
        print("min_y:", self.min_y)
        print("max_x:", self.max_x)
        print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution) # self.resolution 是分辨率
        print("x_width:", self.x_width)
        print("y_width:", self.y_width)

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(self.x_width):
            x = self.calc_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.robot_radius:
                        self.obstacle_map[ix][iy] = True
                        break

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion


def main():
    print(__file__ + " start!!")

    # start and goal position
    sx = -5.0  # [m]
    sy = -5.0  # [m]
    gx = 50.0  # [m]
    gy = 50.0  # [m]
    grid_size = 2.0  # [m]
    robot_radius = 1.0  # [m]

    # set obstacle positions
    ox, oy = [], []
    for i in range(-10, 60):
        ox.append(i)
        oy.append(-10.0)
    for i in range(-10, 60):
        ox.append(60.0)
        oy.append(i)
    for i in range(-10, 61):
        ox.append(i)
        oy.append(60.0)
    for i in range(-10, 61):
        ox.append(-10.0)
        oy.append(i)
    for i in range(-10, 40):
        ox.append(20.0)
        oy.append(i)
    for i in range(0, 40):
        ox.append(40.0)
        oy.append(60.0 - i)

    if show_animation:  # pragma: no cover
        plt.plot(ox, oy, ".k")
        plt.plot(sx, sy, "og")
        plt.plot(gx, gy, "xb")
        plt.grid(True)
        plt.axis("equal")

    dijkstra = Dijkstra(ox, oy, grid_size, robot_radius)
    rx, ry = dijkstra.planning(sx, sy, gx, gy)
    rx.reverse(),ry.reverse()
    
    



    def pid_control(target, current):
        """
        速度的比例控制。

        :param target: (float) 目标速度
        :param current: (float) 当前速度
        :return: (float) 控制输出
        """
        return Kp * (target - current)


    def stanley_control(state, cx, cy, cyaw, last_target_idx):
        """
        Stanley转向控制。

        :param state: (State对象) 车辆状态
        :param cx: ([float]) x坐标列表
        :param cy: ([float]) y坐标列表
        :param cyaw: ([float]) 偏航角列表
        :param last_target_idx: (int) 上一个目标点的索引
        :return: (float, int) 转向角和当前目标点的索引
        """
        current_target_idx, error_front_axle = calc_target_index(state, cx, cy)

        if last_target_idx >= current_target_idx:
            current_target_idx = last_target_idx

        # theta_e修正航向误差
        theta_e = normalize_angle(cyaw[current_target_idx] - state.yaw)
        # theta_d修正横向偏差
        theta_d = np.arctan2(k * error_front_axle, state.v)
        # 转向控制
        delta = theta_e + theta_d

        return delta, current_target_idx




    def main():
        """在三次样条曲线上绘制Stanley转向控制的示例。"""
        # 目标路径
        cx,cy = rx,ry
        # 根据路径rx ，ry生成样条曲线，计算 cyaw
        cyaw = [0]
        for i in range(1, len(cx) - 1):
            cyaw.append(np.arctan2(cy[i + 1] - cy[i - 1], cx[i + 1] - cx[i - 1]))
        cyaw.append(np.arctan2(cy[-1] - cy[-2], cx[-1] - cx[-2]))


        target_speed = 20.0 / 3.6  # [m/s]

        max_simulation_time = 100.0

        # 初始状态
        state = State(x=cx[0], y=cy[0], yaw=cyaw[0], v=0.0)

        last_idx = len(cx) - 1
        time = 0.0
        x = [state.x]
        y = [state.y]
        yaw = [state.yaw]
        v = [state.v]
        t = [0.0]
        target_idx, _ = calc_target_index(state, cx, cy)

        while max_simulation_time >= time and last_idx > target_idx:
            ai = pid_control(target_speed, state.v)
            di, target_idx = stanley_control(state, cx, cy, cyaw, target_idx)
            state.update(ai, di)

            time += dt

            x.append(state.x)
            y.append(state.y)
            yaw.append(state.yaw)
            v.append(state.v)
            t.append(time)

            if show_animation:  # pragma: no cover
                # plt.cla() # 清除当前图形中的当前活动轴
                plt.clf() # 清除当前图形中的所有轴
                
                plt.plot(ox, oy, ".k")
                plt.plot(sx, sy, "og")
                plt.plot(gx, gy, "xb")

                # 通过按下esc键停止仿真。
                plt.gcf().canvas.mpl_connect('key_release_event',
                        lambda event: [exit(0) if event.key == 'escape' else None])
                plt.plot(cx, cy, ".r", label="course")
                plt.plot(x, y, "-b", label="trajectory")
                plt.plot(cx[target_idx], cy[target_idx], "xg", label="target")
                plt.axis("equal")
                plt.grid(True)
                plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
                plt.pause(0.001)

        # 测试
        assert last_idx >= target_idx, "无法到达目标点"

        if show_animation:  # pragma: no cover
            # 展示障碍物
            plt.plot(ox, oy, ".k")
            plt.plot(sx, sy, "og")
            plt.plot(gx, gy, "xb")
            plt.grid(True)
            plt.axis("equal")


            plt.plot(cx, cy, ".r", label="course")
            plt.plot(x, y, "-b", label="trajectory")
            plt.legend()
            plt.xlabel("x[m]")
            plt.ylabel("y[m]")
            plt.axis("equal")
            plt.grid(True)

            plt.subplots(1)
            plt.plot(t, [iv * 3.6 for iv in v], "-r")
            plt.xlabel("Time[s]")
            plt.ylabel("Speed[km/h]")
            plt.grid(True)
            plt.show()
    main()

if __name__ == '__main__':
    main()
