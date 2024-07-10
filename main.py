import math
import numpy as np
import sys
import pathlib
from utils.angle import angle_mod
from PathPlanning.CubicSpline import cubic_spline_planner 
from PathPlanning.AStar.a_star import AStarPlanner
from PathPlanning.Dijkstra.dijkstra import Dijkstra
from PathTracking.stanley_controller.stanley_controller import stanley_control, pid_control
from PathTracking.pure_pursuit.pure_pursuit import pure_pursuit_steer_control, proportional_control
from PathTracking.pure_pursuit.pure_pursuit import States, plot_arrow,TargetCourse

import matplotlib.pyplot as plt
import matplotlib.pyplot as plt
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))
print("PATH=" + str(pathlib.Path(__file__).parent.parent.parent)) # 打印路径

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
        self.rear_x = self.x - ((L / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((L / 2) * math.sin(self.yaw))

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
        self.rear_x = self.x - ((L / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((L / 2) * math.sin(self.yaw))

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)

def main():
    print(__file__ + " start!!")

    # start and goal position
    sx = -5.0  # [m]
    sy = -5.0  # [m]
    gx = 50.0  # [m]
    gy = 50.0  # [m]
    grid_size = 2.0  # [m]
    robot_radius = 5.0  # 设置机器人半径

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

    rx, ry = [], []
    a_star_flag = False
    if(not a_star_flag):
        dijkstra = Dijkstra(ox, oy, grid_size, robot_radius)
        rx, ry = dijkstra.planning(sx, sy, gx, gy)
    
    if(a_star_flag):
        a_star = AStarPlanner(ox, oy, grid_size, robot_radius)
        rx, ry = a_star.planning(sx, sy, gx, gy)
    
    rx.reverse(), ry.reverse()

    # 目标路径
    cx, cy = rx, ry
    print("cx=", cx)
    # 根据路径rx ，ry生成样条曲线，计算 cyaw
    cyaw = [0]
    for i in range(1, len(cx) - 1):
        cyaw.append(np.arctan2(cy[i + 1] - cy[i - 1], cx[i + 1] - cx[i - 1]))
    cyaw.append(np.arctan2(cy[-1] - cy[-2], cx[-1] - cx[-2]))

    max_simulation_time = 100.0
    target_speed = 10.0 / 3.6  # [m/s]
    # 初始状态
    state = State(x=cx[0], y=cy[0], yaw=cyaw[0], v=0.0)
    global time,last_idx,x,y,yaw,v,t,target_idx
    last_idx = len(cx) - 1
    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    target_idx, _ = calc_target_index(state, cx, cy)

    def Stanley_main():
        global time, target_idx, last_idx
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
                plt.clf()
                plt.plot(ox, oy, ".k")
                plt.plot(sx, sy, "og")
                plt.plot(gx, gy, "xb")

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

    def pure_pursuit_main():
        #  target course
        last_idx = len(cx) - 1
        time = 0.0
        states = States()
        states.append(time, state)
        target_course = TargetCourse(cx, cy)
        target_ind, _ = calc_target_index(state, cx, cy)

        while max_simulation_time >= time and last_idx > target_ind:

            # Calc control input
            ai = proportional_control(target_speed, state.v)
            di, target_ind = pure_pursuit_steer_control(state, target_course, target_ind)

            state.update(ai, di)  # Control vehicle

            time += dt
            states.append(time, state)

            if show_animation:  # pragma: no cover
                plt.cla()
                plt.plot(ox, oy, ".k")
                plt.plot(sx, sy, "og")
                plt.plot(gx, gy, "xb")

                plt.gcf().canvas.mpl_connect(
                    'key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
                plot_arrow(state.x, state.y, state.yaw)
                plt.plot(cx, cy, "-r", label="course")
                plt.plot(states.x, states.y, "-b", label="trajectory")
                plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
                plt.axis("equal")
                plt.grid(True)
                plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
                plt.pause(0.001)

        # 测试
        assert last_idx >= target_ind, "无法到达目标点"

        if show_animation:  # pragma: no cover
            plt.cla()
            plt.plot(cx, cy, ".r", label="course")
            plt.plot(states.x, states.y, "-b", label="trajectory")
            plt.legend()
            plt.xlabel("x[m]")
            plt.ylabel("y[m]")
            plt.axis("equal")
            plt.grid(True)

            plt.subplots(1)
            plt.plot(states.t, [iv * 3.6 for iv in states.v], "-r")
            plt.xlabel("Time[s]")
            plt.ylabel("Speed[km/h]")
            plt.grid(True)
            plt.show()

    Stanley_main()
    # pure_pursuit_main() #

if __name__ == '__main__':
    main()
