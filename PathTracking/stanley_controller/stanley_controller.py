import numpy as np
import sys
import pathlib

"""

使用Stanley转向控制和PID速度控制的路径跟踪模拟。

作者：Atsushi Sakai (@Atsushi_twi)

参考：
    - [Stanley: The robot that won the DARPA grand challenge](http://isl.ecst.csuchico.edu/DOCS/darpa2005/DARPA%202005%20Stanley.pdf)
    - [Autonomous Automobile Path Tracking](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)

"""
import matplotlib.pyplot as plt
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))
print("PATH=" + str(pathlib.Path(__file__).parent.parent.parent)) # 打印路径

from utils.angle import angle_mod
from PathPlanning.CubicSpline import cubic_spline_planner

k = 0.5  # 控制增益
Kp = 1.0  # 速度比例增益
dt = 0.1  # [s] 时间差
L = 2.9  # [m] 车辆轴距
max_steer = np.radians(30.0)  # [rad] 最大转向角度

show_animation = False


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


def main():
    """在三次样条曲线上绘制Stanley转向控制的示例。"""
    # 目标路径
    ax = [0.0, 100.0, 100.0, 50.0, 60.0]
    ay = [0.0, 0.0, -30.0, -20.0, 0.0]

    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=0.1)

    target_speed = 30.0 / 3.6  # [m/s]

    max_simulation_time = 100.0

    # 初始状态
    state = State(x=-0.0, y=5.0, yaw=np.radians(20.0), v=0.0)

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
            plt.cla()
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


if __name__ == '__main__':
    main()
