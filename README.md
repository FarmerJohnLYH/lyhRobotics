# 题目
设计一个场景，包括起点，终点，障碍物，采用2种及以上的规划方法实现规划过程，采用2种及以上的横向控制方法实现路径跟踪过程，要求实现模拟移动机器人运动的过程，机器人本体的运动控制模型不限。

# Requirements
对于运行示例代码：

- [Python 3.12.x](https://www.python.org/)

- [NumPy](https://numpy.org/)

- [SciPy](https://scipy.org/)

- [Matplotlib](https://matplotlib.org/)

- [cvxpy](https://www.cvxpy.org/)

开发所需：

- [pytest](https://pytest.org/)（用于单元测试）

- [pytest-xdist](https://pypi.org/project/pytest-xdist/)（用于并行单元测试）

- [mypy](http://mypy-lang.org/)（用于类型检查）

- [sphinx](https://www.sphinx-doc.org/)（用于文档生成）

- [pycodestyle](https://pypi.org/project/pycodestyle/)（用于代码风格检查）

可直接通过 `pip install -r requirements.txt` 安装所需的包


# 路径规划
## 基于网格的搜索

### Dijkstra算法

这是一个基于2D网格的最短路径规划，使用Dijkstra算法。
Dijkstra算法是一种用于寻找图中两点之间最短路径的算法。它由荷兰计算机科学家艾兹格·戴克斯特拉（Edsger W. Dijkstra）在1956年提出，并于1959年发表。该算法可以应用于有向图和无向图，但所有边的权重必须为非负值，因为Dijkstra算法基于这样的前提：一条路径的长度随着路径上的边数增加而增加。

### A\*算法

这是一个基于2D网格的最短路径规划，使用A\*算法。
A\* 算法是一种在图形平面上，有多个节点的路径中，寻找一条从起始点到目标点的最短路径的算法。它由彼得·哈特（Peter Hart）、尼尔斯·尼尔森（Nils Nilsson）和伯特拉姆·拉斐尔（Bertram Raphael）于1968年提出，是一种启发式搜索算法，用于解决最短路径问题。A\*算法结合了最佳优先搜索和迪杰斯特拉算法的优点，通过评估函数f(n) = g(n) + h(n)来选择路径，其中g(n)是起点到当前节点的实际成本，h(n)是当前节点到目标的估计成本。A\*算法在寻找最短路径的过程中，能够以较高的效率避开障碍，找到最优路径。

其启发式函数为2D欧几里德距离。

# 路径跟踪

## Stanley控制

使用Stanley转向控制和PID速度控制的路径跟踪模拟。
Stanley控制是一种用于自动驾驶车辆的路径跟踪控制方法。它由斯坦福大学的研究团队在参加2005年DARPA大挑战赛时提出。Stanley控制方法的核心思想是最小化车辆的横向误差（即车辆与路径的垂直距离）和航向角误差（即车辆行驶方向与路径切线方向之间的角度差）。这种方法通过调整车辆的转向角来实现路径跟踪，特别适合于处理直线和平滑曲线路径。


参考：

- [Stanley: The robot that won the DARPA grand challenge](http://robots.stanford.edu/papers/thrun.stanley05.pdf)

- [Automatic Steering Methods for Autonomous Automobile Path Tracking](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)


## 纯追踪

使用纯追踪转向控制和PID速度控制的路径跟踪模拟。
纯追踪（Pure Pursuit）转向控制是一种常用于自动驾驶和机器人导航的路径跟踪算法。它的基本原理是虚拟一个目标点在车辆前方的路径上，然后控制车辆朝这个目标点行驶。通过调整目标点的位置（通常与车速成比例），该方法可以动态调整车辆的转向角度，以平滑地跟踪路径。纯追踪算法简单直观，易于实现，适用于多种行驶环境。

参考：

- [A Survey of Motion Planning and Control Techniques for Self-driving Urban Vehicles](https://arxiv.org/abs/1604.07446)
