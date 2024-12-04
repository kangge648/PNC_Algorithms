import heapq
import matplotlib.pyplot as plt
import numpy as np

class Node:
    """节点类表示搜索树中的每一个点。"""
    def __init__(self, parent=None, position=None):
        self.parent = parent        # 该节点的父节点
        self.position = position    # 节点在迷宫中的坐标位置
        self.g = 0                  # G值：从起点到当前节点的成本
        self.h = 0                  # H值：当前节点到目标点的估计成本
        self.f = 0                  # F值：G值与H值的和，即节点的总评估成本

    # 比较两个节点位置是否相同
    def __eq__(self, other):
        return self.position == other.position

    # 定义小于操作，以便在优先队列中进行比较
    def __lt__(self, other):
        return self.f < other.f
    

def astar(maze, start, end):
    """A*算法实现，用于在迷宫中找到从起点到终点的最短路径。"""
    start_node = Node(None, start)  # 创建起始节点
    end_node = Node(None, end)      # 创建终点节点

    open_list = []                  # 开放列表用于存储待访问的节点
    closed_list = []                # 封闭列表用于存储已访问的节点

    heapq.heappush(open_list, (start_node.f, start_node))  # 将起始节点添加到开放列表
    print("添加起始节点到开放列表。")

    # 当开放列表非空时，循环执行
    while open_list:
        current_node = heapq.heappop(open_list)[1]  # 弹出并返回开放列表中 f 值最小的节点
        closed_list.append(current_node)            # 将当前节点添加到封闭列表
        print(f"当前节点: {current_node.position}")

        # 如果当前节点是目标节点，则回溯路径
        if current_node == end_node:
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            print("找到目标节点，返回路径。")
            return path[::-1]  # 返回反向路径，即从起点到终点的路径

        # 获取当前节点周围的相邻节点
        (x, y) = current_node.position
        neighbors = [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]

        # 遍历相邻节点
        for next in neighbors:
            # 确保相邻节点在迷宫范围内，且不是障碍物
            if 0 <= next[0] < maze.shape[0] and 0 <= next[1] < maze.shape[1]:
                if maze[next[0], next[1]] == 1:
                    continue
                neighbor = Node(current_node, next)  # 创建相邻节点
                # 如果相邻节点已在封闭列表中，跳过不处理
                if neighbor in closed_list:
                    continue
                neighbor.g = current_node.g + 1  # 计算相邻节点的 G 值
                neighbor.h = ((end_node.position[0] - next[0]) ** 2) + ((end_node.position[1] - next[1]) ** 2)  # 计算 H 值
                neighbor.f = neighbor.g + neighbor.h  # 计算 F 值

                # 如果相邻节点的新 F 值较小，则将其添加到开放列表
                if add_to_open(open_list, neighbor):
                    heapq.heappush(open_list, (neighbor.f, neighbor))
                    print(f"添加节点 {neighbor.position} 到开放列表。")
            else:
                print(f"节点 {next} 越界或为障碍。")

    return None  # 如果没有找到路径，返回 None

def add_to_open(open_list, neighbor):
    """检查并添加节点到开放列表。"""
    for node in open_list:
        # 如果开放列表中已存在相同位置的节点且 G 值更低，不添加该节点
        if neighbor == node[1] and neighbor.g > node[1].g:
            return False
    return True  # 如果不存在，则返回 True 以便添加该节点到开放列表

def visualize_path(maze, path, start, end):
    """将找到的路径可视化在迷宫上。"""
    maze_copy = np.array(maze)
    for step in path:
        maze_copy[step] = 0.5  # 标记路径上的点
    plt.figure(figsize=(10, 10))
    # 将迷宫中的通道显示为黑色，障碍物为白色
    plt.imshow(maze_copy, cmap='hot', interpolation='nearest')
    # 提取路径上的x和y坐标
    path_x = [p[1] for p in path]  # 列坐标
    path_y = [p[0] for p in path]  # 行坐标
    # 绘制路径
    plt.plot(path_x, path_y, color='orange', linewidth=2)
    # 绘制起点和终点
    start_x, start_y = start[1], start[0]
    end_x, end_y = end[1], end[0]
    plt.scatter([start_x], [start_y], color='green', s=100, label='Start', zorder=5)  # 起点为绿色圆点
    plt.scatter([end_x], [end_y], color='red', s=100, label='End', zorder=5)  # 终点为红色圆点
    # 添加图例
    plt.legend()
    # # 隐藏坐标轴
    # plt.axis('off')
    # 显示图像
    plt.show()

# 设定迷宫的尺寸
maze_size = 100

# 创建一个空的迷宫，全部设置为0（表示可通过）
maze = np.zeros((maze_size, maze_size))

# 定义障碍物，每个障碍物是一个矩形
obstacle_blocks = [
    (10, 10, 10, 10),  # (y起始, x起始, 高度, 宽度)
    (30, 40, 10, 10),
    (60, 20, 15, 10),
    (80, 50, 10, 25),
]

# 在迷宫中设置障碍物
for y_start, x_start, height, width in obstacle_blocks:
    maze[y_start:y_start+height, x_start:x_start+width] = 1

# 设定起始点和终点
start = (0, 0)
end = (92, 93)

# 确保起始点和终点不是障碍物
maze[start] = 0
maze[end] = 0

# 输出迷宫的一部分，以确认障碍物的设置
print("迷宫左上角10x10区域的视图:")
print(maze[:10, :10])

path = astar(maze, start, end)
if path:
    print("路径已找到：", path)
    visualize_path(maze, path, start, end)
else:
    print("没有找到路径。")

