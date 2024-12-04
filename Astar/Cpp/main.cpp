#include <iostream>
#include <vector>
#include <algorithm>
#include <queue>
#include <string>
#include <cmath>

#define N 10 // 地图的阶数

using namespace std;

typedef struct NODE
{
    int x, y;    // 节点位置
    int F, G, H; // G为离起点的距离；H为预估的离终点的距离；F = G + H代表总花费
    NODE(int a, int b) { x = a, y = b; }

    // 重载操作符，使优先队列以F值大小为标准维持堆（先比F再比G）
    bool operator<(const NODE &a) const
    {
        return F == a.F ? G > a.G : F > a.F;
    }
} Node;

// 定义方向
//const int next_position[8][2] = {{-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}};//8方向
const int next_position[4][2] = {{-1, 0}, {0, -1}, {0, 1}, {1, 0}};  //4方向
priority_queue<Node> open; // 优先队列，就相当于open表
// 地图
int map[N][N] = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                 {0, 1, 0, 0, 0, 0, 0, 1, 0, 0},
                 {0, 0, 1, 1, 0, 0, 0, 1, 0, 0},
                 {0, 0, 0, 1, 0, 0, 0, 1, 0, 0},
                 {0, 0, 0, 0, 0, 0, 1, 0, 0, 0},
                 {0, 0, 1, 1, 0, 1, 0, 0, 0, 0},
                 {0, 0, 1, 0, 1, 0, 1, 0, 0, 0},
                 {0, 0, 1, 0, 0, 0, 0, 1, 0, 0},
                 {0, 1, 0, 1, 0, 0, 0, 0, 0, 0},
                 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
bool close[N][N]; // 访问情况记录，close列表
int valueF[N][N]; // 记录每个节点对应的F值
int pre[N][N][2]; // 存储每个节点的父节点，方便找到终点后返回路径

int Manhattan(int x, int y, int x1, int y1)
{
    return (abs(x - x1) + abs(y - y1)) * 10;
}

bool isValidNode(int x, int y, int xx, int yy)
{
    if (x < 0 || x >= N || y < 0 || y >= N)
        return false; // 判断边界
    if (map[x][y] == 1)
        return false; // 判断障碍物
    if (x != xx && y != yy && (map[x][yy] == 1 || map[xx][y] == 1))
        return false;    // 两节点成对角型且它们的公共相邻节点存在障碍物，在8方向时用
    return true;
}

void Astar(int x0, int y0, int x1, int y1)
{
    // 起点加入open列表
    Node node(x0, y0);
    node.G = 0;
    node.H = Manhattan(x0, y0, x1, y1);
    node.F = node.G + node.H;
    valueF[x0][y0] = node.F;
    open.push(node);

    while (!open.empty())
    {
        Node node_current = open.top();                   // 取优先队列头元素，即周围单元格中代价最小的点
        open.pop();                                       // 从open列表中移除
        close[node_current.x][node_current.y] = true;     // 访问该点，加入close列表
        if (node_current.x == x1 && node_current.y == y1) // 到达终点
            break;

        // 遍历node_top周围的4个位置，如果是next_position有8，那么就需要遍历周围8个点
        for (int i = 0; i < 4; i++)
        {
            Node node_next(node_current.x + next_position[i][0], node_current.y + next_position[i][1]); // 创建一个node_top周围的点
            // 该节点坐标合法 且没有被访问
            if (isValidNode(node_next.x, node_next.y, node_current.x, node_current.y) && !close[node_next.x][node_next.y])
            {
                // 计算从起点并经过node_top节点到达该节点所花费的代价
                node_next.G = node_current.G + int(sqrt(pow(next_position[i][0], 2) + pow(next_position[i][1], 2)) * 10);
                // 计算该节点到终点的曼哈顿距离
                node_next.H = Manhattan(node_next.x, node_next.y, x1, y1);
                // 从起点经过node_top和该节点到达终点的估计代价
                node_next.F = node_next.G + node_next.H;

                // node_next.F < valueF[node_next.x][node_next.y] 说明找到了更优的路径，进行更新
                // valueF[node_next.x][node_next.y] == 0 说明该节点还未加入open表中，则加入
                if (node_next.F < valueF[node_next.x][node_next.y] || valueF[node_next.x][node_next.y] == 0)
                {
                    // 保存该节点的父节点
                    pre[node_next.x][node_next.y][0] = node_current.x;
                    pre[node_next.x][node_next.y][1] = node_current.y;
                    valueF[node_next.x][node_next.y] = node_next.F; // 修改该节点对应的valueF值
                    open.push(node_next);
                }
            }
        }
    }
}

void PrintPath(int x1, int y1)
{
    if (pre[x1][y1][0] == -1 || pre[x1][y1][1] == -1)
    {
        cout << "no path to get" << endl;
        return;
    }
    int x = x1, y = y1;
    int a, b;
    while (x != -1 || y != -1)
    {
        map[x][y] = 2; // 将可行路径上的节点赋值为2
        a = pre[x][y][0];
        b = pre[x][y][1];
        x = a;
        y = b;
    }
    // ' '表示未经过的节点， '#'表示障碍物， '@'表示可行节点
    string s[3] = {"  ", " #", " @"};
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
            cout << s[map[i][j]];
        cout << endl;
    }
}

int main(int argc, char *argv[])
{
    fill(close[0], close[0] + N * N, false);    // 将visit数组赋初值false
    fill(valueF[0], valueF[0] + N * N, 0);      // 初始化F全为0
    fill(pre[0][0], pre[0][0] + N * N * 2, -1); // 路径同样赋初值-1

    // 起点终点
    int x0 = 2, y0 = 4, x1 = 8, y1 = 6;

    // printf("input start: ");
    // scanf("%d%d", &x0, &y0);
    // printf("iinput destination: ");
    // scanf("%d%d", &x1, &y1);

    if (!isValidNode(x0, y0, x0, y0))
    {
        printf("Invalid input.\n");
        return 0;
    }

    Astar(x0, y0, x1, y1); // A*算法
    PrintPath(x1, y1);     // 打印路径

	return 0;
}
