# 2022中兴捧月-未来网络赛道

## 初赛一

>   赛题链接：https://zte.hina.com/zte/network
>
>   赛题解读：无人机在运动时，无人机网格相对静止，问题可抽象为最短路径问题。本算法考虑使用深度优先搜索算法，在固定的无人机网格中搜索所有可达路径中延时最短的路径。主要有如下几个流程：
>
>   1. 确定起始基站信号接收无人机 
>   2. 在无人机网格中使用深度优先搜索算法
>   3. 打印结果

由题可知，无人机坐标<img src="https://latex.codecogs.com/svg.image?UAV=(v&space;\times&space;t&space;&plus;&space;m&space;\times&space;d_{IntraOrbit},&space;n&space;\times&space;d_{InterOrbit},H)">，其中m,n为任意整数，可以视其为二维网格坐标。这里，我们假设起始基站的坐标为 <img src="https://latex.codecogs.com/svg.image?BS=(x,y,z)">。

则，在起始基站的附近的 UAV 的 (m,n) 坐标为：

<img src="https://latex.codecogs.com/svg.image?\begin{aligned}M&space;&=&space;\frac{(x&space;-&space;v&space;\times&space;time)}{d_{IntraOrbit}}&space;\\N&space;&=&space;\frac{y}{d_{InterOrbit}}\\UAV_{(m,n)}&space;&=&space;\{&space;(m,n)&space;|&space;m&space;\in&space;\{M-1,M,M&plus;1\},&space;n&space;\in&space;\{N,N&plus;1\}&space;\}\end{aligned}">

因此，从6个可能的起始无人机中选择满足题意要求的无人机发送信号即可。

对于一个无人机来说，总共有八种方向可以发送信号，但是只有向着终点基站方向的信号发送路径是有益的。因此，一个无人机的可选发送方向被缩减到三种。在进行深度搜索时，如果当前的时延已经大于已存在解，则放弃当前分支的搜索，以此来加快搜索速度。

## 初赛二

>   赛题理解：与初赛一不同的是，无人机的可选发送方向由原来的八种被缩减为四种，因为信号接受范围的减小使的无人机不能够斜着发送信号。如果条件允许的话，可以使用高空平台来实现发送信号给斜对角的无人机。除此之外，还需要考虑信道容量。本算法考虑使用折线路由算法。

- 使用哈希表结构记录每个信道的服务区间，以此判断该信道是否达到最大容量
- 使用正向折线路由算法和反向折线路由算法，选择其中时延最短的为最终解

## 运行环境说明


![](https://img.shields.io/badge/OS-windows、linux-brightgreen)
![](https://img.shields.io/badge/g++-8.1.0-orange)

```shell
cd result/algorithm
g++ solution.cpp -std=c++17 -o solution
./solution
```

