#include <iostream>
#include <vector>
#include <unordered_map>
#include <cmath>
using namespace std;

// #define DEBUG
#define INFO

struct result_node
{
    double time; // 时刻
    int m, n;    // 无人机序号
    result_node(double time, int m, int n) : time(time), m(m), n(n) {}
};

struct base_station
{
    int id;
    double x, y, z;
    base_station() {}
    base_station(int id, double x, double y, double z) : id(id), x(x), y(y), z(z) {}
};

struct uav_node
{
    double x, y, z;
    uav_node() {}
    uav_node(double x, double y, double z) : x(x), y(y), z(z) {}
};

class Solution
{
private:
    const double v;       // 无人机速度
    const double H;       // 无人机高度
    const double d_intra; // 无人机间距
    const double d_inter; // 轨道行距
    const double d;       // 无人机通信距离
    const double D;       // 基站通信距离
    const double t_f;     // 转发延时固定常数

public:
    unordered_map<int, base_station> BS; // 基站
    vector<result_node> result;          // 分配方案
    double delay;                        // 总时延
#ifdef INFO
    double TOTAL_DELAY; // 所有方案的总时延，用于线下比较
#endif

private:
    /**
     * @brief 打印该组请求结果集
     *
     * @param time 时刻
     * @param st_bs_id 起点BS标号
     * @param ed_bs_id 终点BS标号
     */
    void print_ans(double time, int st_bs_id, int ed_bs_id)
    {
#ifdef DEBUG
        printf("\nRESULT:\n");
#endif
        printf("%.4lf,%d,%d,%.4lf,1\n", time, st_bs_id, ed_bs_id, this->delay);
        for (int i = 0; i < this->result.size(); i++)
        {
            printf("(%.4lf,%d,%d)", this->result[i].time, this->result[i].m, this->result[i].n);
            if (i != this->result.size() - 1)
                printf(",");
            else
                printf("\n");
        }
    }

    /**
     * @brief 获取两个基站之间发送和接收航线，
     * 航线为 int * d_inter
     *
     * @param st_y 起点BS
     * @param ed_y 终点BS
     * @return pair<int, int>
     */
    pair<int, int> get_routes(double st_y, double ed_y)
    {
        int st_route = st_y / this->d_inter;
        int ed_route = ed_y / this->d_inter;
        // // 选择距离最小的那个
        if ((2 * st_route + 1) * this->d_inter <= 2 * st_y)
            st_route++;
        if ((2 * ed_route + 1) * this->d_inter <= 2 * ed_y)
            ed_route++;
        return {st_route, ed_route};
    }

    /**
     * @brief 获取UAV
     *
     * @param time 时间
     * @param m
     * @param n
     * @return uav
     */
    uav_node get_uav(double time, int m, int n)
    {
        return uav_node(this->v * time + m * this->d_intra, n * this->d_inter, this->H);
    }

    /**
     * @brief 获取节点之间的欧式距离
     *
     * @return double
     */
    static double get_distance(double st_x, double st_y, double st_z, double ed_x, double ed_y, double ed_z)
    {
        return sqrt(pow(st_x - ed_x, 2) + pow(st_y - ed_y, 2) + pow(st_z - ed_z, 2));
    }

    /**
     * @brief 获取节点之间的欧式距离
     *
     * @param bs 基站
     * @param uav 无人机
     * @return double
     */
    static double get_distance(base_station &bs, uav_node uav)
    {
        return sqrt(pow(bs.x - uav.x, 2) + pow(bs.y - uav.y, 2) + pow(bs.z - uav.z, 2));
    }

    /**
     * @brief 获取节点之间的欧式距离
     *
     * @param uav_1 无人机
     * @param uav_2 无人机
     * @return double
     */
    static double get_distance(uav_node uav_1, uav_node uav_2)
    {
        return sqrt(pow(uav_1.x - uav_2.x, 2) + pow(uav_1.y - uav_2.y, 2));
    }

    /**
     * @brief 计算转发时延
     *
     * @param S 距离
     * @return double
     */
    double get_trans_delay(double S)
    {
        return this->t_f + S / 10000;
    }

    pair<int, int> send_st(double &time, base_station &st_bs, base_station &ed_bs, int route)
    {
        int m = (st_bs.x - this->v * time) / this->d_intra;
        // 在 m-1, m, m+1 中寻找与终点基站最近的可行点
        int min_m;
        double min_delay;
        double min_dis = 0x7fffffff;
        for (int i = m - 1; i <= m + 1; i++)
        {
            double dis = get_distance(st_bs, get_uav(time, i, route));
            double delay = get_trans_delay(dis);
            if (get_distance(st_bs, get_uav(time + delay, i, route)) > this->D)
                continue;
            double dis_ed = get_distance(ed_bs, get_uav(time + delay, i, route));
            if (dis_ed < min_dis)
            {
                min_dis = dis_ed;
                min_m = i;
                min_delay = delay;
            }
        }
        // 如果没有找出最小值，则报错
        if (min_dis == 0x7fffffff)
            throw "Can not find the send UAV";
#ifdef DEBUG
        printf("send_st: (%d,%d,%lf) %lf\n", min_m, route, this->H, min_delay);
#endif
        // 添加到结果集
        this->delay += min_delay;
        time += min_delay;
        this->result.push_back(result_node(time, min_m, route));
        return {min_m, route};
    }

public:
    Solution(double v, double H, double d_intra, double d_inter, double d, double D, double t_f, unordered_map<int, base_station> &BS)
        : v(v), H(H), d_intra(d_intra), d_inter(d_inter), d(d), D(D), t_f(t_f)
    {
        this->BS = BS;
#ifdef DEBUG
        printf("Parameters:\n");
        printf("v:%.1lf, H:%.1lf, d_intra:%.1lf, d_inter:%.1lf, d:%.1lf, D:%.1lf, t_f:%.1lf\n", v, H, d_intra, d_inter, d, D, t_f);
        printf("Base Station:\n");
        for (const auto &[id, bs] : BS)
            printf("id:%d, (%.2lf, %.2lf, %.2lf)\n", bs.id, bs.x, bs.y, bs.z);
#endif
    }

    /**
     * @brief 计算分配方案
     *
     * @param time 时刻
     * @param st_bs_id 起点BS标号
     * @param ed_bs_id 终点BS标号
     */
    void calculate(double time, int st_bs_id, int ed_bs_id)
    {
        // 清空结果集
        this->result.clear();
        this->delay = 0;
        double time_bak = time;

        // 1. 获取相向的航线
        auto [st_route, ed_route] = get_routes(this->BS[st_bs_id].y, this->BS[ed_bs_id].y);
#ifdef DEBUG
        printf("\ncalculate: %.4lf %d-->%d\n", time, st_bs_id, ed_bs_id);
        printf("get_routes: %.2lf-->%.2lf == %d-->%d\n", this->BS[st_bs_id].y, this->BS[ed_bs_id].y, st_route, ed_route);
#endif
        // 2. 发送到st_route
        auto [st_m, st_n] = send_st(time, this->BS[st_bs_id], this->BS[ed_bs_id], st_route);

        // 3. 从st节点发送到ed_route上，沿着对角线
        int now_m = st_m;
        int now_n = st_n;
        bool flg = false; // 是否中途就可以直接发
        while (now_n != ed_route)
        {
            uav_node uav = get_uav(time, now_m, now_n);
            if (uav.x <= this->BS[ed_bs_id].x && uav.y <= this->BS[ed_bs_id].y)
            {
                if (this->BS[ed_bs_id].x - uav.x >= this->d_intra && this->BS[ed_bs_id].y - uav.y <= this->d_inter)
                    now_m++;
                else if (this->BS[ed_bs_id].y - uav.y >= this->d_inter && this->BS[ed_bs_id].x - uav.x <= this->d_intra)
                    now_n++;
                else
                    now_m++, now_n++;
            }
            else if (uav.x <= this->BS[ed_bs_id].x && uav.y >= this->BS[ed_bs_id].y)
            {
                if (this->BS[ed_bs_id].x - uav.x >= this->d_intra && uav.y - this->BS[ed_bs_id].y <= this->d_inter)
                    now_m++;
                else if (uav.y - this->BS[ed_bs_id].y >= this->d_inter && this->BS[ed_bs_id].x - uav.x <= this->d_intra)
                    now_n--;
                else
                    now_m++, now_n--;
            }
            else if (uav.x >= this->BS[ed_bs_id].x && uav.y <= this->BS[ed_bs_id].y)
            {
                if (uav.x - this->BS[ed_bs_id].x >= this->d_intra && this->BS[ed_bs_id].y - uav.y <= this->d_inter)
                    now_m--;
                else if (this->BS[ed_bs_id].y - uav.y >= this->d_inter && uav.x - this->BS[ed_bs_id].x <= this->d_intra)
                    now_n++;
                else
                    now_m--, now_n++;
            }
            else if (uav.x >= this->BS[ed_bs_id].x && uav.y >= this->BS[ed_bs_id].y)
            {
                if (uav.x - this->BS[ed_bs_id].x >= this->d_intra && uav.y - this->BS[ed_bs_id].y <= this->d_inter)
                    now_m--;
                else if (uav.y - this->BS[ed_bs_id].y >= this->d_inter && uav.x - this->BS[ed_bs_id].x <= this->d_intra)
                    now_n--;
                else
                    now_m--, now_n--;
            }
            else
                break;
#ifdef DEBUG
            uav_node uav2 = get_uav(time, now_m, now_n);
            printf("send_ed_route: (%d,%d)(%.1lf, %.1lf)-->%d(%.1lf, %.1lf)\n", now_m, now_n, uav2.x, uav2.y, ed_route, this->BS[ed_bs_id].x, this->BS[ed_bs_id].y);
#endif
            double dis = get_distance(uav, get_uav(time, now_m, now_n));
            double delay = get_trans_delay(dis);
            this->delay += delay;
            time += delay;
            this->result.push_back(result_node(time, now_m, now_n));
            // 如果该点可以直接传信息给终点基站，则不要在循环了
            if (get_distance(this->BS[ed_bs_id], get_uav(time, now_m, now_n)) <= this->D)
            {
                double _delay = get_trans_delay(get_distance(this->BS[ed_bs_id], get_uav(time, now_m, now_n)));
                if (get_distance(this->BS[ed_bs_id], get_uav(time + _delay, now_m, now_n)) <= this->D)
                {
                    this->delay += _delay;
                    time += _delay;
                    flg = true;
                    break;
                }
            }
        }

        // 4. 从now_route传送到ed_bs_id
        while (!flg && true)
        {
            double dis = get_distance(this->BS[ed_bs_id], get_uav(time, now_m, now_n));
            double delay = get_trans_delay(dis);
            if (get_distance(this->BS[ed_bs_id], get_uav(time + delay, now_m, now_n)) <= this->D)
            {
                this->delay += delay;
                time += delay;
                break;
            }
            else
            {
                if (this->v * time + now_m * this->d_intra < this->BS[ed_bs_id].x)
                    now_m++;
                else
                    now_m--;
#ifdef DEBUG
                printf("send_bs: %d\n", now_m);
#endif
                delay = this->t_f + this->d_intra / 10000;
                this->delay += delay;
                time += delay;
                this->result.push_back(result_node(time, now_m, now_n));
            }
        }

        // 4. 打印结果
        print_ans(time_bak, st_bs_id, ed_bs_id);

#ifdef INFO
        this->TOTAL_DELAY += this->delay;
#endif
    }
};

int main()
{
    // 基站信息
    unordered_map<int, base_station> BS = {
        {0, base_station(0, 45.73, 45.26, 0)},
        {1, base_station(1, 1200, 700, 0)},
        {2, base_station(2, -940, 1100, 0)}};
    // 解决方案
    Solution solution(5, 10, 90, 80, 125, 70, 0.1, BS);
    solution.calculate(0, 0, 1);
    solution.calculate(0, 0, 2);
    solution.calculate(4.7, 0, 1);
    solution.calculate(4.7, 0, 2);
    solution.calculate(16.4, 0, 1);
    solution.calculate(16.4, 0, 2);

#ifdef INFO
    cout << "TOTAL: " << solution.TOTAL_DELAY << endl;
#endif

    return 0;
}