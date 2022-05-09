#include <iostream>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <string>
#include <algorithm>
using namespace std;

// #define DEBUG
// #define INFO

struct result_node
{
    double time; // 时刻
    bool type;   // 类型 0:无人机, 1:高空平台
    int m, n;    // 无人机序号
    int l;       // 高空平台序号
    result_node(double time, int m, int n) : time(time), m(m), n(n), type(0) {}
    result_node(double time, int l) : time(time), l(l), type(1) {}
    void print()
    {
        if (this->type == 0)
            printf("(%.4lf,%d,%d)", this->time, this->m, this->n);
        else
            printf("(%.4lf,%d)", this->time, this->l);
    }
};

struct position
{
    double x, y, z;
    position() {}
    position(double x, double y, double z) : x(x), y(y), z(z) {}
};

struct base_station : public position
{
    int id;
    base_station() {}
    base_station(int id, double x, double y, double z) : position(x, y, z), id(id) {}
    pair<int, int> get_coord()
    {
        return {40000, this->id};
    }
};

struct ha_platform : public position
{
    int id;
    ha_platform() {}
    ha_platform(int id, double x, double y, double z) : position(x, y, z), id(id) {}
    pair<int, int> get_coord()
    {
        return {50000, this->id};
    }
};

struct uav_node : public position
{
    int m, n;
    uav_node() {}
    uav_node(double x, double y, double z) : position(x, y, z) {}
};

struct route_node
{
    vector<pair<double, double>> service; // 服务时间
    route_node() {}
    route_node(double st, double ed)
    {
        this->service.push_back({st, ed});
    }
};

typedef pair<pair<int, int>, pair<int, int>> ROUTE_KEY;

struct route_hash
{
    size_t operator()(const ROUTE_KEY &k) const
    {
        auto h1 = hash<int>()(k.first.first);
        auto h2 = hash<int>()(k.first.second);
        auto h3 = hash<int>()(k.second.first);
        auto h4 = hash<int>()(k.second.second);
        return (h1 & h3) ^ (h2 & h4);
    }
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
    const int c;          // 最大载荷
    const int s;          // 同时转发信号数

public:
    unordered_map<int, base_station> BS;                    // 基站
    unordered_map<int, ha_platform> HAPS;                   // 高空平台
    unordered_map<ROUTE_KEY, route_node, route_hash> ROUTE; // 服务列表
    vector<result_node> result;                             // 分配方案
    double delay;                                           // 总时延
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
     * @param delay 总时延
     * @param load 路径载荷
     * @param result 结果集
     */
    static void print_ans(double time, int st_bs_id, int ed_bs_id, double delay, int load, vector<result_node> &result)
    {
#ifdef DEBUG
        printf("\nRESULT:\n");
#endif
        printf("%.4lf,%d,%d,%.4lf,%d\n", time, st_bs_id, ed_bs_id, delay, load);
        for (int i = 0; i < result.size(); i++)
        {
            result[i].print();
            if (i != result.size() - 1)
                printf(",");
            else
                printf("\n");
        }
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
     * @param bs 基站
     * @param uav 无人机
     * @return double
     */
    static double get_distance(position A, position B)
    {
        return sqrt(pow(fabs(A.x - B.x), 2) + pow(fabs(A.y - B.y), 2) + pow(fabs(A.z - B.z), 2));
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

    double get_wait_time(int m, int n, base_station &ed)
    {
        double A = sqrt(pow(this->D, 2) - pow(this->H - ed.z, 2) - pow(this->d_inter * n - ed.y, 2));
        double B1 = (ed.x - this->d_intra * m + A) / this->v;
        double B2 = (ed.x - this->d_intra * m - A) / this->v;
        double ans = 10;
        if (B1 >= 0)
            ans = min(ans, B1);
        if (B2 >= 0)
            ans = min(ans, B2);
        return ans;
    }

    /**
     * @brief 确定起始基站发送的第一个无人机
     *
     * @param time 当前时刻
     * @param st_bs 起始基站
     * @param ed_bs 终点基站
     * @param route 确定的起始行
     * @return pair<int, int>
     */
    pair<int, int> send_st(double &time, base_station &st_bs, base_station &ed_bs)
    {
        int m = (st_bs.x - this->v * time) / this->d_intra;
        int n = st_bs.y / this->d_inter;
        // 在 m-1, m, m+1 中寻找与终点基站最近的可行点
        // 1. 当前时刻可以找到满足要求的解
        // 需要计算当前不可用的无人机或信道的等待时间
        // 2. 如果没有解，那么要考虑下一个无人机飞进发送范围的时间是否在wait_time内，如果超过，就自动延迟wait_time
        // wait_time由可用无人机中最短空闲时间决定
        // 可以再次调用本函数 return send_st_(time + wait_time, st_bs, ed_bs, route)
        vector<pair<int, int>> candidate_uav = {{m - 1, n}, {m, n}, {m + 1, n}, {m - 1, n + 1}, {m, n + 1}, {m + 1, n + 1}};
        bool flg = 1; // 标记是否需要进入第二阶段
        int select_m, select_n;
        double select_delay;
        double select_dis = 0x7fffffff;
        double wait_time = 0.12; // 任何一次发送等待0.12s一定不会冲突
        // 在候选无人机中寻找可行解
        for (auto &[m, n] : candidate_uav)
        {
            double dis = get_distance(st_bs, get_uav(time, m, n));
            double delay = get_trans_delay(dis);
            // 如果信道被占用，则计算等待时间并更新全局等待时间
            if (is_occupy(st_bs.get_coord(), {m, n}, {time, time + delay}))
            {
                wait_time = min(wait_time, get_occupy_free(st_bs.get_coord(), {m, n}, time));
                continue;
            }
            // 如果超出发信范围，则计算等待时间并更新全局等待时间
            if (get_distance(st_bs, get_uav(time, m, n)) > this->D)
            {
                wait_time = min(wait_time, get_wait_time(m, n, st_bs));
                continue;
            }
            // 至少是个可行解,但不一定是最优解
            flg = 0;
            double dis_ed = get_distance(ed_bs, get_uav(time + delay, m, n));
            if (dis_ed <= select_dis)
            {
                select_dis = dis_ed;
                select_m = m;
                select_n = n;
                select_delay = delay;
            }
        }
        // 如果没有找到可行解,则需要等待一段时间
        if (flg)
        {
            // 递归调用该方法
            time += wait_time;
            this->delay += wait_time;
            return send_st(time, st_bs, ed_bs);
        }
#ifdef DEBUG
        printf("send_st: (%d,%d,%lf) %lf\n", select_m, select_n, this->H, select_delay);
#endif
        // 添加到结果集
        update_ROUTE(st_bs.get_coord(), {select_m, select_n}, {time, time + select_delay});
        this->delay += select_delay;
        time += select_delay;
        this->result.push_back(result_node(time, select_m, select_n));
        return {select_m, select_n};
    }

    /**
     * @brief 检查该信道在该请求时间片是否占用
     *
     * @param st_node 信道起点
     * @param ed_node 信道终点
     * @param request 请求时间片
     * @return bool 是否被占用
     */
    bool is_occupy(pair<int, int> st_node, pair<int, int> ed_node, pair<double, double> request)
    {
        ROUTE_KEY key = {st_node, ed_node};
        if (this->ROUTE.find(key) == this->ROUTE.end())
            return false;
        for (auto &[st, ed] : this->ROUTE[key].service)
            if (request.second > st && request.first < ed)
                return true;
        return false;
    }

    /**
     * @brief 更新该信道的服务区间
     *
     * @param st_node 信道起点
     * @param ed_node 信道终点
     * @param request 请求时间片
     */
    void update_ROUTE(pair<int, int> st_node, pair<int, int> ed_node, pair<double, double> request)
    {
        if (is_occupy(st_node, ed_node, request))
            printf("\n!!!!!!!!!!\nERROR: (%d,%d)<-->(%d,%d):(%.4lf,%.4lf) ROUTE is occupied\n!!!!!!!!!!\n\n", st_node.first, st_node.second, ed_node.first, ed_node.second, request.first, request.second);
        this->ROUTE[{st_node, ed_node}].service.push_back(request);
        this->ROUTE[{ed_node, st_node}].service.push_back(request);
    }

    /**
     * @brief 获取该信道到达空闲时间的等待时间
     *
     * @param st_node 信道起点
     * @param ed_node 信道终点
     * @param time 当前时刻
     * @return double 等待时间
     */
    double get_occupy_free(pair<int, int> st_node, pair<int, int> ed_node, double time)
    {
        double ans = 10;
        ROUTE_KEY key = {st_node, ed_node};
        for (auto &[st, ed] : this->ROUTE[key].service)
        {
            double tmp = ed - time;
            ans = tmp > 0 ? min(ans, tmp) : ans;
        }
        return ans;
    }

public:
    Solution(double v, double H, double d_intra, double d_inter, double d, double D, double t_f, int c, int s, unordered_map<int, base_station> &BS, unordered_map<int, ha_platform> &HAPS)
        : v(v), H(H), d_intra(d_intra), d_inter(d_inter), d(d), D(D), t_f(t_f), c(c), s(s)
    {
        this->BS = BS;
        this->HAPS = HAPS;
#ifdef DEBUG
        printf("Parameters:\n");
        printf("v:%.1lf, H:%.1lf, d_intra:%.1lf, d_inter:%.1lf, d:%.1lf, D:%.1lf, t_f:%.1lf\n", v, H, d_intra, d_inter, d, D, t_f);
        printf("c:%d s:%d\n", c, s);
        printf("Base Station:\n");
        for (const auto &[id, bs] : BS)
            printf("id:%d, (%.2lf, %.2lf, %.2lf)\n", bs.id, bs.x, bs.y, bs.z);
        printf("HA Platform:\n");
        for (const auto &[id, haps] : HAPS)
            printf("id:%d, (%.2lf, %.2lf, %.2lf)\n", haps.id, haps.x, haps.y, haps.z);
        printf("\n");
#endif
    }

    /**
     * @brief 计算分配方案
     *
     * @param time 时刻
     * @param st_bs_id 起点BS标号
     * @param ed_bs_id 终点BS标号
     */
    void calculate(double time, int st_bs_id, int ed_bs_id, int load)
    {
        // 清空结果集
        this->result.clear();
        this->delay = 0;
        double time_bak = time;

        // 1. 选择发送起点
        auto [st_m, st_n] = send_st(time, this->BS[st_bs_id], this->BS[ed_bs_id]);

        // 2. 方法一发送
        double DELAY_BAK = this->delay;
        vector<result_node> RESULT_BAK(this->result);
        unordered_map<ROUTE_KEY, route_node, route_hash> ROUTE_BAK(this->ROUTE);
        calculate_part_1(time, st_m, st_n, st_bs_id, ed_bs_id, load);
        double delay_part_1 = this->delay;
        vector<result_node> result_part_1(this->result);
        unordered_map<ROUTE_KEY, route_node, route_hash> ROUTE_part_1(this->ROUTE);

        // 还原
        this->delay = DELAY_BAK;
        this->ROUTE = unordered_map<ROUTE_KEY, route_node, route_hash>(ROUTE_BAK);
        this->result = vector<result_node>(RESULT_BAK);

        // 3. 方法二发送
        calculate_part_2(time, st_m, st_n, st_bs_id, ed_bs_id, load);
        if (delay_part_1 < this->delay)
        {
            this->delay = delay_part_1;
            this->result = vector<result_node>(result_part_1);
            this->ROUTE = unordered_map<ROUTE_KEY, route_node, route_hash>(ROUTE_part_1);
        }

        // 4. 打印结果
        print_ans(time_bak, st_bs_id, ed_bs_id, this->delay, load, this->result);

#ifdef INFO
        this->TOTAL_DELAY += this->delay * load;
#endif
    }

    void calculate_part_1(double time, int m, int n, int st_bs_id, int ed_bs_id, int load)
    {
        int now_m = m;
        int now_n = n;
        bool flg = false; // 是否中途就可以直接发
        while (!flg)
        {
            uav_node uav = get_uav(time, now_m, now_n);
            vector<pair<int, int>> candidate_uav; // 候选终点uav,{now_m,now_n}->{candidate_m,candidate_n}
            bool is_need_HAPS = false;            // 是否需要HAPS,要求在对角线请求中存在一个HAPS
            int dia_m = now_m, dia_n = now_n;     // 如果使用HAPS的话，终点坐标
            if (uav.x <= this->BS[ed_bs_id].x && uav.y <= this->BS[ed_bs_id].y)
            {
                candidate_uav.push_back({now_m + 1, now_n});
                candidate_uav.push_back({now_m, now_n + 1});
                if (fabs(this->BS[ed_bs_id].x - uav.x) >= this->d_intra && fabs(this->BS[ed_bs_id].y - uav.y) >= this->d_inter)
                {
                    is_need_HAPS = true;
                    dia_m++;
                    dia_n++;
                }
            }
            else if (uav.x <= this->BS[ed_bs_id].x && uav.y >= this->BS[ed_bs_id].y)
            {
                candidate_uav.push_back({now_m + 1, now_n});
                candidate_uav.push_back({now_m, now_n - 1});
                if (fabs(this->BS[ed_bs_id].x - uav.x) >= this->d_intra && fabs(this->BS[ed_bs_id].y - uav.y) >= this->d_inter)
                {
                    is_need_HAPS = true;
                    dia_m++;
                    dia_n--;
                }
            }
            else if (uav.x >= this->BS[ed_bs_id].x && uav.y <= this->BS[ed_bs_id].y)
            {
                candidate_uav.push_back({now_m - 1, now_n});
                candidate_uav.push_back({now_m, now_n + 1});
                if (fabs(this->BS[ed_bs_id].x - uav.x) >= this->d_intra && fabs(this->BS[ed_bs_id].y - uav.y) >= this->d_inter)
                {
                    is_need_HAPS = true;
                    dia_m--;
                    dia_n++;
                }
            }
            else if (uav.x >= this->BS[ed_bs_id].x && uav.y >= this->BS[ed_bs_id].y)
            {
                candidate_uav.push_back({now_m - 1, now_n});
                candidate_uav.push_back({now_m, now_n - 1});
                if (fabs(this->BS[ed_bs_id].x - uav.x) >= this->d_intra && fabs(this->BS[ed_bs_id].y - uav.y) >= this->d_inter)
                {
                    is_need_HAPS = true;
                    dia_m--;
                    dia_n--;
                }
            }
            // 判断从(now_m,now_n)->(sel_m,sel_n)之间是否存在HAPS
            // 如果存在，需要转发两次
            bool is_send_HAPS = false;
            if (is_need_HAPS)
            {
                for (auto &[id, hap] : this->HAPS)
                {
                    double dis_st = get_distance(get_uav(time, now_m, now_n), hap);
                    double delay_st = get_trans_delay(dis_st);
                    double dis_st_after = get_distance(get_uav(time + delay_st, now_m, now_n), hap);
                    if (dis_st > this->d || dis_st_after > this->d)
                        continue;
                    double dis_ed = get_distance(get_uav(time + delay_st, dia_m, dia_n), hap);
                    double delay_ed = get_trans_delay(dis_ed);
                    double dis_ed_after = get_distance(get_uav(time + delay_st + delay_ed, dia_m, dia_n), hap);
                    if (dis_ed > this->d || dis_ed_after > this->d)
                        continue;
                    // 距离比直接发送要远
                    if (dis_st + dis_ed >= this->d_inter + this->d_intra)
                        continue;
                    // 信道占用
                    if (is_occupy({now_m, now_n}, this->HAPS[id].get_coord(), {time, time + delay_st}) || is_occupy(this->HAPS[id].get_coord(), {dia_m, dia_n}, {time + delay_st, time + delay_st + delay_ed}))
                        continue;
                    // 可以
                    is_send_HAPS = true;
                    update_ROUTE({now_m, now_n}, this->HAPS[id].get_coord(), {time, time + delay_st});
                    this->delay += delay_st;
                    time += delay_st;
                    this->result.push_back(result_node(time, id));
                    update_ROUTE(this->HAPS[id].get_coord(), {dia_m, dia_n}, {time, time + delay_ed});
                    this->delay += delay_ed;
                    time += delay_ed;
                    this->result.push_back(result_node(time, dia_m, dia_n));
                    break;
                }
            }
            if (!is_send_HAPS)
            {
                // 从候选节点中，寻找与终点最近的那个
                // 如果信道被占用则，等待wait_time时间,直接累加时间后continue
                // 信道占用时长
                double wait_time = 10;
                int select_m, select_n;
                double select_dis = 0x7fffffff;
                double select_delay;
                bool _flg = 1;
                for (auto &[m, n] : candidate_uav)
                {
                    double dis = get_distance(uav, get_uav(time, m, n));
                    double delay = get_trans_delay(dis);
                    // 如果信道被占用，则计算等待时间并更新全局等待时间
                    if (is_occupy({now_m, now_n}, {m, n}, {time, time + delay}))
                    {
                        wait_time = min(wait_time, get_occupy_free({now_m, now_n}, {m, n}, time));
                        continue;
                    }
                    // 至少是个可行解,但不一定是最优解
                    _flg = 0;
                    double dis_ed = get_distance(this->BS[ed_bs_id], get_uav(time + delay, m, n));
                    if (dis_ed <= select_dis)
                    {
                        select_dis = dis_ed;
                        select_delay = delay;
                        select_m = m;
                        select_n = n;
                    }
                }
                // 如果没有可行解，则等待一段时间
                if (_flg || select_delay >= wait_time)
                {
                    time += wait_time;
                    this->delay += wait_time;
                    continue;
                }
                double dis = get_distance(get_uav(time, now_m, now_n), get_uav(time, select_m, select_n));
                double delay = get_trans_delay(dis);
                update_ROUTE({now_m, now_n}, {select_m, select_n}, {time, time + delay});
                this->delay += delay;
                time += delay;
                now_m = select_m;
                now_n = select_n;
                this->result.push_back(result_node(time, now_m, now_n));
            }
            else
            {
                now_m = dia_m;
                now_n = dia_n;
            }
#ifdef DEBUG
            printf("send_ed_route: (%d,%d) %.4lf\n", now_m, now_n, time);
#endif
            // 如果该点可以直接传信息给终点基站，则不要在循环了
            if (get_distance(this->BS[ed_bs_id], get_uav(time, now_m, now_n)) <= this->D)
            {
                double _delay = get_trans_delay(get_distance(this->BS[ed_bs_id], get_uav(time, now_m, now_n)));
                if (get_distance(this->BS[ed_bs_id], get_uav(time + _delay, now_m, now_n)) <= this->D)
                {
                    if (is_occupy({now_m, now_n}, {this->BS[ed_bs_id].get_coord()}, {time, time + _delay}))
                    {
                        double wait_time = get_occupy_free({now_m, now_n}, {this->BS[ed_bs_id].get_coord()}, time);
                        this->delay += wait_time;
                        time += wait_time;
                        continue;
                    }
                    update_ROUTE({now_m, now_n}, {this->BS[ed_bs_id].get_coord()}, {time, time + _delay});
                    this->delay += _delay;
                    time += _delay;
                    flg = true;
                    break;
                }
            }
        }
    }

    void calculate_part_2(double time, int m, int n, int st_bs_id, int ed_bs_id, int load)
    {
        // 确定终点所在行
        int ed_n = this->BS[ed_bs_id].y / this->d_inter;
        if (this->BS[ed_bs_id].y - this->d_inter * ed_n > this->D)
            ed_n++;
        // 发送分为两段，从(m,n)->(m+ed_n-n,ed_n),从(m+ed_n-n,ed_n)->ed_bs_id
        int now_m = m;
        int now_n = n;
        int ed_m = m;
        if(this->BS[st_bs_id].x < this->BS[ed_bs_id].x)
            ed_m += abs(ed_n - n);
        else
            ed_m -= abs(ed_n - n);
        while (now_n != ed_n)
        {
            uav_node uav = get_uav(time, now_m, now_n);
            uav_node ed_uav = get_uav(time, ed_m, ed_n);
            vector<pair<int, int>> candidate_uav; // 候选终点uav,{now_m,now_n}->{candidate_m,candidate_n}
            bool is_need_HAPS = false;            // 是否需要HAPS,要求在对角线请求中存在一个HAPS
            int dia_m = now_m, dia_n = now_n;     // 如果使用HAPS的话，终点坐标
            if (uav.x <= ed_uav.x && uav.y <= ed_uav.y)
            {
                candidate_uav.push_back({now_m + 1, now_n});
                candidate_uav.push_back({now_m, now_n + 1});
                if (fabs(ed_uav.x - uav.x) >= this->d_intra && fabs(ed_uav.y - uav.y) >= this->d_inter)
                {
                    is_need_HAPS = true;
                    dia_m++;
                    dia_n++;
                }
            }
            else if (uav.x <= ed_uav.x && uav.y >= ed_uav.y)
            {
                candidate_uav.push_back({now_m + 1, now_n});
                candidate_uav.push_back({now_m, now_n - 1});
                if (fabs(ed_uav.x - uav.x) >= this->d_intra && fabs(ed_uav.y - uav.y) >= this->d_inter)
                {
                    is_need_HAPS = true;
                    dia_m++;
                    dia_n--;
                }
            }
            else if (uav.x >= ed_uav.x && uav.y <= ed_uav.y)
            {
                candidate_uav.push_back({now_m - 1, now_n});
                candidate_uav.push_back({now_m, now_n + 1});
                if (fabs(ed_uav.x - uav.x) >= this->d_intra && fabs(ed_uav.y - uav.y) >= this->d_inter)
                {
                    is_need_HAPS = true;
                    dia_m--;
                    dia_n++;
                }
            }
            else if (uav.x >= ed_uav.x && uav.y >= ed_uav.y)
            {
                candidate_uav.push_back({now_m - 1, now_n});
                candidate_uav.push_back({now_m, now_n - 1});
                if (fabs(ed_uav.x - uav.x) >= this->d_intra && fabs(ed_uav.y - uav.y) >= this->d_inter)
                {
                    is_need_HAPS = true;
                    dia_m--;
                    dia_n--;
                }
            }
            // 判断从(now_m,now_n)->(sel_m,sel_n)之间是否存在HAPS
            // 如果存在，需要转发两次
            bool is_send_HAPS = false;
            if (is_need_HAPS)
            {
                for (auto &[id, hap] : this->HAPS)
                {
                    double dis_st = get_distance(get_uav(time, now_m, now_n), hap);
                    double delay_st = get_trans_delay(dis_st);
                    double dis_st_after = get_distance(get_uav(time + delay_st, now_m, now_n), hap);
                    if (dis_st > this->d || dis_st_after > this->d)
                        continue;
                    double dis_ed = get_distance(get_uav(time + delay_st, dia_m, dia_n), hap);
                    double delay_ed = get_trans_delay(dis_ed);
                    double dis_ed_after = get_distance(get_uav(time + delay_st + delay_ed, dia_m, dia_n), hap);
                    if (dis_ed > this->d || dis_ed_after > this->d)
                        continue;
                    // 距离比直接发送要远
                    if (dis_st + dis_ed >= this->d_inter + this->d_intra)
                        continue;
                    // 信道占用
                    if (is_occupy({now_m, now_n}, this->HAPS[id].get_coord(), {time, time + delay_st}) || is_occupy(this->HAPS[id].get_coord(), {dia_m, dia_n}, {time + delay_st, time + delay_st + delay_ed}))
                        continue;
                    // 可以
                    is_send_HAPS = true;
                    update_ROUTE({now_m, now_n}, this->HAPS[id].get_coord(), {time, time + delay_st});
                    this->delay += delay_st;
                    time += delay_st;
                    this->result.push_back(result_node(time, id));
                    update_ROUTE(this->HAPS[id].get_coord(), {dia_m, dia_n}, {time, time + delay_ed});
                    this->delay += delay_ed;
                    time += delay_ed;
                    this->result.push_back(result_node(time, dia_m, dia_n));
                    break;
                }
            }
            if (!is_send_HAPS)
            {
                // 从候选节点中，寻找与终点最近的那个
                // 如果信道被占用则，等待wait_time时间,直接累加时间后continue
                // 信道占用时长
                double wait_time = 10;
                int select_m, select_n;
                double select_dis = 0x7fffffff;
                double select_delay;
                bool _flg = 1;
                for (auto &[m, n] : candidate_uav)
                {
                    double dis = get_distance(uav, get_uav(time, m, n));
                    double delay = get_trans_delay(dis);
                    // 如果信道被占用，则计算等待时间并更新全局等待时间
                    if (is_occupy({now_m, now_n}, {m, n}, {time, time + delay}))
                    {
                        wait_time = min(wait_time, get_occupy_free({now_m, now_n}, {m, n}, time));
                        continue;
                    }
                    // 至少是个可行解,但不一定是最优解
                    _flg = 0;
                    double dis_ed = get_distance(ed_uav, get_uav(time + delay, m, n));
                    if (dis_ed <= select_dis)
                    {
                        select_dis = dis_ed;
                        select_delay = delay;
                        select_m = m;
                        select_n = n;
                    }
                }
                // 如果没有可行解，则等待一段时间
                if (_flg)
                {
                    time += wait_time;
                    this->delay += wait_time;
                    continue;
                }
                double dis = get_distance(get_uav(time, now_m, now_n), get_uav(time, select_m, select_n));
                double delay = get_trans_delay(dis);
                update_ROUTE({now_m, now_n}, {select_m, select_n}, {time, time + delay});
                this->delay += delay;
                time += delay;
                now_m = select_m;
                now_n = select_n;
                this->result.push_back(result_node(time, now_m, now_n));
            }
            else
            {
                now_m = dia_m;
                now_n = dia_n;
            }
        }
        calculate_part_1(time, now_m, now_n, st_bs_id, ed_bs_id, load);
    }

    void CAL(double time, int st_bs_id, int ed_bs_id)
    {
        this->ROUTE.clear();
        // 发 3, 3, 3, 1
        // 每次等待0.1075s再发
        calculate(time, st_bs_id, ed_bs_id, 3);
        calculate(time, st_bs_id, ed_bs_id, 3);
        calculate(time, st_bs_id, ed_bs_id, 3);
        calculate(time, st_bs_id, ed_bs_id, 1);
    }
};

int main()
{
    // 基站信息
    unordered_map<int, base_station> BS = {
        {0, base_station(0, 45.73, 45.26, 0)},
        {1, base_station(1, 1200, 700, 0)},
        {2, base_station(2, -940, 1100, 0)}};
    // 空中平台信息
    unordered_map<int, ha_platform> HAPS = {
        {0, ha_platform(0, -614, 1059, 24)},
        {1, ha_platform(1, -934, 715, 12)},
        {2, ha_platform(2, 1073, 291, 37)},
        {3, ha_platform(3, 715, 129, 35)},
        {4, ha_platform(4, 186, 432, 21)},
        {5, ha_platform(5, -923, 632, 37)},
        {6, ha_platform(6, 833, 187, 24)},
        {7, ha_platform(7, -63, 363, 11)}};
    // 解决方案
    Solution solution(5, 10, 90, 80, 115, 70, 0.1, 3, 10, BS, HAPS);
    solution.CAL(0, 0, 1);
    solution.CAL(0, 1, 0);
    solution.CAL(0, 0, 2);
    solution.CAL(0, 2, 0);
    solution.CAL(0, 1, 2);
    solution.CAL(0, 2, 1);

    solution.CAL(4.7, 0, 1);
    solution.CAL(4.7, 1, 0);
    solution.CAL(4.7, 0, 2);
    solution.CAL(4.7, 2, 0);
    solution.CAL(4.7, 1, 2);
    solution.CAL(4.7, 2, 1);

    solution.CAL(16.4, 0, 1);
    solution.CAL(16.4, 1, 0);
    solution.CAL(16.4, 0, 2);
    solution.CAL(16.4, 2, 0);
    solution.CAL(16.4, 1, 2);
    solution.CAL(16.4, 2, 1);

#ifdef INFO
    cout << "TOTAL: " << solution.TOTAL_DELAY << endl;
#endif

    return 0;
}