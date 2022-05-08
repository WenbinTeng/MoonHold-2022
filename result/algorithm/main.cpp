#include <iostream>
#include <iomanip>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <cfloat>

using namespace std;

typedef struct coor_t {
    float x, y, z;
    coor_t() {}
    coor_t(float x, float y, float z) : x(x), y(y), z(z) {}
    coor_t operator+(const coor_t& right) {
        return {x + right.x, y + right.y, z + right.z};
    }
    void operator+=(const coor_t& right) {
        x += right.x;
        y += right.y;
        z += right.z;
    }
    coor_t dot(const coor_t& right) {
        return {x * right.x, y * right.y, z * right.z};
    }
    float distance(const coor_t& right) {
        return sqrt((x - right.x) * (x - right.x) + (y - right.y) * (y - right.y) + (z - right.z) * (z - right.z));
    }
} coor_t;

typedef struct {
    int id;
    coor_t coor;
} bs_t;

typedef struct {
    int m, n;
    coor_t coor;
} uav_t;

typedef struct {
    float v;           // 无人机速度
    float H;           // 无人机高度
    float d_intra;     // 无人机间距
    float d_inter;     // 轨道行距
    float d;           // 无人机通信距离
    float D;           // 基站通信距离
    float tf;          // 转发延时固定常数
} param_t;

typedef struct {
    float time;
    int m, n;
} route_t;

typedef struct {
    float time;
    int start_id;
    int end_id;
    float delay;
    int payload;
    vector<route_t> path;
} result_t;

void data_loader(param_t& param) {
    param.v = 5;
    param.H = 10;
    param.d_intra = 90;
    param.d_inter = 80;
    param.d = 125;
    param.D = 70;
    param.tf = 0.1;
}

void data_output(vector<result_t>& results) {
    for (const auto& result: results) {
        cout << fixed << setprecision(4) << result.time << ",";
        cout.unsetf(ostream::fixed);
        cout << result.start_id << ",";
        cout << result.end_id << ",";
        cout << fixed << setprecision(4) << result.delay << ",";
        cout.unsetf(ostream::fixed);
        cout << result.payload;
        cout << endl;
        for (int i = 0; i < result.path.size(); ++i) {
            if (i)
                cout << ",";
            cout << "(";
            cout << fixed << setprecision(4) << result.path[i].time << ",";
            cout.unsetf(ostream::fixed);
            cout << result.path[i].m << ",";
            cout << result.path[i].n;
            cout << ")";
        }
        cout << endl;
    }
}

uav_t get_gateway(float& time, bs_t& start, bs_t& end, param_t& param) {
    uav_t gateway;

    int min_idx = -1;
    float min_dis = FLT_MAX;

    int x = (int)(start.coor.x - time * param.v) / (int)param.d_intra * (int)param.d_intra;
    int y = (int)(start.coor.y) / (int)param.d_inter * (int)param.d_inter;
    coor_t near_coor(x, y, 0);

    vector<coor_t> df = {
        { 0,  0,  0},
        { 1,  0,  0},
        { 1,  1,  0},
        { 0,  1,  0},
        {-1,  1,  0},
        {-1,  0,  0},
        {-1, -1,  0},
        { 0, -1,  0},
        { 1, -1,  0}
    };

    for (int i = 0; i < 9; ++i) {
        coor_t uav_coor = near_coor + df[i].dot({param.d_intra, param.d_inter, 1}) + coor_t(time * param.v, 0, 0);
        float before_send_dis = start.coor.distance(uav_coor);
        float after_send_dis = start.coor.distance(uav_coor + coor_t((param.tf + before_send_dis / 10000.0) * param.v, 0.0, 0.0));
        float dis = uav_coor.distance(end.coor);
        
        if (before_send_dis <= param.D && after_send_dis <= param.D)
            if (min_dis > dis) {
                min_dis = dis;
                min_idx = i;
            }
    }

    if (min_idx == -1) {
        cout << "cannot find gateway." << endl;
        exit(1);
    }
    
    time += param.tf + start.coor.distance(gateway.coor) / 10000.0;
    gateway.m = (near_coor.x / param.d_intra) + df[min_idx].x;
    gateway.n = (near_coor.y / param.d_inter) + df[min_idx].y;
    gateway.coor = near_coor + df[min_idx].dot({param.d_intra, param.d_inter, 1}) + coor_t(time * param.v, 0, 0);
    
    return gateway;
}

bool get_next_route(float& time, uav_t& curr, bs_t& end, param_t& param) {
    float before_send_dis = end.coor.distance(curr.coor);
    float after_send_dis = end.coor.distance(curr.coor + coor_t((param.tf + before_send_dis / 10000.0) * param.v, 0.0, 0.0));
    if (before_send_dis <= param.D && after_send_dis <= param.D) {
        time += param.tf + before_send_dis / 10000.0;
        return false;
    }
    
    int min_idx = -1;
    float min_dis = FLT_MAX;

    vector<coor_t> df = {
        { 1,  0,  0},
        { 1,  1,  0},
        { 0,  1,  0},
        {-1,  1,  0},
        {-1,  0,  0},
        {-1, -1,  0},
        { 0, -1,  0},
        { 1, -1,  0}
    };

    for (int i = 0; i < 8; ++i) {
        float dis = end.coor.distance(curr.coor + df[i].dot({param.d_intra, param.d_inter, 1}));
        if (min_dis > dis) {
            min_dis = dis;
            min_idx = i;
        }
    }
    
    time += param.tf + df[min_idx].dot({param.d_intra, param.d_inter, 1}).distance({0,0,0}) / 10000.0;
    curr.m += df[min_idx].x;
    curr.n += df[min_idx].y;
    curr.coor = {time * param.v + curr.m * param.d_intra, curr.n * param.d_inter, 0};

    return true;
}

result_t calculate(float time, bs_t& start, bs_t& end, param_t& param) {
    result_t result;
    result.time = time;
    result.start_id = start.id;
    result.end_id = end.id;
    result.payload = 1;

    uav_t uav = get_gateway(time, start, end, param);
    result.path.push_back({time, uav.m, uav.n});

    while (get_next_route(time, uav, end, param)) {
        result.path.push_back({time, uav.m, uav.n});
    }

    result.delay = time - result.time;

    return result;
}

int main(int argc, char const *argv[])
{
    unordered_map<int, bs_t> bs = {
        {0, {0, {45.73, 45.26, 0}}},
        {1, {1, {1200, 700, 0}}},
        {2, {2, {-940, 1100, 0}}}
    };

    vector<result_t> results;
    param_t global_param;
    data_loader(global_param);
    results.push_back(calculate(0, bs[0], bs[1], global_param));
    results.push_back(calculate(0, bs[0], bs[2], global_param));
    results.push_back(calculate(4.7, bs[0], bs[1], global_param));
    results.push_back(calculate(4.7, bs[0], bs[2], global_param));
    results.push_back(calculate(16.4, bs[0], bs[1], global_param));
    results.push_back(calculate(16.4, bs[0], bs[2], global_param));
    data_output(results);

    return 0;
}
