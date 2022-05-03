#include <iostream>
#include <iomanip>
#include <vector>
#include <unordered_map>

using namespace std;

typedef struct {
    int id;
    float x, y, z;
} bs_t;

typedef struct {
    int m, n;
    float x, y, z;
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
        cout << fixed << setprecision(4) << result.delay;
        cout.unsetf(ostream::fixed);
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

void calculate() {

}

int main(int argc, char const *argv[])
{
    unordered_map<int, bs_t> bs_loc = {
        {0, {0, 45.73, 45.26, 0}},
        {1, {1, 1200, 700, 0}},
        {2, {2, -940, 1100, 0}}
    };
    param_t global_param;
    data_loader(global_param);
    return 0;
}
