#pragma GCC optimize("O3")
#pragma GCC target("sse,sse2,sse3,ssse3,sse4,popcnt,abm,mmx,avx,tune=native")
//
// Created by T118029 on 2021/03/15.
//

#include <GL/glut.h>
#include <GL/freeglut.h>
#include <iostream>
#include "simbase.h"
#include <random>
#include <omp.h>
#include <fstream>
#include <unistd.h>

#define LEFT   0
#define CENTER 1
#define RIGHT  2
#define TRANGE 1.5 //タッチセンサーのレンジ 半径の倍数
#define RANGE 0 //通信レンジ の半径
#define INHIBITOR_RANGE 100
#define RIGHT_TURN -0.1        //右回転 0.1ラジアンの定義
#define LEFT_TURN    0.1        //左回転 0.1ラジアンの定義
#define ROBOS  3 //ロボット台数　10台
#define N 5000
#define MAPDENSITY 80
#define V 0.6
#define RADIUS 100
#define step 10020
#define GLID 15
#define GLIDGLID 225
#define AUTOCORR 1000
#define SAMPLEROBOT 10000

double input_concentration[15][15] = {
        {0.5,     0.00001, 0.00001, 0.00001, 0.00001, 1,       1,       1,   0.5, 0.00001, 0.00001, 1,       1,       1,   1},
        {1,       1,       0.00001, 0.00001, 0.00001, 1,       1,       1,   1,   0.5,     0.00001, 0.5,     1,       1,   1},
        {1,       1,       0.5,     0.00001, 0.00001, 0.5,     1,       1,   1,   0.5,     0.00001, 0.5,     1,       1,   1},
        {1,       1,       0.5,     1,       1,       0.00001, 0.5,     1,   1,   0.5,     0.00001, 0.5,     1,       1,   1},
        {1,       1,       1,       0.5,     1,       1,       1,       1,   1,   0.5,     0.00001, 0.00001, 0.5,     1,   1},

        {0.5,     0.5,     1,       1,       1,       0.00001, 1,       1,   1,   1,       0.00001, 0.00001, 0.00001, 0.5, 1,},
        {0.00001, 0.5,     1,       1,       0.5,     0.00001, 0.5,     1,   0.5, 1,       0.00001, 0.00001, 0.00001, 0.5, 1},
        {0.00001, 1,       1,       1,       0.00001, 0.00001, 0.00001, 0.5, 1,   0.5,     0.00001, 0.00001, 0.00001, 0.5, 1},
        {0.5,     1,       1,       1,       0.00001, 0.00001, 0.00001, 0.5, 1,   1,       0.5,     0.00001, 0.5,     1,   1},
        {0.00001, 1,       1,       1,       0.00001, 0.00001, 0.00001, 0.5, 1,   1,       1,       1,       1,       1,   1},

        {0.00001, 0.00001, 1,       1,       0.5,     0.5,     0.5,     1,   1,   1,       1,       1,       1,       1,   1},
        {0.00001, 0.00001, 0.5,     1,       1,       1,       1,       1,   1,   1,       0.5,     0.5,     0.5,     1,   1},
        {0.00001, 1,       1,       1,       1,       1,       1,       1,   1,   0.5,     0.00001, 0.00001, 0.5,     1,   1},
        {0.5,     1,       1,       1,       1,       1,       1,       1,   0.5, 0.00001, 0.00001, 0.00001, 0.00001, 0.5, 1},
        {1,       1,       1,       1,       1,       1,       1,       1,   0.5, 0.00001, 0.00001, 0.00001, 0.00001, 0.5, 1},
};


using namespace std;

struct ROBOTLOG {
    double x;
    double y;
    double distance;
    int hogehoge;
};

struct GLID_STRUCT {
    double ave_activator;
    double ave_inhibitor;
};

typedef struct ROBO {
    int v=0;
    int step_counter = 0;

    double r{};
    double x{}, y{};
    double dir{};

    int glid_x = 0;
    int glid_y = 0;

    int map_glid_x = 0;
    int map_glid_y = 0;

    int tCenter = 0;
    int tRight = 0;
    int tLeft = 0;

    //信号出力フラグ群
    int act_flag{};
    int inh_flag{};

    //反応拡散用パラメータ群
    double activator = 0;
    double inhibitor = 0;
    double sum_activator = 0;
    double sum_inhibitor = 0;
    int act_touch_counter = 0;
    int inh_touch_counter = 0;

    double dx = 0;
    double dy = 0;

    double Dv = 0.40;
    double Du = 0.08;
    double Cv = 0.0000;
    double Cu = 0.00010;
    double a = 0.010;
    double b = 0.011;
    double c = 0.008;
    double d = 0.009;

    POSITION tsensor[3]{}; //構造体変数の追加
public:
    void draw();

    void action();

    void init();

    void turn(double q);

    void forward(double v);

    int touchsensor(int i);

    static int check_cross_wall(POSITION p1, POSITION p2);

    int check_cross_others(POSITION p);

    int stack = 0;

    void nearrobotsensor();

} ROBO;

ROBO robo[ROBOS];//要素数ROBOSで配列変数roboを定義

int epoch = 0;
// 通信可能性のあるロボットを効率的に探すためのグリッド切り
int gridline = (int) point * 2 / INHIBITOR_RANGE;
int half_gridline = gridline / 2;
// ビジュアライズしたマップのためのグリッド切り
int map_gridline = (int) point * 2 / MAPDENSITY;
int half_map_gridline = map_gridline / 2;

int windows[2];

double save_grid_activater[100000][1000] = {};
double save_autocorr[N][1000] = {};
double save_autocorr_fin[N] = {};
int save_sum_distance[step][30] = {};

std::random_device rnd;     // 非決定的な乱数生成器
std::mt19937 mt(rnd());
GLID_STRUCT GL[100][100];
ROBOTLOG robotlog[ROBOS][step] = {};


double input_ave = 0;

void calculate_grid_concentration();

void draw_grid_density_map();

void grid_display();

void wall_draw();

void Initialize();

void input_turingpattern();

void save_grid_concentration();

double calculate_input_ave();

void calculate_autocorr();

void save_robot_loging();

void calculate_sum_distance();

void input_grid_point();

void wall_draw() {
    glBegin(GL_LINES);
    for (auto &i: wall) {
        glVertex2d(pin[i.p1].x, pin[i.p1].y);
        glVertex2d(pin[i.p2].x, pin[i.p2].y);
    }
    glEnd();
}

void grid_wall_draw() {

    glBegin(GL_LINES);
    glColor3d(1, 1, 1);

    for (auto &i: wall) {
        glVertex2d(pin[i.p1].x, pin[i.p1].y);
        glVertex2d(pin[i.p2].x, pin[i.p2].y);
    }
    glEnd();

    glBegin(GL_LINES);
    // tate
    for (int j = 0; j < map_gridline; ++j) {
        glVertex2d(-point + (j * MAPDENSITY), point);
        glVertex2d(-point + (j * MAPDENSITY), -point);
    }

    // yoko
    for (int j = 0; j < map_gridline; ++j) {
        glVertex2d(point, point - (j * MAPDENSITY));
        glVertex2d(-point, point - (j * MAPDENSITY));
    }
    glEnd();
}

void display() {

    glClear(GL_COLOR_BUFFER_BIT);
    graphics();
    wall_draw();
    for (auto &i: robo) {
        i.draw();
    }
    glColor3d(1, 1, 1);
    glRasterPos3d(-point + 10, -point + 10, 0.0);
    string epoch_num = to_string(epoch);
    for (char &c: epoch_num) {
        glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, c);
    }
    glutSwapBuffers();
}

void grid_display() {
    glClear(GL_COLOR_BUFFER_BIT);
    graphics();
    draw_grid_density_map();
    grid_wall_draw();
    glColor3d(1, 1, 1);


    glRasterPos3d(-point + 10, -point + 10, 0.0);
    string epoch_num = to_string(epoch);
    for (char &c: epoch_num) {
        glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, c);
    }
    glutSwapBuffers();
}

void calculate_grid_concentration() {
    // 初期化
    double sum_glid_activator;
    double sum_glid_inhibitor;

    for (int x = 0; x < map_gridline; ++x) {
        for (int y = 0; y < map_gridline; ++y) {// y point
            GL[x][y].ave_activator = 0;
            GL[x][y].ave_inhibitor = 0;
        }
    }
    // 初期化 終わり
    for (int x = 0; x < map_gridline; ++x) {
        for (int y = 0; y < map_gridline; ++y) {// y point
            int counter = 0;
            //配列を初期化すべし
            sum_glid_activator = 0;
            sum_glid_inhibitor = 0;
            for (int i = 0; i < ROBOS; i++) {// i point
                if (x == robo[i].map_glid_x && y == robo[i].map_glid_y) {
                    counter++;
                    sum_glid_activator += robo[i].activator;
                    sum_glid_inhibitor += robo[i].inhibitor;
                }
            }
            GL[x][y].ave_activator = sum_glid_activator / counter;
            GL[x][y].ave_inhibitor = sum_glid_inhibitor / counter;
        }
    }
//    cout << "epoch," << epoch << endl;
}

void draw_grid_density_map() {
    for (int i = 0; i < map_gridline; ++i) {
        for (int j = 0; j < map_gridline; ++j) {
            glBegin(GL_TRIANGLE_STRIP);
            glColor3d(GL[i][j].ave_activator, GL[i][j].ave_inhibitor,
                      1 - 0.5 * (GL[i][j].ave_activator + GL[i][j].ave_inhibitor));
            glColor3d(GL[i][j].ave_activator, 0, 0);
            glVertex2d(-point + (MAPDENSITY * i), -point + (MAPDENSITY * j));
            glVertex2d(-point + (MAPDENSITY * (i + 1)), -point + (MAPDENSITY * j));
            glVertex2d(-point + (MAPDENSITY * i), -point + (MAPDENSITY * (j + 1) * 1));
            glVertex2d(-point + (MAPDENSITY * (i + 1)), -point + (MAPDENSITY * (j + 1) * 1));
            glEnd();
        }
    }
}

void ROBO::forward(double v) {
    x = x + cos(dir) * v;
    y = y + sin(dir) * v;
}

void ROBO::turn(double q) {
    dir += q;
}

void ROBO::action() {
    step_counter++;

    glid_x = (int) half_gridline + floor(x) / (2 * INHIBITOR_RANGE);//グリッドぎり
    glid_y = (int) half_gridline + floor(y) / (2 * INHIBITOR_RANGE);

    map_glid_x = half_map_gridline + floor(x) / (MAPDENSITY);
    map_glid_y = half_map_gridline + floor(y) / (MAPDENSITY);

    dx = activator * a - inhibitor * b + Cu;
    dy = activator * c - inhibitor * d + Cv;

    activator = activator + dx;
    inhibitor = inhibitor + dy;
    if (activator > 1) {
        activator = 1;
    }
    if (activator < 0) {
        activator = 0;
    }
    nearrobotsensor();

    tCenter = touchsensor(CENTER);    //中央センサーの値
    tRight = touchsensor(RIGHT);        //右センサーの値
    tLeft = touchsensor(LEFT);        //左センサーの値

    if (stack < 25) {
        if (tLeft == 1)        //左チセンサ反応あり
        {
            turn(RIGHT_TURN);
            stack++;
        } else if (tRight == 1)    //右センサ反応あり
        {
            turn(LEFT_TURN);
            stack++;
        } else if (tCenter == 1) //正面センサ反応あり
        {
            turn(RIGHT_TURN);
            stack++;
        } else    //いずれの条件も当てはまらないのは全てのタッチセンサが０のとき
        {
            forward(V);//前進1.0ステップ
            v=1;
            stack = 0;
        }
    } else {
        turn(1 * PI);
        stack = 0;
    }
}

void ROBO::init() {

    std::uniform_int_distribution<int> distr(-point + 10, point - 10);    // 非決定的な乱数生成器
    std::uniform_real_distribution<> dir_gen(0, 360);
    std::uniform_real_distribution<> rando(0.0, 1.0);

    sum_activator = 0;
    sum_inhibitor = 0;
    activator = rando(mt);
    inhibitor = rando(mt);

    dir = dir_gen(mt);
    x = distr(mt);
    y = distr(mt);
    r = RADIUS;

    tsensor[CENTER].x = TRANGE * r; //タッチセンサーのレンジ（棒の長さ）
    tsensor[CENTER].y = 0;
    tsensor[LEFT].x = TRANGE * r * cos(60.0 / 180.0 * PI); //正面が０度なので、左は６０度
    tsensor[LEFT].y = TRANGE * r * sin(60.0 / 180.0 * PI);
    tsensor[RIGHT].x = TRANGE * r * cos(-60.0 / 180.0 * PI); //正面が０度なので、右は－６０度
    tsensor[RIGHT].y = TRANGE * r * sin(-60.0 / 180.0 * PI);
}

void ROBO::draw() {

    glPushMatrix(); //現在の座標系の保存
    glTranslated(x, y, 0);              //ロボットの現在座標へ座標系をずらす
    glRotated(dir / PI * 180, 0, 0, 1); //進行方向へZ軸回転

    glColor3d(1, 0, 0); //ロボットに対して活性化因子に応じた色をつける。
    draw_robo_circle(0, 0, r);
    glColor3d(0.5, 0.5, 0.5);
    draw_circle(0, 0, r); //本体外形円の描画　現在の座標系の原点に対して描くことに注意


    /*
     * 以下のゾーンは通信範囲を目視したい場合に利用する。
     * 通常シミュレーション中はごちゃごちゃするのでオフにすること推奨。
     * */

//    glColor3d(0, 0, 0.5);
//    draw_circle(0, 0, RANGE); //通信範囲の描画

//    glColor3d(0, 0.5, 0);
//    draw_circle(0, 0, INHIBITOR_RANGE); //通信範囲の描画

//    glColor3d(0.3, 0.3, 0.3);
//    glBegin(GL_LINES);

    /*
     * おわり
     * */

    glVertex2d(0, 0); //左センサーの描画
    glVertex2d(tsensor[LEFT].x, tsensor[LEFT].y);
    glVertex2d(0, 0); //正面センサーの描画
    glVertex2d(tsensor[CENTER].x, tsensor[CENTER].y);
    glVertex2d(0, 0); //右センサーの描画
    glVertex2d(tsensor[RIGHT].x, tsensor[RIGHT].y);

    glEnd();
    glPopMatrix(); //保存ておいた座標系へ戻す
}

//接触センサー関数 戻り値に　接触状態を１　非接触状態を０　返す
int ROBO::touchsensor(int i) {
    int fw = 0;
    int fo = 0; //他のロボットとの接触フラグ
    POSITION p1, p2;
    // p1はロボットの中心座標
    p1.x = x; //ロボットの中心座標　この関数は構造体の中の関数なので、ｘは構造体ROBOの中のｘを意味する。
    p1.y = y; //同様にｙ

    // p2はセンサーの座標
    //まずロボット本体からの相対座標系で計算
    p2 = tsensor[i];      // i番のセンサーの座標を代入(p2は相対座標)
    //空間に対する進行方向を計算
    p2 = rotate(p2, dir); //ロボットの進行方向に回転（p2は絶対方向、相対位置）
    //空間に対する位置を計算
    p2.x += x;            //ロボットの中心座標を足す（p2は絶対座標）
    p2.y += y;

    //ロボット中心からセンサーまでの線分p1,p2と壁の交差チェック
    fw = check_cross_wall(p1, p2);

    //ここに他のロボットの検出判定処理を入れる
    fo = check_cross_others(p2); //センサーの先端位置

    if (fw == 1 || fo == 1) //どちらかのフラグが１だったらセンサー反応あり
    {
        return 1;
    }
    return 0; //センサー反応なし
}

int ROBO::check_cross_wall(POSITION p1, POSITION p2) {

    for (auto &i: wall) {
        double Wp1x, Wp1y, Wp2x, Wp2y; //線分の両端点
        double Bvx, Bvy;               //線分への垂線の方向ベクトル
        double Bx, By;              //球体の位置, 半径
        double det, a, b, c, d, e, f;  //ベクトルの式の変数

        //接触判定
        Bx = p1.x;
        By = p1.y;

        Wp1x = pin[i.p1].x;
        Wp1y = pin[i.p1].y;
        Wp2x = pin[i.p2].x;
        Wp2y = pin[i.p2].y;

        Bvx = (p2.x - p1.x);
        Bvy = (p2.y - p1.y);

        a = Wp2x - Wp1x;
        b = -Bvx;
        c = Wp2y - Wp1y;
        d = -Bvy;
        e = Bx - Wp1x;
        f = By - Wp1y;
        det = a * d - b * c;

        if (det != 0) {
            double t, s; //接触条件処理の条件変数
            s = 1.0 / det * (d * e - b * f);
            t = 1.0 / det * (-c * e + a * f);
            if (0 < t && t < 1 && 0 < s && s < 1) { //交差あり
                return 1;
            }
        }
    }
    return 0; //交差なし
}

int ROBO::check_cross_others(POSITION p) {
    /*
     * 自機と他のロボットの接触チェック
     */

    double l;
    double sensor_x;
    double sensor_y;

    sensor_x = p.x;
    sensor_y = p.y;

    for (auto &i: robo) {

        // 全ロボットに対して自機と近接したグリッドにいるか検索。
        if (
                (glid_x == i.glid_x || glid_x == i.glid_x + 1 || glid_x == i.glid_x - 1) &&
                (glid_y == i.glid_y || glid_y == i.glid_y + 1 || glid_y == i.glid_y - 1)
                ) {
            double another_robo_x = i.x;
            double another_robo_y = i.y;
            double distance_x;
            double distance_y;

            distance_x = another_robo_x - sensor_x;
            distance_y = another_robo_y - sensor_y;

            l = sqrt(distance_x * distance_x + distance_y * distance_y);
            if (l < i.r) {
                return 1;
            }
        }
    }
    return 0;
}

void ROBO::nearrobotsensor() {
    /*
     * 通信範囲に誰がいるのかをチェックし、確認出来たら値の更新を行う。
     */
    double l;
    double distance_x;
    double distance_y;

    // 全ロボットに対して自機と近接したグリッドにいるか検索。
    for (auto &i: robo) {
        if ((glid_x == i.glid_x || glid_x == i.glid_x + 1 || glid_x == i.glid_x - 1) &&
            (glid_y == i.glid_y || glid_y == i.glid_y + 1 || glid_y == i.glid_y - 1)) {

            double another_robo_x = i.x;
            double another_robo_y = i.y;

            distance_x = another_robo_x - x;
            distance_y = another_robo_y - y;

            l = sqrt(distance_x * distance_x + distance_y * distance_y);

            //近接グリッドにいたので、実際に通信範囲内にいるのか探索。
            //二つの通信範囲に応じて場合分け
            if (l < RANGE) {
                act_touch_counter++;
                i.act_touch_counter++;

                double tx;

                tx = Du * (activator - i.activator);

                sum_activator += activator - tx;
                i.sum_activator += i.activator + tx;

                act_flag = 1;

            }
            if (l < INHIBITOR_RANGE) {
                inh_touch_counter++;
                i.inh_touch_counter++;

                double ty;

                ty = Dv * (inhibitor - i.inhibitor);

                sum_inhibitor += inhibitor - ty;
                i.sum_inhibitor += i.inhibitor + ty;
                inh_flag = 1;

            }

            if (step_counter >= 10) {
                if (act_flag == 1) {
                    activator = sum_activator / act_touch_counter;
                    act_flag = 0;
                }
                if (inh_flag == 1) {
                    inhibitor = sum_inhibitor / inh_touch_counter;
                    inh_flag = 0;
                }
                sum_activator = 0;
                sum_inhibitor = 0;
                act_touch_counter = 0;
                inh_touch_counter = 0;
                step_counter = 0;
            }
        }
    }
}

void idle() {
    //ループするシミュレーション全体の動きを定義
    if (fStart == 0) return;

    for (auto &i: robo) i.action();


    calculate_grid_concentration();
    save_grid_concentration();
    save_robot_loging();

    for (int i; i < 2; i++) {
        glutSetWindow(windows[i]);
        glutPostRedisplay();
    }

    epoch++;

    if (epoch > step) {
        glutLeaveMainLoop();
    }
    usleep(100000);
}

void save_robot_loging() {
    /*
     * 全ロボットの座標をstepごとに全て記憶する。
     */
    for (int i = 0; i < ROBOS; ++i) {
        robotlog[i][epoch].x = robo[i].x;
        robotlog[i][epoch].y = robo[i].y;
        robotlog[i][epoch].hogehoge=robo[i].v;
        if (epoch == 0) {
            robotlog[i][epoch].distance = 0.0;
        } else {
            robotlog[i][epoch].distance =
                    sqrt(
                            (robo[i].x - robotlog[i][0].x) * (robo[i].x - robotlog[i][0].x) +
                            (robo[i].y - robotlog[i][0].y) * (robo[i].y - robotlog[i][0].y)
                    );

        }
    }
}

void save_grid_concentration() {
    /*
     * 全グリッドの濃度をstepごとに全て記憶する。
     * のちの自己相関であったり、OpenCVの利用へつなげる。
     */
    for (int x = 0; x < map_gridline; ++x) {
        for (int y = 0; y < map_gridline; ++y) {
            if (GL[x][y].ave_activator != GL[x][y].ave_activator) {
                save_grid_activater[epoch][x * (map_gridline) + y] = input_concentration[y][x];
//                cout << "なんなん？" << endl;
            } else {
                save_grid_activater[epoch][x * (map_gridline) + y] = GL[x][y].ave_activator;
            }
        }
    }
}

void mouse(int button, int state, int x, int y) //マウスボタンの処理
{
    if (state == GLUT_DOWN) { //ボタンが押されたら...
        if (fStart == 1) {
            fStart = 0;
        } else {
            fStart = 1;
        }
    }
}

void input_grid_point() {
    /*
     * ロボットの初期配置を行う関数。この38のマジックナンバーの2乗がグリッド配置できる数になる。
     * 15なども配置調整のためのマジックナンバーなのでここの数値を変えたら調整すること。
     */
    for (int i = 1; i <= 38; ++i) {
        for (int j = 0; j < 38; ++j) {
            robo[(i - 1) * 38 + j].x = -point + 15 + 31.5 * (i - 1);
            robo[(i - 1) * 38 + j].y = -point + 15 + 31.5 * j;
        }
    }

}

int main(int argc, char *argv[]) {

    Initialize();
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
    glutInitWindowSize(point, point);
    windows[0] = glutCreateWindow("Robot sim");
    glutDisplayFunc(display);
    glutReshapeFunc(resize);
    glutMouseFunc(mouse); //マウスのボタンを検出 これを切るとコマ送りになる。
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
    glutInitWindowSize(point, point);
    windows[1] = glutCreateWindow("Grid");
    glutDisplayFunc(grid_display);
    glutReshapeFunc(resize);
    glutIdleFunc(idle);
    glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);

    glutMainLoop();

    calculate_autocorr();
    cout << "finish" << endl;

    return 0;
}


void Initialize() {
    cout << "map_gridline:" << map_gridline << endl;
    epoch = 0;
    make_circle();//円図形データの作成
    for (auto &i: robo) i.init();
//    input_grid_point();

}

void calculate_autocorr() {
    /*
     *　記憶した濃度を利用して自己相関を求める関数。
     *  考え方とすれば一次元である波長の自己相関をn*nまで拡張したものと思ってほしい。
     *  1ピクセル（グリッドのセルのこと）1波長入っている。
     */
    double tmp = map_gridline * map_gridline;
    double ave[N] = {};

    //各列の平均を取得。
    for (int i = 0; i < tmp; ++i) {
        for (int j = 0; j < N * 2; ++j) {
            ave[i] += save_grid_activater[j][i] / (N * 2);
        }
    }

    //自己共分散
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            for (int k = 0; k < tmp; ++k) {
                save_autocorr[i][k] +=
                        ((save_grid_activater[j][k]) *
                         (save_grid_activater[j + i][k]));
            }
        }
    }

    //平均分引いて値を調整
    for (int i = 1; i < N; ++i) {
        for (int j = 0; j < tmp; ++j) {
            save_autocorr[i][j] = save_autocorr[i][j] / (save_autocorr[0][j]);
            save_autocorr_fin[i] += save_autocorr[i][j] / tmp;
        }
    }

/***************************
 * 出力部 ファイルはcmake-build-debug下に作られる。
 ***************************/
//    calculate_sum_distance();
    cout << "f1-finish" << endl;

    //自己相関の結果
//    ofstream output_autocorr_log("autocorr.csv");
//
//    for (int i = 0; i < N; ++i) {
//        output_autocorr_log << save_autocorr_fin[i];
//        for (int j = 0; j < GLIDGLID; ++j) {
//            output_autocorr_log << "," << save_autocorr[i][j];
//        }
//        output_autocorr_log << endl;
//    }
//    output_autocorr_log.close();
    cout << "f2-finish" << endl;

    //各ロボットの初期位置からの移動距離計算。
    //何かを間違えていて1step目が計算されてない。けど本来はゼロです。
//    ofstream robot_distance_log("robot_distance_log.csv");
//    for (int i = 0; i < N * 2; ++i) {
//        for (int j = 0; j < ROBOS; ++j) { robot_distance_log << robotlog[j][i].distance << ","; }
//        robot_distance_log << endl;
//    }
//    robot_distance_log.close();

    cout << "f3-finish" << endl;

    //ロボットの移動距離を分類わけしてヒストグラム（作成のための集計）を作る。
    //ヒストグラム作りたい場合はexcelとかでやってね。
//    ofstream robot_histgramdata("robot_histgramdata.csv");
//    for (int i = 0; i < step; ++i) {
//        for (int j = 0; j < 30; ++j) {
//            robot_histgramdata << save_sum_distance[i][j] << ",";
//        }
//        robot_histgramdata << endl;
//    }
//    robot_histgramdata.close();
    cout << "f4-finish" << endl;

//    //各ロボットの生の座標ログ。
//    ofstream robot_point_log("robot_point_log.csv");
//    for (int i = 0; i < N * 2; ++i) {
//        for (int j = 0; j < ROBOS; ++j) {
//            robot_point_log << ",robot_point num[" << i << "]," << robotlog[j][i].x << "," << robotlog[j][i].y;
//        }
//        robot_point_log << endl;
//    }
//    robot_point_log.close();
    cout << "f4-finish" << endl;

    ofstream robot_v_log("robot_v_log.csv");
    for (int i = 0; i < N * 2; ++i) {
        for (int j = 0; j < ROBOS; ++j) {
            robot_v_log  << robotlog[j][i].hogehoge << "," ;
        }
        robot_v_log << endl;
    }
    robot_v_log.close();


    cout << "calcfinish" << endl;
}

void calculate_sum_distance() {

    for (int i = 0; i < step; ++i) {
        for (int j = 0; j < ROBOS; ++j) {
            for (int k = 0; k < 30; ++k) {
                if ((k * 50) <= robotlog[j][i].distance && robotlog[j][i].distance < ((k + 1) * 50)) {
                    save_sum_distance[i][k]++;
                }
            }
        }
    }
}
