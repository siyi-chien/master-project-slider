/******************************************************************************
-First Author:  Siyi
-Last Modified by: Siyi
-SLIDER project, Robot Intelligence Lab, Imperial College London, 2023
******************************************************************************/
#include <vector>
#include <time.h>

#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>
#include <gazebo_msgs/LinkStates.h>
#include <qpOASES.hpp>
#include <cstdlib>
#include "matplotlibcpp.h"
#include "dynamics.h"
#include <fstream>
#include <iostream>
#include <slider_controller/Weight_Ref.h>
#include "cstdlib"
namespace plt = matplotlibcpp;
int pre_Nsxy = 0;
int pre_Nsz = 0;

int param_recieve = 0; // The flag for the rosparam input,  set 0 if we do not apply rosparam
int sample_time = 400; // The number of data we saved in csv files for debugging

double PI = 3.14;

double g = 9.8;
double Zc = 0.7;    // CoM height
double k = 1208.52; // The stiffness of the spring
double m = 15.0;    // The mass of the robot

double Ts_xy = 0.025; // sampling time in Horizontal MPC
double Ts_z = 0.025;  // sampling time in Vertical MPC

double w_z = 8.975971429;    // The parameter of Vertical dynamics, w_z=sqrt(k/m);
double w_xy = 3.74165738677; // The parameter of Horizen dynamics, w_xy=sqrt(g/z);
double mgk = m * g / k;

double forward_velocity = 0.4;                     // pre-set forward velocity
double lateral_velocity = 0.25;                    // pre-set lateral velocity
double forward_velocity_limit = 0.40;              // Max forward velocity
double lateral_velocity_limit = 0.25;              // Max lateral velocity
double step_time = 0.7;                            // The time cost in each footstep
double step_length = forward_velocity * step_time; // The length of each footstep
double inter_feet_clearance = 0.28;                // The width of two feet
int N_steps_xy = 4;
int N_steps_z = 1;
int Nsxy = step_time / Ts_xy;
int Nsz = step_time / Ts_z;

Vector2d x_info; // estimated current CoM position and velocity in the x direction
Vector2d y_info; // estimated current CoM position and velocity in the y direction
Vector2d z_info; // estimated current CoM position and velocity in the z direction

double remaining_time = step_time;      // remaining time for the current step
double p0_x = 0.0;                      // current foot position in x direction
double p0_y = inter_feet_clearance / 2; // current foot position in y direction
double support_foot = -1;               // right 1, left -1
const double LOOP_RATE = 1000;          // frequency of the communication

float x_q2 = 48.9035;
float y_q2 = 2.9026;
float x_q1 = 1e-4;
float y_q1 = 1e-4;

float x_w = 1243.6533;
float x_r = 300;
float y_w = 608.4622;
float y_r = 300;

// delete txt contant in myOpt

/***********************************************
 * @brief Obtaining the states from the Gazebo
 ************************************************/
void callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    x_info << msg->data[0], msg->data[1];
    y_info << msg->data[2], msg->data[3];
    z_info << msg->data[4], msg->data[5];

    remaining_time = msg->data[6];
    p0_x = msg->data[7];
    p0_y = msg->data[8];
    support_foot = msg->data[9];
    ofstream co;
    co.open("/home/siyi/SLIDER/src/slider_controller/myOpt.txt", ios::app);
    co << "****************************************************" << endl;
    for (int i = 0; i < msg->data.size(); i++)
    {
        co << p0_x << endl;
        co << p0_y << endl;
    }
    co.close();
}

bool setWeightRefCallback(slider_controller::Weight_Ref::Request &req, slider_controller::Weight_Ref::Response &res)
{
    forward_velocity = req.forward_velocity;
    lateral_velocity = req.lateral_velocity;
    x_q1 = req.x_q1;
    y_q1 = req.y_q1;
    x_q2 = req.x_q2;
    y_q2 = req.y_q2;
    x_w = req.x_w;
    x_r = req.x_r;
    y_w = req.y_w;
    y_r = req.y_r;
    res.success = true;
    return true;
}

int main(int argc, char **argv)
{

    unsigned seed;
    srand((unsigned)time(NULL));
    clock_t start_time, end_time;
    double running_time = 0; // ms
    std_msgs::Float64MultiArray opt;
    ofstream out;
    x_info << 0, 0;
    y_info << 0, 0;
    z_info << Zc, 0;
    // the setting of the ros nodes
    ros::init(argc, argv, " ros_SLIDER_planner_node");
    ros::NodeHandle mpc_planner_node;
    // output of the MPC motion planner
    ros::Publisher pub = mpc_planner_node.advertise<std_msgs::Float64MultiArray>("/slider_gazebo/zmp_foothold", 1);
    // input of the MPC
    ros::Subscriber sub = mpc_planner_node.subscribe<std_msgs::Float64MultiArray>("/slider_gazebo/planner_input", 1, callback);
    ros::Subscriber sub_1 = mpc_planner_node.subscribe<std_msgs::Float64MultiArray>("/slider_gazebo/weights", 1, callback);
    ros::ServiceServer srv_controller_parameter = mpc_planner_node.advertiseService("/slider_gazebo/setWeightRef", setWeightRefCallback);

    ros::Rate Loop_rate(LOOP_RATE);
    int current_steps = 0;
    int support_foot_pre = support_foot;

    horizen_dynamics modelx(g, Zc, Ts_xy, k, m, w_xy);
    horizen_dynamics modely(g, Zc, Ts_xy, k, m, w_xy);
    vertical_dynamics modelz(g, Zc, Ts_z, k, m, w_z);

    MPCController mpc_x(modelx.A, modelx.B, N_steps_xy, Ts_xy, step_time, remaining_time, 0, p0_x, inter_feet_clearance, x_q1, y_q1, x_q2, y_q2, x_w, x_r, y_w, y_r);
    MPCController mpc_y(modely.A, modely.B, N_steps_xy, Ts_xy, step_time, remaining_time, 1, p0_y, inter_feet_clearance, x_q1, y_q1, x_q2, y_q2, x_w, x_r, y_w, y_r);
    MPCController mpc_z(modelz.A, modelz.B, N_steps_z, Ts_z, step_time, remaining_time, 2, Zc, inter_feet_clearance, x_q1, y_q1, x_q2, y_q2, x_w, x_r, y_w, y_r);

    Zreference z_reference(g, w_z, Ts_z);

    float tbegin = clock();
    start_time = clock();
    double Uopt_z[mpc_z.N + mpc_z.N_steps + 1];
    double Uopt_x[mpc_x.N + mpc_x.N_steps];
    double Uopt_y[mpc_y.N + mpc_y.N_steps];
    double pre_remaing_time = 0;
    double tl = 0;
    while (ros::ok())
    {

        Nsxy = remaining_time / Ts_xy;
        Nsz = remaining_time / Ts_z;

        if (support_foot_pre != support_foot)
        {
            support_foot_pre = support_foot;
        }

        if (pre_Nsz != Nsz)
        {

            mpc_z.updateMatrix(remaining_time, 0, z_info, support_foot, z_info(0, 0) + mgk - 9.8 / pow(w_z, 2), x_q1, y_q1, x_q2, y_q2, x_w, x_r, y_w, y_r, 0);
            mpc_z.plan(Uopt_z);
            pre_Nsz = Nsz;
        }

        if (pre_Nsxy != Nsxy)
        {

            double deltat = pre_remaing_time > remaining_time ? pre_remaing_time - remaining_time : pre_remaing_time + Ts_xy - remaining_time;
            pre_remaing_time = remaining_time;

            mpc_x.updateMatrix(remaining_time, forward_velocity, x_info, support_foot, p0_x, x_q1, y_q1, x_q2, y_q2, x_w, x_r, y_w, y_r, deltat);
            mpc_y.updateMatrix(remaining_time, lateral_velocity, y_info, support_foot, p0_y, x_q1, y_q1, x_q2, y_q2, x_w, x_r, y_w, y_r, deltat);
            mpc_x.plan(Uopt_x);
            mpc_y.plan(Uopt_y);

            pre_Nsxy = Nsxy;
        }

        MatrixXd u_z = Map<MatrixXd>(Uopt_z, mpc_z.N, 1);
        MatrixXd z_all = mpc_z.Phi * z_info + mpc_z.Gamma * u_z;
        double z = z_all(0, 0);
        double zd = z_all(1, 0);
        double zdd = -pow(w_z, 2) * z_info(0) + pow(w_z, 2) * u_z(0, 0);

        int N = mpc_x.N;
        opt.data = {Uopt_x[0], Uopt_y[0], (p0_x + Uopt_x[N]), (p0_y + Uopt_y[N]), z, zd, zdd, z_info(0), z_info(0), (p0_x + Uopt_x[N] + Uopt_x[N + 1]), (p0_y + Uopt_y[N] + Uopt_y[N + 1]), (p0_x + Uopt_x[N] + Uopt_x[N + 1] + Uopt_x[N + 2]), (p0_y + Uopt_y[N] + Uopt_y[N + 1] + Uopt_y[N + 2]), (p0_x + Uopt_x[N] + Uopt_x[N + 1] + Uopt_x[N + 2] + Uopt_x[N + 3]), (p0_y + Uopt_y[N] + Uopt_y[N + 1] + Uopt_y[N + 2] + Uopt_y[N + 3])};

        pub.publish(opt);
        ros::spinOnce();
        Loop_rate.sleep();
    }
    return 0;
}
