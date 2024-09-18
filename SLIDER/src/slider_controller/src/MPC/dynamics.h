/******************************************************************************
-First Author:  Siyi
-Last Modified by: Siyi
-SLIDER project, Robot Intelligence Lab, Imperial College London, 2023
******************************************************************************/
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <eigen3/unsupported/Eigen/KroneckerProduct>
#include <cmath>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <qpOASES.hpp>
#include <fstream>
#include <iostream>

using namespace qpOASES;
using namespace std;
using namespace Eigen;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> RowMajMat;
class vertical_dynamics
{
public:
    double g;
    double Zc;    // height of the COM
    double Ts;    // sample time
    double k;     // The stiffness of the spring
    double m;     // The mass of the robot
    double omega; // The parameter of Vertical dynamics, w_z=sqrt(k/m);
    Matrix2d A;   // system matrix
    Vector2d B;   // control matrix

    vertical_dynamics(double g, double Zc, double Ts, double k, double m, double omega);
};

class horizen_dynamics
{
public:
    double g;
    double Zc;    // height of the COM
    double Ts;    // sample time
    double k;     // The stiffness of the spring
    double m;     // The mass of the robot
    double omega; // The parameter of Vertical dynamics, w_z=sqrt(k/m);
    Matrix2d A;   // system matrix
    Vector2d B;   // control matrix

    horizen_dynamics(double g, double Zc, double Ts, double k, double m, double omega);
};

class MPCController
{
public:
    int N; // horizen length
    double Ts;
    int N_steps;
    double step_time;
    double reference_velocity;
    Matrix2d A; // system matrix
    Vector2d B; // control matrix
    MatrixXd Q; // weighting matrix on the CoM state
    MatrixXd R; // weighting matix on the ZMP tracking
    MatrixXd W; // weighting matrix distance between different steps
    MatrixXd Gamma;
    MatrixXd Phi;
    MatrixXd E;
    MatrixXd F;
    MatrixXd F1;
    Vector2d x0;
    VectorXd d_ref;
    VectorXd x_ref;
    MatrixXd T;
    MatrixXd L_bar;
    MatrixXd R_bar;
    MatrixXd Huu;
    MatrixXd GammatQ;
    MatrixXd Gamma_v;
    MatrixXd Phi_v;
    int support_foot;
    double p0;
    int direction; // 0 x 1 y 2 z
    double remaining_time;
    double footwidth;
    float x_q2;
    float y_q2;
    float x_w;
    float x_r;
    float y_w;
    float y_r;
    float ref_position;
    SQProblem qp;
    bool isfirst;
    MPCController(Matrix2d A, Vector2d B, int N_steps, double Ts, double step_time, double remaining_time, int direction, double pre_input, double footwidth, float x_q1, float y_q1, float x_q2, float y_q2, float x_w, float x_r, float y_w, float y_r); // flag 0 x, 1 y 2 z ,
    void updateMatrix(double remaining_time, double reference_velocity, Vector2d x0, int support_foort, double p0, float x_q1, float y_q1, float x_q2, float y_q2, float x_w, float x_r, float y_w, float y_r, double delta_x);
    void plan(double *res);
};

class Zreference
{
public:
    double s1;
    double s2;
    double r0;
    double rT;
    double g;
    double w_z;
    double Ts;
    double zt;
    double zdt;
    double zd0;
    double ut;
    Zreference(double g, double w_z, double Ts);
    void newStepUpdate(double z0, double zd0, double r0, double rT);
    void caculate_zt(double remaining_time);
};
