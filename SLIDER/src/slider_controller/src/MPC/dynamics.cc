/******************************************************************************
-First Author:  Siyi
-Last Modified by: Siyi
-SLIDER project, Robot Intelligence Lab, Imperial College London, 2023
******************************************************************************/
#include "dynamics.h"

vertical_dynamics::vertical_dynamics(double g, double Zc, double Ts, double k, double m, double omega)
{
    this->g = g;
    this->Zc = Zc;
    this->Ts = Ts;
    this->k = k;
    this->m = m;
    this->omega = omega;
    double cs = cos(this->omega * this->Ts);
    double sn = sin(this->omega * this->Ts);
    this->A << cs, sn / omega, -omega * sn, cs;
    this->B << 1 - cs, omega * sn;
}

horizen_dynamics::horizen_dynamics(double g, double Zc, double Ts, double k, double m, double omega)
{
    this->g = g;
    this->Zc = Zc;
    this->Ts = Ts;
    this->k = k;
    this->m = m;
    this->omega = omega;
    double cs = cosh(this->omega * this->Ts);
    double sn = sinh(this->omega * this->Ts);
    this->A << cs, sn / omega, omega * sn, cs;
    this->B << 1 - cs, -omega * sn;
}

MPCController::MPCController(Matrix2d A, Vector2d B, int N_steps, double Ts, double step_time, double remaining_time, int direction, double p0, double footwidth, float x_q1, float y_q1, float x_q2, float y_q2, float x_w, float x_r, float y_w, float y_r)
{
    this->A = A;
    this->B = B;
    this->N_steps = N_steps;
    this->Ts = Ts;
    this->step_time = step_time;
    // this->N =   int(remaining_time/Ts)+N_steps*int(step_time/Ts);
    this->N = N_steps * int(step_time / Ts);
    this->direction = direction;
    this->remaining_time = remaining_time;
    this->p0 = p0;
    this->footwidth = footwidth;
    this->x_q2 = x_q2;
    this->y_q2 = y_q2;
    this->x_w = x_w;
    this->x_r = x_r;
    this->y_w = y_w;
    this->y_r = y_r;
    this->ref_position = 0;
    this->isfirst = true;
    if (direction == 2)
        this->qp = SQProblem(N + N_steps + 1, N);
    else
        this->qp = SQProblem(N + N_steps, N);

    if (direction == 0) // x direction
    {

        Matrix2d q;
        q << x_q1, 0, 0, x_q2;
        this->Q = kroneckerProduct(MatrixXd::Identity(N, N), q);

        MatrixXd w = x_w * MatrixXd::Identity(1, 1);
        this->W = kroneckerProduct(MatrixXd::Identity(N_steps, N_steps), w);

        MatrixXd r = x_r * MatrixXd::Identity(1, 1);
        this->R = kroneckerProduct(MatrixXd::Identity(N, N), r);
    }
    else if (direction == 1) // y direction
    {
        Matrix2d q;
        q << y_q1, 0, 0, y_q2;
        this->Q = kroneckerProduct(MatrixXd::Identity(N, N), q);

        MatrixXd w = y_w * MatrixXd::Identity(1, 1);
        this->W = kroneckerProduct(MatrixXd::Identity(N_steps, N_steps), w);

        MatrixXd r = y_r * MatrixXd::Identity(1, 1);
        this->R = kroneckerProduct(MatrixXd::Identity(N, N), r);
    }
    else // z direction
    {
        Matrix2d q;
        q << 1e-4, 0, 0, 6;
        this->Q = kroneckerProduct(MatrixXd::Identity(N, N), q);

        MatrixXd w = 5 * MatrixXd::Identity(1, 1);
        this->W = kroneckerProduct(MatrixXd::Identity(N_steps + 1, N_steps + 1), w);

        MatrixXd r = 75 * MatrixXd::Identity(1, 1);
        this->R = kroneckerProduct(MatrixXd::Identity(N, N), r);
    }

    Phi = MatrixXd::Zero(2 * N, 2);
    Gamma = MatrixXd::Zero(2 * N, N);

    MatrixXd tmpA = A;
    MatrixXd tmpB = B;
    for (int i = 0; i < N; i++)
    {
        Phi.block(i * 2, 0, 2, 2) = tmpA;
        Gamma.block(i * 2, 0, 2, i + 1) = tmpB;
        tmpA = tmpA * A;
        tmpB.conservativeResize(tmpB.rows(), tmpB.cols() + 1);
        tmpB.block(0, 0, tmpB.rows(), tmpB.cols() - 1) = A * tmpB.block(0, 0, tmpB.rows(), tmpB.cols() - 1);
        tmpB.block(0, tmpB.cols() - 1, tmpB.rows(), 1) = B;
    }
    this->Huu = Gamma.transpose() * Q * Gamma + R;
    this->GammatQ = Gamma.transpose() * Q;
    MatrixXd Tv = MatrixXd::Zero(N, 2 * N);
    for (int i = 0; i < Tv.rows(); i++)
        Tv(i, 2 * i + 1) = 1;
    this->Gamma_v = Tv * Gamma;
    this->Phi_v = Tv * Phi;
}

void MPCController::updateMatrix(double remaining_time, double reference_velocity, Vector2d x0, int support_foot, double p0, float x_q1, float y_q1, float x_q2, float y_q2, float x_w, float x_r, float y_w, float y_r, double deltat)
{

    if (x_q2 != this->x_q2 || y_q2 != this->y_q2 || x_w != this->x_w || x_r != this->x_r || y_w != this->y_w || y_r != this->y_r)
    {
        this->x_q2 = x_q2;
        this->y_q2 = y_q2;
        this->x_w = x_w;
        this->x_r = x_r;
        this->y_w = y_w;
        this->y_r = y_r;

        if (direction == 0) // x direction
        {

            Matrix2d q;
            q << x_q1, 0, 0, x_q2;
            this->Q = kroneckerProduct(MatrixXd::Identity(N, N), q);

            MatrixXd w = x_w * MatrixXd::Identity(1, 1);
            this->W = kroneckerProduct(MatrixXd::Identity(N_steps, N_steps), w);

            MatrixXd r = x_r * MatrixXd::Identity(1, 1);
            this->R = kroneckerProduct(MatrixXd::Identity(N, N), r);
        }
        else if (direction == 1) // y direction
        {
            Matrix2d q;
            q << y_q1, 0, 0, y_q2;
            this->Q = kroneckerProduct(MatrixXd::Identity(N, N), q);

            MatrixXd w = y_w * MatrixXd::Identity(1, 1);
            this->W = kroneckerProduct(MatrixXd::Identity(N_steps, N_steps), w);

            MatrixXd r = y_r * MatrixXd::Identity(1, 1);
            this->R = kroneckerProduct(MatrixXd::Identity(N, N), r);
        }
        Huu = Gamma.transpose() * Q * Gamma + R;
        GammatQ = Gamma.transpose() * Q;
    }

    this->reference_velocity = reference_velocity;
    this->x0 = x0;
    this->remaining_time = remaining_time;
    this->support_foot = support_foot;
    this->p0 = p0;

    int Nb = int(step_time / Ts) - int(remaining_time / Ts);
    if (direction == 2)
    {
        MatrixXd F1 = MatrixXd::Ones(N, N);
        F1 = F1.triangularView<Eigen::Lower>();
        MatrixXd F21 = kroneckerProduct(MatrixXd::Identity(N_steps - 1, N_steps - 1), VectorXd::Ones(int(step_time / Ts)));
        MatrixXd F22 = VectorXd::Ones(int(remaining_time / Ts));
        MatrixXd F23 = VectorXd::Ones(Nb);
        MatrixXd F2 = MatrixXd::Zero(N, N_steps + 1);
        F2.block(0, 0, int(remaining_time / Ts), 1) = F22;
        F2.block(int(remaining_time / Ts), 1, (N_steps - 1) * int(step_time / Ts), N_steps - 1) = F21;
        F2.block(N - Nb, N_steps, Nb, 1) = F23;

        this->F = F1 * F2;
        this->E = p0 * VectorXd::Ones(N);
        this->d_ref = VectorXd::Zero(N_steps + 1);
        Vector2d zref;
        zref << p0, 0;
        this->x_ref = kroneckerProduct(VectorXd::Ones(N), zref);
    }
    else
    {
        this->ref_position = this->ref_position + (reference_velocity)*deltat;
        MatrixXd tmp = MatrixXd::Ones(N_steps, N_steps);
        tmp = tmp.triangularView<Eigen::Lower>();
        MatrixXd F2 = MatrixXd::Zero(N_steps + 1, N_steps);
        F2.block(1, 0, N_steps, N_steps) = tmp;

        MatrixXd F11 = kroneckerProduct(MatrixXd::Identity(N_steps - 1, N_steps - 1), VectorXd::Ones(int(step_time / Ts)));
        MatrixXd F12 = VectorXd::Ones(int(remaining_time / Ts));
        MatrixXd F13 = VectorXd::Ones(Nb);
        MatrixXd F1 = MatrixXd::Zero(N, N_steps + 1);
        F1.block(0, 0, int(remaining_time / Ts), 1) = F12;
        F1.block(int(remaining_time / Ts), 1, (N_steps - 1) * int(step_time / Ts), N_steps - 1) = F11;
        F1.block(N - Nb, N_steps, Nb, 1) = F13;

        this->F = F1 * F2;
        this->E = p0 * VectorXd::Ones(N);
        Vector2d ref;
        ref << ref_position, reference_velocity;
        this->x_ref = kroneckerProduct(VectorXd::Ones(N), ref);

        if (direction == 0)
            this->d_ref = VectorXd::Ones(N_steps) * step_time * reference_velocity;
        else
        {
            d_ref = VectorXd(N_steps);
            for (int i = 0; i < d_ref.size(); i++)
            {
                if (support_foot == -1)
                {
                    if (i % 2 == 0)
                        d_ref(i) = -footwidth;
                    else
                        d_ref(i) = footwidth;
                }
                else
                {
                    if (i % 2 == 0)
                        d_ref(i) = footwidth;
                    else
                        d_ref(i) = -footwidth;
                }
            }
        }
    }
}

void MPCController::plan(double *res)
{

    float tbegin = clock();

    MatrixXd fu1 = GammatQ * (Phi * x0 - x_ref);

    MatrixXd Hud = -R * F;
    MatrixXd Hdd1 = F.transpose() * R * F;
    MatrixXd fd1 = F.transpose() * R * E;

    MatrixXd fu2 = -R * E;
    MatrixXd Hdd2 = W;
    MatrixXd fd2 = -W * d_ref;

    MatrixXd Hdd = Hdd1 + Hdd2;
    MatrixXd fd = fd1 + fd2;
    MatrixXd fu = fu1 + fu2;

    MatrixXd H1(Huu.rows(), Huu.cols() + Hud.cols());
    MatrixXd H2(Hdd.rows(), Hud.rows() + Hdd.cols());

    H1 << Huu, Hud;
    H2 << Hud.transpose(), Hdd;
    MatrixXd Ht(H1.cols(), H1.rows() + H2.rows());
    Ht << H1.transpose(), H2.transpose();
    MatrixXd H = Ht.transpose();
    MatrixXd ft(fu.cols(), fu.rows() + fd.rows());
    ft << fu.transpose(), fd.transpose();
    MatrixXd f = ft.transpose();

    real_t H_array[H.rows() * H.cols()];
    RowMajMat::Map(H_array, H.rows(), H.cols()) = H;

    real_t f_array[f.rows() * f.cols()];
    RowMajMat::Map(f_array, f.rows(), f.cols()) = f;

    if (direction != 2)
    {

        float vl;
        float vh;
        float reach;
        if (direction == 0)
        {
            vl = -2;
            vh = 2;
            reach = 0.3;
        }
        else
        {
            vl = -2;
            vh = 2;
            reach = 0.15;
        }

        MatrixXd lbA1 = vl * MatrixXd::Ones(N, 1) - Phi_v * x0;
        ;
        MatrixXd ubA1 = vh * MatrixXd::Ones(N, 1) - Phi_v * x0;
        MatrixXd lbA2 = E - reach * VectorXd::Ones(N);
        MatrixXd ubA2 = E + reach * VectorXd::Ones(N);
        VectorXd lbA = VectorXd::Zero(2 * N);
        VectorXd ubA = VectorXd::Zero(2 * N);
        lbA.block(0, 0, N, 1) = lbA1;
        lbA.block(N, 0, N, 1) = lbA2;
        ubA.block(0, 0, N, 1) = ubA1;
        ubA.block(N, 0, N, 1) = ubA2;

        MatrixXd Pu = MatrixXd::Identity(N, N + N_steps);
        MatrixXd Pd = MatrixXd::Zero(N_steps, N + N_steps);
        Pd.block(0, N, N_steps, N_steps) = MatrixXd::Identity(N_steps, N_steps);

        MatrixXd EA1 = Gamma_v * Pu;
        MatrixXd EA2 = Pu - F * Pd;

        MatrixXd EA = MatrixXd::Zero(EA1.rows() + EA2.rows(), EA1.cols());
        EA.block(0, 0, EA1.rows(), EA1.cols()) = EA1;
        EA.block(EA1.rows(), 0, EA2.rows(), EA2.cols()) = EA2;

        MatrixXd lb = INT_MIN * MatrixXd::Ones(N + N_steps, 1);
        MatrixXd ub = INT_MAX * MatrixXd::Ones(N + N_steps, 1);

        real_t A_array[EA.rows() * EA.cols()];
        RowMajMat::Map(A_array, EA.rows(), EA.cols()) = EA;

        real_t lbA_array[lbA.rows() * lbA.cols()];
        RowMajMat::Map(lbA_array, lbA.rows(), lbA.cols()) = lbA;

        real_t ubA_array[ubA.rows() * ubA.cols()];
        RowMajMat::Map(ubA_array, ubA.rows(), ubA.cols()) = ubA;

        real_t lb_array[lb.rows() * lb.cols()];
        RowMajMat::Map(lb_array, lb.rows(), lb.cols()) = lb;

        real_t ub_array[ub.rows() * ub.cols()];
        RowMajMat::Map(ub_array, ub.rows(), ub.cols()) = ub;

        int_t nWSR = 30000;
        returnValue returnvalue;
        real_t xOpt[N + N_steps];
        Options myOptions;
        myOptions.setToMPC();
        myOptions.printLevel = PL_LOW;
        qp.setOptions(myOptions);
        if (isfirst)
        {
            returnvalue = qp.init(H_array, f_array, A_array, lb_array, ub_array, lbA_array, ubA_array, nWSR);
            isfirst = false;
        }
        else
        {
            returnvalue = qp.hotstart(H_array, f_array, A_array, lb_array, ub_array, lbA_array, ubA_array, nWSR);
        }
        real_t opt[N + N_steps];
        qp.getPrimalSolution(opt);

        for (int i = 0; i < N + N_steps; i++)
            res[i] = opt[i];
    }
    else
    {
        MatrixXd EA = MatrixXd::Identity(N, N + N_steps + 1);
        real_t A_array[EA.rows() * EA.cols()];
        RowMajMat::Map(A_array, EA.rows(), EA.cols()) = EA;
        MatrixXd lb = INT_MIN * MatrixXd::Ones(N + N_steps + 1, 1);
        MatrixXd ub = INT_MAX * MatrixXd::Ones(N + N_steps + 1, 1);

        real_t lbA_array[N];
        real_t ubA_array[N];
        MatrixXd lbA = INT_MIN * MatrixXd::Ones(N, 1);
        RowMajMat::Map(lbA_array, lbA.rows(), lbA.cols()) = lbA;
        MatrixXd ubA = INT_MAX * MatrixXd::Ones(N, 1);
        RowMajMat::Map(ubA_array, ubA.rows(), ubA.cols()) = ubA;

        real_t lb_array[lb.rows() * lb.cols()];
        RowMajMat::Map(lb_array, lb.rows(), lb.cols()) = lb;

        real_t ub_array[ub.rows() * ub.cols()];
        RowMajMat::Map(ub_array, ub.rows(), ub.cols()) = ub;

        int_t nWSR = 30000;
        returnValue returnvalue;
        real_t xOpt[N + N_steps + 1];
        Options myOptions;
        myOptions.setToMPC();
        myOptions.printLevel = PL_LOW;
        qp.setOptions(myOptions);
        if (isfirst)
        {
            returnvalue = qp.init(H_array, f_array, A_array, lb_array, ub_array, lbA_array, ubA_array, nWSR);
            isfirst = false;
        }
        else
        {
            returnvalue = qp.hotstart(H_array, f_array, A_array, lb_array, ub_array, lbA_array, ubA_array, nWSR);
        }

        real_t opt[N + N_steps + 1];
        qp.getPrimalSolution(opt);

        for (int i = 0; i < N + N_steps + 1; i++)
            res[i] = opt[i];
    }
}

Zreference::Zreference(double g, double w_z, double Ts)
{
    this->g = g;
    this->w_z = w_z;
    this->Ts = Ts;
}

void Zreference::newStepUpdate(double z0, double zd0, double r0, double rT)
{
    this->r0 = r0;
    this->rT = rT;
    this->zd0 = zd0;
    this->s1 = z0 - r0 + g / (w_z * w_z);
}

void Zreference::caculate_zt(double remaining_time)
{
    double t = Ts - remaining_time;
    double rt = r0 + (rT - r0) * t / Ts;
    this->s2 = zd0 / (w_z * w_z) - rt / (Ts * w_z) + r0 / (Ts * w_z);
    zt = s1 * cos(w_z * t) + s2 * sin(w_z * t) + rt - g / (w_z * w_z);
    zdt = -s1 * w_z * sin(w_z * t) + s2 * w_z * cos(w_z * t) + (rT - r0) / Ts;
    ut = rt - g / (w_z * w_z);
}