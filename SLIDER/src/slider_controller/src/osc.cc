
/******************************************************************************
-Last Modified by: Siyi
-SLIDER project, Robot Intelligence Lab, Imperial College London, 2023
******************************************************************************/
#include "osc.h"

void OSC::updateData(const pin::Model &model, pin::Data &data,
                     const VectorXd &q_in, const VectorXd &v_in, const Support whichLeg, int counter)
{
  nq = model.nq;
  nv = model.nv;
  q = q_in;
  v = v_in;

  pin::forwardKinematics(model, data, q);

  // Computes global COM, COM Jacobian and joint Jacobian.
  pin::jacobianCenterOfMass(model, data, q, false);
  // cout << "------------COM jacobian------------" << endl;

  Jcom = data.Jcom;

  if (counter == 0)
  {
    Jcom_last = Jcom;
  }
  dJcom = (Jcom - Jcom_last) / 0.001;
  Jcom_last = Jcom;

  // These two functions have to be called before running getFrameJacobian
  pin::computeJointJacobians(model, data, q);
  pin::framesForwardKinematics(model, data, q);

  // Centroidal Momentum Matrix
  pin::computeCentroidalMap(model, data, q);
  Ag = data.Ag;
  pin::computeCentroidalMapTimeVariation(model, data, q, v);
  dAg = data.dAg;

  // cout << "------------Pelvis jacobian------------" << endl;
  // // because the Jacobian returns the previous frame jacobian, so we use left_contact_1 to get the real foot Jacobian
  MatrixXd J_pelvis = MatrixXd::Zero(6, nv);
  pin::getFrameJacobian(model, data, model.getFrameId("base_link"), pin::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_pelvis);
  // cout<< "Base Jacobian is\n" << J_pelvis << endl;
  J_pelvis_angular = J_pelvis.block(3, 0, 3, nv);

  // cout << "------------Left foot jacobian------------" << endl;
  J_left = MatrixXd::Zero(6, nv);
  pin::getFrameJacobian(model, data, model.getFrameId("Left_Foot"), pin::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_left);

  J_right = MatrixXd::Zero(6, nv);
  pin::getFrameJacobian(model, data, model.getFrameId("Right_Foot"), pin::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_right);

  MatrixXd J_left_pad1 = MatrixXd::Zero(6, nv);
  MatrixXd J_left_pad2 = MatrixXd::Zero(6, nv);
  MatrixXd J_left_pad3 = MatrixXd::Zero(6, nv);
  MatrixXd J_left_pad4 = MatrixXd::Zero(6, nv);
  pin::getFrameJacobian(model, data, model.getFrameId("Left_Foot_A"), pin::ReferenceFrame::LOCAL, J_left_pad1);
  pin::getFrameJacobian(model, data, model.getFrameId("Left_Foot_B"), pin::ReferenceFrame::LOCAL, J_left_pad2);
  pin::getFrameJacobian(model, data, model.getFrameId("Left_Foot_C"), pin::ReferenceFrame::LOCAL, J_left_pad3);
  pin::getFrameJacobian(model, data, model.getFrameId("Left_Foot_D"), pin::ReferenceFrame::LOCAL, J_left_pad4);

  MatrixXd J_left_pad1_linear = J_left_pad1.block(0, 0, 3, nv);
  MatrixXd J_left_pad2_linear = J_left_pad2.block(0, 0, 3, nv);
  MatrixXd J_left_pad3_linear = J_left_pad3.block(0, 0, 3, nv);
  MatrixXd J_left_pad4_linear = J_left_pad4.block(0, 0, 3, nv);

  // cout << "------------Right foot pad jacobian------------" << endl;
  MatrixXd J_right_pad1 = MatrixXd::Zero(6, nv);
  MatrixXd J_right_pad2 = MatrixXd::Zero(6, nv);
  MatrixXd J_right_pad3 = MatrixXd::Zero(6, nv);
  MatrixXd J_right_pad4 = MatrixXd::Zero(6, nv);
  pin::getFrameJacobian(model, data, model.getFrameId("Right_Foot_A"), pin::ReferenceFrame::LOCAL, J_right_pad1);
  pin::getFrameJacobian(model, data, model.getFrameId("Right_Foot_B"), pin::ReferenceFrame::LOCAL, J_right_pad2);
  pin::getFrameJacobian(model, data, model.getFrameId("Right_Foot_C"), pin::ReferenceFrame::LOCAL, J_right_pad3);
  pin::getFrameJacobian(model, data, model.getFrameId("Right_Foot_D"), pin::ReferenceFrame::LOCAL, J_right_pad4);

  MatrixXd J_right_pad1_linear = J_right_pad1.block(0, 0, 3, nv);
  MatrixXd J_right_pad2_linear = J_right_pad2.block(0, 0, 3, nv);
  MatrixXd J_right_pad3_linear = J_right_pad3.block(0, 0, 3, nv);
  MatrixXd J_right_pad4_linear = J_right_pad4.block(0, 0, 3, nv);

  // cout << "------------Combined support jacobian------------" << endl;
  n_ee = 2;              // Number of end-effectors
  n_contact = 4;         // Number of contact points per foot
  np = n_ee * n_contact; // Total number of contact points

  // The combined support jacobiean stack the end-effctor Jacobian vertically for each support point
  MatrixXd Jc(3 * np, nv);
  Jc << J_left_pad1_linear,
      J_left_pad2_linear,
      J_left_pad3_linear,
      J_left_pad4_linear,
      J_right_pad1_linear,
      J_right_pad2_linear,
      J_right_pad3_linear,
      J_right_pad4_linear;
  Jc_T = Jc.transpose();
  // cout << Jc_T << endl;

  // Computes joint Jacobian time derivative
  pin::computeJointJacobiansTimeVariation(model, data, q, v);

  dJ_left = MatrixXd::Zero(6, nv);
  pin::getFrameJacobianTimeVariation(model, data, model.getFrameId("Left_Foot"), pin::ReferenceFrame::LOCAL_WORLD_ALIGNED, dJ_left);

  dJ_right = MatrixXd::Zero(6, nv);
  pin::getFrameJacobianTimeVariation(model, data, model.getFrameId("Right_Foot"), pin::ReferenceFrame::LOCAL_WORLD_ALIGNED, dJ_right);

  MatrixXd dJ_pelvis = MatrixXd::Zero(6, nv);
  pin::getFrameJacobianTimeVariation(model, data, model.getFrameId("base_link"), pin::ReferenceFrame::LOCAL_WORLD_ALIGNED, dJ_pelvis);
  dJ_pelvis_angular = dJ_pelvis.block(3, 0, 3, nv);

  // Computes COM position, velocity
  pin::centerOfMass(model, data, q, v, false);

  // Computes the upper triangular part of the joint space inertia matrix M
  pin::crba(model, data, q);
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();

  M = data.M;

  // Computes the nonlinear effects of the Lagrangian dynamics
  pin::nonLinearEffects(model, data, q, v);

  nle = data.nle;

  // Construct the actuated joint selection matrix, its transpose should be [zero; identity]
  n_actuator = nv - 6;
  Sa_T = MatrixXd::Zero(nv, n_actuator);
  Sa_T.block(6, 0, n_actuator, n_actuator) = MatrixXd::Identity(n_actuator, n_actuator);

  // Construct the contact force selection matrix, its transpose should be [zero; identity]
  Sf_T = MatrixXd::Zero(3 * np, 3 * np);
  const int dim_fi = 3 * n_contact; // Dimension of the force vector per foot
  switch (whichLeg)
  {
  case Support::L:
    Sf_T.block(dim_fi, dim_fi, dim_fi, dim_fi) = MatrixXd::Identity(dim_fi, dim_fi);
    break;
  case Support::R:
    Sf_T.block(0, 0, dim_fi, dim_fi) = MatrixXd::Identity(dim_fi, dim_fi);
    break;
  case Support::D:
    break;
  case Support::F:
    Sf_T = MatrixXd::Identity(3 * np, 3 * np);
    break;
  }
  // cout << "------------Transpose of the contact force selection matrix (24*24 for SLIDER)------------" << endl;
  // cout << Sf_T << endl;
}

VectorXd OSC::solveQP(const Vector3d &ddx_com, const Vector3d &dh_ang, const Vector3d &ddx_left, const Vector3d &ddx_right, const Vector3d &ddx_left_ori, const Vector3d &ddx_right_ori, const Vector3d &ddx_base_orientation, const VectorXd &w, const double k, const VectorXd &torque_limit, int counter)
{
  // Weighting matrix in the objective function
  MatrixXd W(21, 21);
  W = w.asDiagonal();

  // Construct QP objective function
  MatrixXd A(21, nv);
  A << Jcom,
      Ag.block(3, 0, 3, nv),
      J_left,
      J_right,
      J_pelvis_angular;

  MatrixXd dA(21, nv);
  dA << dJcom,
      dAg.block(3, 0, 3, nv),
      dJ_left,
      dJ_right,
      dJ_pelvis_angular;

  MatrixXd ddx_cmd(21, 1);
  ddx_cmd << ddx_com,
      dh_ang,
      ddx_left,
      ddx_left_ori,
      ddx_right,
      ddx_right_ori,
      ddx_base_orientation;

  MatrixXd H0 = 2 * A.transpose() * W * A;
  VectorXd g0 = 2 * A.transpose() * W * (dA * v - ddx_cmd);

  MatrixXd H = MatrixXd::Zero(n_variables, n_variables); // Hessian matrix of the QP
  H.block(0, 0, nv, nv) = H0;
  H = H + 0.0001 * MatrixXd::Identity(n_variables, n_variables); // This trick makes H positve definite

  VectorXd g = VectorXd::Zero(n_variables, 1); // Gradient vector of the QP
  g.block(0, 0, nv, 1) = g0;

  // Construct QP constraints
  MatrixXd B(4, 3);
  B << 1, 0, -k,
      -1, 0, -k,
      0, 1, -k,
      0, -1, -k;

  MatrixXd Bt = MatrixXd::Zero(4 * np, 3 * np);
  for (int i = 0; i < np; i++)
  {
    Bt.block(4 * i, 3 * i, 4, 3) = B;
  }

  MatrixXd A_QP = MatrixXd::Zero(n_constraints, n_variables);
  A_QP.block(0, 0, nv, nv) = M;
  A_QP.block(0, nv, nv, 3 * np) = -Jc_T;
  A_QP.block(nv, nv, 4 * np, 3 * np) = Bt;
  A_QP.block(nv + 4 * np, nv, 3 * np, 3 * np) = Sf_T;

  // Lower bounds of the constraints
  VectorXd lbA = VectorXd::Zero(n_constraints, 1);
  lbA.block(0, 0, nv, 1) = -nle;
  lbA.block(6, 0, n_actuator, 1) -= torque_limit;
  lbA.block(nv, 0, 4 * np, 1) = -inf * VectorXd::Ones(4 * np, 1);

  VectorXd lb = -inf * VectorXd::Ones(n_variables, 1);
  for (int i = 0; i < np; i++)
  {
    lb(nv + 2 + 3 * i) = 0;
  }

  // Upper bounds of the constraints
  VectorXd ubA = VectorXd::Zero(n_constraints, 1);
  ubA.block(0, 0, nv, 1) = -nle;
  ubA.block(6, 0, n_actuator, 1) += torque_limit;

  VectorXd ub = inf * VectorXd::Ones(n_variables, 1);

  lb[3] = -1.0; // base roll
  ub[3] = 1.0;  // base roll

  lb[4] = -1.5; // base pitch
  ub[4] = 1.5;  // base pitch

  lb[5] = -0.5; // base yaw
  ub[5] = 0.5;  // base yaw

  // Solving QP

  /* Setup data for the QP. */
  real_t H_array[n_variables * n_variables];
  real_t A_array[n_constraints * n_variables];
  real_t g_array[n_variables];
  real_t lb_array[n_variables];
  real_t ub_array[n_variables];
  real_t lbA_array[n_constraints];
  real_t ubA_array[n_constraints];

  RowMajMat::Map(H_array, H.rows(), H.cols()) = H;
  RowMajMat::Map(A_array, A_QP.rows(), A_QP.cols()) = A_QP;
  RowMajMat::Map(g_array, g.rows(), g.cols()) = g;
  RowMajMat::Map(lb_array, lb.rows(), lb.cols()) = lb;
  RowMajMat::Map(ub_array, ub.rows(), ub.cols()) = ub;
  RowMajMat::Map(lbA_array, lbA.rows(), lbA.cols()) = lbA;
  RowMajMat::Map(ubA_array, ubA.rows(), ubA.cols()) = ubA;

  /* Solve QP. */
  int_t nWSR = 10000; // Maximum number of working set recalculations
  returnValue returnValue;

  Options myOptions;
  myOptions.setToMPC();
  myOptions.printLevel = PL_LOW;
  qp.setOptions(myOptions);

  if (counter == 0)
  {

    returnValue = qp.init(H_array, g_array, A_array, lb_array, ub_array, lbA_array, ubA_array, nWSR, 0);
  }
  else
  {
    returnValue = qp.hotstart(H_array, g_array, A_array, lb_array, ub_array, lbA_array, ubA_array, nWSR, 0);
  }



  /* Get and print solution of the QP. */
  VectorXd tau;
  real_t xOpt[n_variables];
  qp.getPrimalSolution(xOpt);

  VectorXd qdd_Opt = Map<VectorXd>(xOpt, 16);
  VectorXd f_Opt = Map<VectorXd>((xOpt + 16), 24);

  tau = M.bottomRows(n_actuator) * qdd_Opt + nle.bottomRows(n_actuator) - Jc_T.bottomRows(n_actuator) * f_Opt;

  return tau;
}