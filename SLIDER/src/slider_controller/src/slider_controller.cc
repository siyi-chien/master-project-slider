
/******************************************************************************
-First Author:  Zhonghe Jiang (Harry)
-Last Modified by: Siyi
-SLIDER project, Robot Intelligence Lab, Imperial College London, 2023
******************************************************************************/

#include "osc.h"
#include "gen_trajectory.h"
#include "define.h"
#include "LIPM.h"
#include "ros/ros.h"
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>

#include <slider_controller/Controller_paramaters.h>
#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/LinkStates.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <iostream>
#include <chrono>
#include <cmath>
#include <vector>

// The joint configuration (angle, velocity) should be a column Eigen vector,
// this should be obtained from sensor data (e.g. Gazebo joint data).
bool flag1 = true;
bool flag2 = false;
bool flag3 = true;
VectorXd q(17);
VectorXd v(16);
VectorXd q_base(7);
VectorXd q_joint(10);
VectorXd v_base(6);
VectorXd v_joint(10);
Vector3d left_foot_pos, right_foot_pos, base_euler_angle;
const double alpha = 10.0 * 3.14159 / 180.0;
const double x_offset = 0.0625; //
// Desired first stage trajectories
Vector3d ddx_com, dx_com, x_com, ddx_left, dx_left, x_left, ddx_right, dx_right, x_right;
int support_foot_flag;
Support whichLeg;
// Actual end-effector position, velocity and acceleration
Vector3d left_pos, left_vel, left_acc, right_pos, right_vel, right_acc;
Vector3d left_angVel, right_angVel;
Matrix3d left_rot, right_rot;
// Result obtained from the MPC planner
VectorXd zmp_foothold = Eigen::VectorXd::Zero(4);
Vector3d Z_desired;
Vector2d Z_steps;
VectorXd first_zmp_foothold = Eigen::VectorXd::Zero(4);
Vector3d first_Z_desired;
Vector2d first_Z_steps;
double remaining_time = 0;
double current_time = 0;

// Swing foot start position
double right_foot_start_x = 0;
double right_foot_start_y = 0;
double left_foot_start_x = 0;
double left_foot_start_y = 0;

Vector2d x_hat, y_hat, z_hat;
// Next CoM state to be tracked by the OSC controller
Vector2d x_next_state, y_next_state;

// Next foot trajectory (pos, vel, acc) in the x, y and z direction
Vector3d x_next_foot, y_next_foot, z_next_foot;

// Next com trajectory (pos, vel, acc) in the x, y direction
Vector3d x_next_com, y_next_com;

Vector3d ddx_com_cmd, ddx_left_cmd, ddx_right_cmd, ddx_pelvis_orientation, ddx_left_ori_cmd, ddx_right_ori_cmd, left_ankle_ori_feedback, right_ankle_ori_feedback;
Vector3d dh_ang_cmd;
VectorXd w(21);

double k = 0.75 / 1.414;   // The ground friction coefficient is 0.75
VectorXd torque_limit(10); // The torque limit on each motor


// Optimal torque
VectorXd traj_stage2;
Matrix3d R_wb;
Matrix3d R_wb_T;
Vector3d eps_m, r_vrp, Fd;
Vector3d h_ang;
Vector3d dh_ang;

std_msgs::Float64 getROSmsg(double torque)
{
    std_msgs::Float64 msg;
    msg.data = torque;
    return msg;
}

struct EulerAngles
{
    double roll, pitch, yaw;
};

Vector3d ToEulerAngles(Quaternionf q)
{
    EulerAngles angles;
    Vector3d euler_angles;

    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
    angles.roll = atan(sinr_cosp / cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
    if (fabs(sinp) >= 1)
        angles.pitch = copysign(PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    angles.yaw = atan(siny_cosp / cosy_cosp);
    euler_angles << angles.roll, angles.pitch, angles.yaw;

    return euler_angles;
}

/*!
 * Convert a quaternion to a rotation matrix.  This matrix represents a
 * coordinate transformation into the frame which has the orientation specified
 * by the quaternion
 */
Matrix3d quaternionToRotationMatrix(const VectorXd &q)
{
    double e0 = q(3); // w
    double e1 = q(0); // x
    double e2 = q(1); // y
    double e3 = q(2); // z

    Matrix3d R;
    R << 1 - 2 * (e2 * e2 + e3 * e3), 2 * (e1 * e2 - e0 * e3),
        2 * (e1 * e3 + e0 * e2), 2 * (e1 * e2 + e0 * e3),
        1 - 2 * (e1 * e1 + e3 * e3), 2 * (e2 * e3 - e0 * e1),
        2 * (e1 * e3 - e0 * e2), 2 * (e2 * e3 + e0 * e1),
        1 - 2 * (e1 * e1 + e2 * e2);
    // R.transposeInPlace();
    return R;
}

// Joint_state callback function
// file: sensor_msgs/JointState.msg
void jointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
{ // left right
    // roll pitch slide pitch_foot, roll_foot
    q_joint << msg->position[3], msg->position[2], msg->position[4], msg->position[0], msg->position[1],
        msg->position[8], msg->position[7], msg->position[9], msg->position[5], msg->position[6];
    v_joint << msg->velocity[3], msg->velocity[2], msg->velocity[4], msg->velocity[0], msg->velocity[1],
        msg->velocity[8], msg->velocity[7], msg->velocity[9], msg->velocity[5], msg->velocity[6];
}

// Link_state callback function
// file: gazebo_msgs/Linkstate.msg
void linkStateCallback(const gazebo_msgs::LinkStates::ConstPtr &msg)
{
    q_base << msg->pose[1].position.x, msg->pose[1].position.y, msg->pose[1].position.z,
        msg->pose[1].orientation.x, msg->pose[1].orientation.y, msg->pose[1].orientation.z, msg->pose[1].orientation.w;

    v_base << msg->twist[1].linear.x, msg->twist[1].linear.y, msg->twist[1].linear.z,
        msg->twist[1].angular.x, msg->twist[1].angular.y, msg->twist[1].angular.z;

    left_foot_pos << msg->pose[6].position.x, msg->pose[6].position.y, (msg->pose[6].position.z - 0.04);
    right_foot_pos << msg->pose[11].position.x, msg->pose[11].position.y, (msg->pose[11].position.z - 0.04);

    ofstream co;
    co.open("/home/siyi/SLIDER/src/slider_controller/states.txt", ios::app);
    co << "****************************************************" << endl;
    co << left_foot_pos << endl;
    co << right_foot_pos << endl;
    co.close();
}

// Callback function for receiving the planner outputs
void zmpFootholdCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    zmp_foothold(0) = msg->data[0];
    zmp_foothold(1) = msg->data[1];
    zmp_foothold(2) = msg->data[2];
    zmp_foothold(3) = msg->data[3];
    Z_desired(0) = msg->data[4];
    Z_desired(1) = msg->data[5];
    Z_desired(2) = msg->data[6];
    Z_steps(0) = msg->data[7];
    Z_steps(1) = msg->data[8];

    if (zmp_foothold(0) != 0 && flag3)
    {
        first_zmp_foothold = zmp_foothold;
        first_Z_desired = Z_desired;
        first_Z_steps = Z_steps;
        flag3 = false;
    }
}

double Kp_com = -70;         // 70
double Kd_com = -5;          // 5
double Kp_orientation = -50; // 70
double Kd_orientation = 5;   // 5
double Kp_left = -70;        //-70
double Kd_left = -5;         //-5
double Kp_right = -70;       //-70
double Kd_right = -5;        //-5
double kp_ang = -20;         //-20
double kd_ang = -2;          //-2
double Kp_foot_ori = -15;    //-15
double Kd_foot_ori = 1;      // 1

float S_Kp_com = -100;
float S_Kd_com = -10;
float S_Kp_left = -50;
float S_Kd_left = -5;
float S_Kp_right = -50;
float S_Kd_right = -5;
float S_Kp_foot_ori = 100;
float S_Kd_foot_ori = -10;
float S_Kp_ang = -0;
float S_Kd_ang = -0;
float S_Kp_orientation = 100;
float S_Kd_orientation = -10;
bool isFirstStep = true;

float R_Kp_com = -100;
float R_Kd_com = -10;
float R_Kp_left = -400;
float R_Kd_left = -40;
float R_Kp_right = -100;
float R_Kd_right = -10;
float R_Kp_foot_ori = 10;
float R_Kd_foot_ori = -1;
float R_Kp_ang = -1;
float R_Kd_ang = -0.1;
float R_Kp_orientation = 400;
float R_Kd_orientation = -40;

int counter = 0;

bool controllerParameterCallback(slider_controller::Controller_paramaters::Request &req, slider_controller::Controller_paramaters::Response &res)
{
    R_Kp_com = req.R_Kp_com;
    R_Kd_com = req.R_Kd_com;
    R_Kp_left = req.R_Kp_left;
    R_Kd_left = req.R_Kd_left;
    R_Kp_right = req.R_Kp_right;
    R_Kd_right = req.R_Kd_right;
    R_Kp_foot_ori = req.R_Kp_foot_ori;
    R_Kd_foot_ori = req.R_Kd_foot_ori;
    R_Kp_ang = req.R_Kp_ang;
    R_Kd_ang = req.R_Kd_ang;
    R_Kp_orientation = req.R_Kp_orientation;
    R_Kd_orientation = req.R_Kd_orientation;

    res.success = true;
}

// create a ros client for  parameter_selftuning
string filePath = "/home/siyi/SLIDER/src/slider_controller/data/";
std::string filename_urdf = filePath + "SLIDER_ROTOGAIT_FOOT_pin.urdf";
bool reset_counter(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res)
{

    counter = 0;
    whichLeg = Support::S;
    remaining_time = 0;
    current_time = 0;

    // Swing foot start position
    right_foot_start_x = 0;
    right_foot_start_y = 0;
    left_foot_start_x = 0;
    left_foot_start_y = 0;

    q_base << 0, 0, 0, 0, 0, 0, 1;
    isFirstStep = true;
    flag2 = true;
    zmp_foothold = first_zmp_foothold;
    Z_desired = first_Z_desired;
    Z_steps = first_Z_steps;
    R_wb_T = Eigen::Matrix3d::Zero();

    return true;
}

int main(int argc, char **argv)
{

    //********************************************************************************************
    //****************************ROS node initialization*****************************************
    //********************************************************************************************
    ros::init(argc, argv, "ros_SLIDER_OSC_node");
    ros::NodeHandle n;
    // Load gazebo
    // client::setup(argc, argv);
    ros::Rate loop_rate(LOOP_RATE);

    // Publisher for ros torque control
    ros::Publisher left_hip_pitch_pub = n.advertise<std_msgs::Float64>("/slider_gazebo/left_hip_pitch_torque_controller/command", 1);
    ros::Publisher left_hip_roll_pub = n.advertise<std_msgs::Float64>("/slider_gazebo/left_hip_roll_torque_controller/command", 1);
    ros::Publisher left_hip_slide_pub = n.advertise<std_msgs::Float64>("/slider_gazebo/left_hip_slide_torque_controller/command", 1);
    ros::Publisher left_ankle_roll_pub = n.advertise<std_msgs::Float64>("/slider_gazebo/left_ankle_roll_torque_controller/command", 1);
    ros::Publisher left_ankle_pitch_pub = n.advertise<std_msgs::Float64>("/slider_gazebo/left_ankle_pitch_torque_controller/command", 1);

    ros::Publisher right_hip_pitch_pub = n.advertise<std_msgs::Float64>("/slider_gazebo/right_hip_pitch_torque_controller/command", 1);
    ros::Publisher right_hip_roll_pub = n.advertise<std_msgs::Float64>("/slider_gazebo/right_hip_roll_torque_controller/command", 1);
    ros::Publisher right_hip_slide_pub = n.advertise<std_msgs::Float64>("/slider_gazebo/right_hip_slide_torque_controller/command", 1);
    ros::Publisher right_ankle_roll_pub = n.advertise<std_msgs::Float64>("/slider_gazebo/right_ankle_roll_torque_controller/command", 1);
    ros::Publisher right_ankle_pitch_pub = n.advertise<std_msgs::Float64>("/slider_gazebo/right_ankle_pitch_torque_controller/command", 1);

    // Publish the data used for MPC planner
    ros::Publisher planner_input_pub = n.advertise<std_msgs::Float64MultiArray>("/slider_gazebo/planner_input", 1);

    // Subscribe the Gazebo joint_states topic
    ros::Subscriber sub_joint = n.subscribe<sensor_msgs::JointState>("/slider_gazebo/joint_states", 1, jointStateCallback);
    ros::Subscriber sub_link = n.subscribe<gazebo_msgs::LinkStates>("/gazebo/link_states", 1, linkStateCallback);
    // ros::Subscriber sub_modelstates = n.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 1, modelStateCallback);
    // Subscribe the data from the MPC planner
    ros::Subscriber sub_planner_output = n.subscribe<std_msgs::Float64MultiArray>("/slider_gazebo/zmp_foothold", 1, zmpFootholdCallback);

    // subscribe to the parameter of the controller
    // ros::Subscriber sub_controller_parameter = n.subscribe<slider_controller::Controller_paramaters>("/slider_gazebo/controller_parameters", 1, controllerParameterCallback);
    ros::ServiceServer srv_controller_parameter = n.advertiseService("/slider_gazebo/controller_parameters", controllerParameterCallback);
    // create a ROS service
    ros::ServiceServer srv_reset_count = n.advertiseService("/slider_gazebo/reset_counter", reset_counter);

    // ros::ServiceClient client = n.serviceClient<slider_controller::Controller_paramaters>("/slider_gazebo/controller_parameters");
    //********************************************************************************************
    //****************************Prepare for the planner I/O*************************************
    //********************************************************************************************
    //  Linver Inverted Pendulum model
    Matrix2d A_loop_rate;
    Vector2d B_loop_rate;
    genLIPM(A_loop_rate, B_loop_rate, (1.0 / LOOP_RATE));

    // Current support foot
    whichLeg = Support::S;

    // Remaining and current time for the current step
    remaining_time = 0;
    current_time = 0;

    // Swing foot start position
    right_foot_start_x = 0;
    right_foot_start_y = 0;
    left_foot_start_x = 0;
    left_foot_start_y = 0;

    // Initial coronal sway (take a right step first)
    double y_start = -step_width / 2;                                                     // origin relative to the left foot
    double ydt_start = calc_Vel0_given_TPos0Vel1(k_LIPM, step_time / 2, y_start, 0);      // Initial velocity necessary to travel to apex in Tss/2
    double y_apex = calc_Pos1_given_TPos0Vel0(k_LIPM, step_time / 2, y_start, ydt_start); // relative to the left foot
    y_apex = -step_width / 2 - y_apex;                                                    // relative to the origin
    // cout << "ydt_start: " << ydt_start << endl;
    // cout << "y_apex: " << y_apex << endl;

    //********************************************************************************************
    //*********************************OSC initialization*****************************************
    //********************************************************************************************

    // Build kinematics and dynamics model from urdf

    pin::Model model;
    pin::urdf::buildModel(filename_urdf, model);
    pin::Data data(model);

    // Initialize OSC controller
    OSC OSC_controller;

    // Commanded acceleration given to OSC controller
    torque_limit << 1500, // Left hip roll
        600,              // Left hip pitch
        600,              // Left hip slide
        300,              // Left ankle pitch
        300,              // Left ankle roll
        1500,             // Right hip roll
        600,              // Right hip pitch
        600,              // Right hip slide
        300,              // Right ankle pitch
        300;              // Right ankle roll

    // Parameters for regulating the base orientation
    Quaternionf base_orientation;
    isFirstStep = true;

    //***************************************************************************************
    //****************************Online Calculations****************************************
    //***************************************************************************************

    q_base << 0, 0, 0, 0, 0, 0, 1;
    while (ros::ok())
    {

        //***************************************************************************************
        //******************************State Estimation*****************************************
        //***************************************************************************************
        // get base orientation represented in Euler angles
        base_orientation.x() = q_base[3];
        base_orientation.y() = q_base[4];
        base_orientation.z() = q_base[5];
        base_orientation.w() = q_base[6];
        base_euler_angle = ToEulerAngles(base_orientation);
        // cout<<"The pelvis orientation is " << base_euler_angle << endl;

        // Rotation matrix
        R_wb = quaternionToRotationMatrix(q_base.tail(4));

        // Update joint position and velocity
        if ((whichLeg == Support::S) && (counter <= update_delay))
        {
            q << 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
            v << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
            base_euler_angle << 0.0, 0.0, 0.0;
            v_base << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }
        else
        {
            // Rotation matrix
            // R_wb = quaternionToRotationMatrix(q_base.tail(4));
            R_wb_T = R_wb.transpose();
            //  cout << "Rotation matrix\n" << R_wb << endl;

            q << q_base, q_joint;
            // v << v_base, v_joint;
            v << R_wb_T * v_base.head(3),
                R_wb_T * v_base.tail(3),
                v_joint;
        }

        // Update date based on the current configuration of the robot
        OSC_controller.updateData(model, data, q, v, whichLeg, counter);

        left_pos = data.oMf[model.getFrameId("Left_Foot")].translation();
        left_rot = data.oMf[model.getFrameId("Left_Foot")].rotation();

        // cout << "------------Left foot position------------" << endl;

        // cout << left_pos << endl;
        //  cout << "------------Left foot velocity------------" << endl;
        left_vel = pin::getFrameVelocity(model, data, model.getFrameId("Left_Foot"), pin::ReferenceFrame::LOCAL_WORLD_ALIGNED).linear();
        // cout << left_vel << endl;
        left_angVel = pin::getFrameVelocity(model, data, model.getFrameId("Left_Foot"), pin::ReferenceFrame::LOCAL).angular();

        //  cout << "------------Left foot acceleration------------" << endl;
        left_acc = pin::getFrameAcceleration(model, data, model.getFrameId("Left_Foot"), pin::ReferenceFrame::LOCAL_WORLD_ALIGNED).linear();
        // cout << left_acc << endl;

        //  cout << "------------Right foot position------------" << endl;
        right_pos = data.oMf[model.getFrameId("Right_Foot")].translation();
        right_rot = data.oMf[model.getFrameId("Right_Foot")].rotation();
        // cout << "right foot pos is " << endl;
        //  cout << right_pos << endl;
        //   cout << "------------Right foot velocity------------" << endl;
        right_vel = pin::getFrameVelocity(model, data, model.getFrameId("Right_Foot"), pin::ReferenceFrame::LOCAL_WORLD_ALIGNED).linear();
        right_angVel = pin::getFrameVelocity(model, data, model.getFrameId("Right_Foot"), pin::ReferenceFrame::LOCAL).angular();
        //   cout << right_vel << endl;
        //    cout << "------------Right foot acceleration------------" << endl;
        right_acc = pin::getFrameAcceleration(model, data, model.getFrameId("Right_Foot"), pin::ReferenceFrame::LOCAL_WORLD_ALIGNED).linear();
        //     cout << right_acc << endl;

        pin::computeCentroidalMomentumTimeVariation(model, data);
        // pin::computeCentroidalMomentum(model, data);

        h_ang = data.hg.angular();
        dh_ang = data.dhg.angular();

        x_hat << data.com[0][0], data.vcom[0][0];
        y_hat << data.com[0][1], data.vcom[0][1];
        z_hat << data.com[0][2], data.vcom[0][2];

        // Publish data for the MPC planner, it will recreate every iteration
        std_msgs::Float64MultiArray planner_input;

        planner_input.data.push_back(x_hat(0));
        planner_input.data.push_back(x_hat(1));
        planner_input.data.push_back(y_hat(0));
        planner_input.data.push_back(y_hat(1));
        planner_input.data.push_back(z_hat(0));
        planner_input.data.push_back(z_hat(1));
        // cout << "--------New iteration (" << whichLeg << ")----------" << endl;

        //***************************************************************************************
        //*****************************Run State Machine*****************************************
        //***************************************************************************************

        // Run state machine and the corresponding planner

        switch (whichLeg)
        {
        case Support::S: // start motion (assuming take a right step first)

            current_time = double(counter) / LOOP_RATE;

            // desired left foot trajectory
            ddx_left = Vector3d::Zero();
            dx_left = left_vel;
            x_left = left_pos;

            // desired right foot trajectory
            ddx_right = Vector3d::Zero();
            dx_right = right_vel;
            x_right = right_pos;

            if (current_time <= initial_sway_time) // origin to apex
            {
                y_next_com = fifthOrderPolynomialInterpolation(initial_sway_time, current_time, 0, 0, 0, y_apex, 0, 0);
                ddx_com << 0, y_next_com(2), 0;
                dx_com << 0, y_next_com(1), 0;
                x_com << 0, y_next_com(0), com_height;
            }
            else // apex back to origin
            {
                ddx_com << 0, (GRAVITY / com_height) * (y_hat(0) - right_pos(1)), 0;
                y_next_state = A_loop_rate * y_hat + B_loop_rate * right_pos(1);
                dx_com << 0, y_next_state(1), 0;
                x_com << 0, y_next_state(0), com_height;

                if (y_hat(0) > 0.001)
                {
                    whichLeg = Support::L;
                    counter = 0;
                    remaining_time = step_time;
                    planner_input.data.push_back(remaining_time); // remaining time for the current step
                    planner_input.data.push_back(left_pos(0));    // current step location in the x direction
                    planner_input.data.push_back(left_pos(1));    // current step location in the y direction
                    planner_input.data.push_back(whichLeg);       // support foot
                    planner_input.data.push_back(0.0);            // current step location in the x direction
                    planner_input.data.push_back(0.0);            // current step location in the y direction
                    planner_input.data.push_back(0.0);            // support foot
                    planner_input.data.push_back(left_pos(2));    // swing foot
                    planner_input.data.push_back(right_pos(2));
                    planner_input_pub.publish(planner_input);
                }
            }
            // OSC weighting parameters
            w << 1, 1, 1,      // ddx_com
                0.1, 0.1, 0.1, // dh_ang
                1, 1, 1,       // ddx_left
                1, 1, 1,       // ddx_left_ori
                1, 1, 1,       // ddx_right
                1, 1, 1,       // ddx_right_ori
                1, 1, 1;       // ddx_base_orientation

            Kp_com = S_Kp_com;
            Kd_com = S_Kd_com;
            Kp_right = S_Kp_right;
            Kd_right = S_Kd_right;
            Kp_left = S_Kp_left;
            Kd_left = S_Kd_left;
            Kp_foot_ori = S_Kp_foot_ori;
            Kd_foot_ori = S_Kd_foot_ori;
            kp_ang = S_Kp_ang;
            kd_ang = S_Kd_ang;
            Kp_orientation = S_Kp_orientation;
            Kd_orientation = S_Kd_orientation;

            break;

        case Support::R: //

            current_time = double(counter) / LOOP_RATE;
            remaining_time = step_time - current_time;

            planner_input.data.push_back(remaining_time);    // remaining time for the current step
            planner_input.data.push_back(right_pos(0));      // current step location in the x direction
            planner_input.data.push_back(right_pos(1));      // current step location in the y direction
            planner_input.data.push_back(whichLeg);          // support foot
            planner_input.data.push_back(left_foot_pos[0]);  // swing foot
            planner_input.data.push_back(left_foot_pos[1]);  // swing foot
            planner_input.data.push_back(left_foot_pos[2]);  // swing foot
            planner_input.data.push_back(right_foot_pos[2]); // swing foot
            planner_input_pub.publish(planner_input);

            // desired right foot trajectory
            ddx_right = Vector3d::Zero();
            dx_right = right_vel;
            x_right = right_pos;

            // desired left foot trajectory
            x_next_foot = fifthOrderPolynomialInterpolation(step_time, current_time, left_foot_start_x, 0, 0, zmp_foothold(2), 0, 0); // x

            y_next_foot = fifthOrderPolynomialInterpolation(step_time, current_time, left_foot_start_y, 0, 0, zmp_foothold(3), 0, 0);                                                                      // y
            if (current_time <= (step_time / 2))                                                                                                                                                           // ground to peak
                z_next_foot = fifthOrderPolynomialInterpolation(step_time / 2, current_time, foot_height + Z_steps(0) - com_height, 0, 0, step_height + (Z_steps(0) + Z_steps(1)) / 2 - com_height, 0, 0); // z
            //                    z_next_foot = fifthOrderPolynomialInterpolation(step_time/2, current_time, foot_height+(left_foot_start_x-x_offset)*tan(alpha), 0, 0, step_height+((right_foot_start_x+zmp_foothold(2))/2-x_offset)*tan(alpha), 0, -1); //z
            else                                                                                                                                                                                                                // peak to ground
                z_next_foot = fifthOrderPolynomialInterpolation(step_time / 2, (current_time - step_time / 2), step_height + (Z_steps(0) + Z_steps(1)) / 2 - com_height, 0, 0, foot_height + Z_steps(1) - com_height, 0, -0.1); //+(zmp_foothold(2)-x_offset)
                                                                                                                                                                                                                                //                    z_next_foot = fifthOrderPolynomialInterpolation(step_time/2, (current_time - step_time/2), step_height+((right_foot_start_x+zmp_foothold(2))/2-x_offset)*tan(alpha), 0, -1, foot_height+(zmp_foothold(2)-x_offset)*tan(alpha), 0, 0); //+(zmp_foothold(2)-x_offset)
            ddx_left << x_next_foot(2), y_next_foot(2), z_next_foot(2);
            dx_left << x_next_foot(1), y_next_foot(1), z_next_foot(1);
            x_left << x_next_foot(0), y_next_foot(0), z_next_foot(0);

            // desired CoM trajectory
            ddx_com << (GRAVITY / com_height) * (x_hat(0) - zmp_foothold(0)), (GRAVITY / com_height) * (y_hat(0) - zmp_foothold(1)), Z_desired(2); // (GRAVITY/com_height)*(x_hat(0)-zmp_foothold(0))*tan(alpha)
            x_next_state = A_loop_rate * x_hat + B_loop_rate * zmp_foothold(0);
            y_next_state = A_loop_rate * y_hat + B_loop_rate * zmp_foothold(1);
            dx_com << x_next_state(1), y_next_state(1), Z_desired(1); // x_next_state(1)*tan(alpha)
            x_com << x_next_state(0), y_next_state(0), Z_desired(0);  // com_height+(x_next_state(0)-x_offset)*tan(alpha)

            // OSC weighting parameters
            w << 1, 1, 1,      // ddx_com
                0.1, 0.1, 0.1, // dh_ang
                1, 1, 1,       // ddx_left
                1, 1, 1,       // ddx_left_ori
                1, 1, 1,       // ddx_right
                1, 1, 1,       // ddx_right_ori
                1, 5, 1;       // ddx_base_orientation

            Kp_com = R_Kp_com;
            Kd_com = R_Kd_com;
            Kp_right = R_Kp_right;
            Kd_right = R_Kd_right;
            Kp_left = R_Kp_left;
            Kd_left = R_Kd_left;
            Kp_foot_ori = R_Kp_foot_ori;
            Kd_foot_ori = R_Kd_foot_ori;
            kp_ang = R_Kp_ang;
            kd_ang = R_Kd_ang;
            Kp_orientation = R_Kp_orientation;
            Kd_orientation = R_Kd_orientation;

            if (remaining_time < 1.0 / LOOP_RATE)
            {
                whichLeg = Support::L;
                counter = 0;
                remaining_time = step_time;
                planner_input.data.push_back(remaining_time);    // remaining time for the current step
                planner_input.data.push_back(left_pos(0));       // current step location in the x direction
                planner_input.data.push_back(left_pos(1));       // current step location in the y direction
                planner_input.data.push_back(whichLeg);          // support foot
                planner_input.data.push_back(right_foot_pos[0]); // swing foot
                planner_input.data.push_back(right_foot_pos[1]); // swing foot
                planner_input.data.push_back(right_foot_pos[2]); // swing foot
                planner_input.data.push_back(left_foot_pos[2]);  // swing foot
                planner_input_pub.publish(planner_input);

                right_foot_start_x = right_pos(0);
                right_foot_start_y = right_pos(1);
                // cout << "reach here" << endl;
            }

            break;

        case Support::L: //
            current_time = double(counter) / LOOP_RATE;
            remaining_time = step_time - current_time;
            // cout << "remaining_time: " << remaining_time << endl;
            planner_input.data.push_back(remaining_time);    // remaining time for the current step
            planner_input.data.push_back(left_pos(0));       // current step location in the x direction
            planner_input.data.push_back(left_pos(1));       // current step location in the y direction
            planner_input.data.push_back(whichLeg);          // support foot
            planner_input.data.push_back(right_foot_pos[0]); // swing foot
            planner_input.data.push_back(right_foot_pos[1]); // swing foot
            planner_input.data.push_back(right_foot_pos[2]); // swing foot
            planner_input.data.push_back(left_foot_pos[2]);  // swing foot
            planner_input_pub.publish(planner_input);

            // desired left foot trajectory
            ddx_left = Vector3d::Zero();
            dx_left = left_vel;
            x_left = left_pos;

            // desired right foot trajectory
            x_next_foot = fifthOrderPolynomialInterpolation(step_time, current_time, right_foot_start_x, 0, 0, zmp_foothold(2), 0, 0);                                                                     // x
            y_next_foot = fifthOrderPolynomialInterpolation(step_time, current_time, right_foot_start_y, 0, 0, zmp_foothold(3), 0, 0);                                                                     // y
            if (current_time <= (step_time / 2))                                                                                                                                                           // ground to peak
                z_next_foot = fifthOrderPolynomialInterpolation(step_time / 2, current_time, foot_height + Z_steps(0) - com_height, 0, 0, step_height + (Z_steps(0) + Z_steps(1)) / 2 - com_height, 0, 0); // z
            //                    z_next_foot = fifthOrderPolynomialInterpolation(step_time/2, current_time, foot_height+(right_foot_start_x-x_offset)*tan(alpha), 0, 0, step_height+((right_foot_start_x+zmp_foothold(2))/2-x_offset)*tan(alpha), 0, -1); //z
            else                                                                                                                                                                                                                // peak to ground
                z_next_foot = fifthOrderPolynomialInterpolation(step_time / 2, (current_time - step_time / 2), step_height + (Z_steps(0) + Z_steps(1)) / 2 - com_height, 0, 0, foot_height + Z_steps(1) - com_height, 0, -0.1); //
                                                                                                                                                                                                                                //                    z_next_foot = fifthOrderPolynomialInterpolation(step_time/2, (current_time - step_time/2), step_height+((right_foot_start_x+zmp_foothold(2))/2-x_offset)*tan(alpha), 0, -1, foot_height+(zmp_foothold(2)-x_offset)*tan(alpha), 0, 0); //
            ddx_right << x_next_foot(2), y_next_foot(2), z_next_foot(2);
            dx_right << x_next_foot(1), y_next_foot(1), z_next_foot(1);
            x_right << x_next_foot(0), y_next_foot(0), z_next_foot(0);

            // desired CoM trajectory
            ddx_com << (GRAVITY / com_height) * (x_hat(0) - zmp_foothold(0)), (GRAVITY / com_height) * (y_hat(0) - zmp_foothold(1)), Z_desired(2); //(GRAVITY/com_height)*(x_hat(0)-zmp_foothold(0))*tan(alpha)
            x_next_state = A_loop_rate * x_hat + B_loop_rate * zmp_foothold(0);
            y_next_state = A_loop_rate * y_hat + B_loop_rate * zmp_foothold(1);
            dx_com << x_next_state(1), y_next_state(1), Z_desired(1); // x_next_state(1)*tan(alpha)
            x_com << x_next_state(0), y_next_state(0), Z_desired(0);  // com_height+(x_next_state(0)-x_offset)*tan(alpha)

            // OSC weighting parameters
            w << 1, 1, 1,      // ddx_com
                0.1, 0.1, 0.1, // dh_ang
                1, 1, 1,       // ddx_left
                1, 1, 1,       // ddx_left_ori
                1, 1, 1,       // ddx_right
                1, 1, 1,       // ddx_right_ori
                1, 5, 1;       // ddx_base_orientation

            Kp_com = R_Kp_com;
            Kd_com = R_Kd_com;
            Kp_right = R_Kp_left;
            Kd_right = R_Kd_left;
            Kp_left = R_Kp_right;
            Kd_left = R_Kd_right;
            Kp_foot_ori = R_Kp_foot_ori;
            Kd_foot_ori = R_Kd_foot_ori;
            kp_ang = R_Kp_ang;
            kd_ang = R_Kd_ang;
            Kp_orientation = R_Kp_orientation;
            Kd_orientation = R_Kd_orientation;

            if (remaining_time < 1.0 / LOOP_RATE)
            {
                whichLeg = Support::R;
                counter = 0;
                remaining_time = step_time - double(counter) / LOOP_RATE;
                planner_input.data.push_back(remaining_time);    // remaining time for the current step
                planner_input.data.push_back(right_pos(0));      // current step location in the x direction
                planner_input.data.push_back(right_pos(1));      // current step location in the y direction
                planner_input.data.push_back(whichLeg);          // support foot
                planner_input.data.push_back(left_foot_pos[0]);  // swing foot
                planner_input.data.push_back(left_foot_pos[1]);  // swing foot
                planner_input.data.push_back(left_foot_pos[2]);  // swing foot
                planner_input.data.push_back(right_foot_pos[2]); // swing foot
                planner_input_pub.publish(planner_input);
                left_foot_start_x = left_pos(0);
                left_foot_start_y = left_pos(1);
                // cout << "reach here" << endl;
                if (isFirstStep)
                {
                    isFirstStep = false;
                }
            }

            break;
        }
        //***************************************************************************************
        //*****************************Run OSC controller****************************************
        //***************************************************************************************
        // PD and feedforward controller
        ddx_com_cmd = Kp_com * (data.com[0] - x_com) + Kd_com * (data.vcom[0] - dx_com) + ddx_com;
        dh_ang_cmd = kp_ang * h_ang + kd_ang * dh_ang;
        // ddx_pelvis_orientation = R_wb*Kp_orientation*(pin::log3(R_wb.transpose())) + Kd_orientation*(v_base.tail(3));
        double beta = 0;
        Matrix3d R_d;
        R_d << cos(beta), 0, sin(beta),
            0, 1, 0,
            -sin(beta), 0, cos(beta);
        ddx_pelvis_orientation = R_wb * Kp_orientation * (pin::log3(R_wb_T * R_d)) + Kd_orientation * (v_base.tail(3));

        ddx_left_cmd = Kp_left * (left_pos - x_left) + Kd_left * (left_vel - dx_left) + ddx_left;
        ddx_right_cmd = Kp_right * (right_pos - x_right) + Kd_right * (right_vel - dx_right) + ddx_right;

        Matrix3d left_rot_T = left_rot.transpose();
        Matrix3d right_rot_T = right_rot.transpose();

        ddx_left_ori_cmd = left_rot * (Kp_foot_ori * pin::log3(left_rot_T) + Kd_foot_ori * left_angVel);
        ddx_right_ori_cmd = right_rot * (Kp_foot_ori * pin::log3(right_rot_T) + Kd_foot_ori * right_angVel);

        if (counter > 500)
        {
            flag1 = false;
            flag2 = false;
        }

        // Solve for the optimal torque
        traj_stage2 = OSC_controller.solveQP(ddx_com_cmd, dh_ang_cmd, ddx_left_cmd, ddx_right_cmd, ddx_left_ori_cmd, ddx_right_ori_cmd, ddx_pelvis_orientation, w, k, torque_limit, counter);

        left_hip_roll_pub.publish(getROSmsg(traj_stage2[0]));
        left_hip_slide_pub.publish(getROSmsg(traj_stage2[2])); //
        left_ankle_roll_pub.publish(getROSmsg(traj_stage2[4]));

        right_hip_roll_pub.publish(getROSmsg(traj_stage2[5]));
        right_hip_slide_pub.publish(getROSmsg(traj_stage2[7]));
        right_ankle_roll_pub.publish(getROSmsg(traj_stage2[9]));

        left_hip_pitch_pub.publish(getROSmsg(traj_stage2[1]));
        left_ankle_pitch_pub.publish(getROSmsg(traj_stage2[3]));
        right_hip_pitch_pub.publish(getROSmsg(traj_stage2[6])); //
        right_ankle_pitch_pub.publish(getROSmsg(traj_stage2[8]));
        counter++;
        //  cout << "counter number is "<< counter << endl;

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}