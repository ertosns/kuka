#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include "math.h"
#include <cmath>
#include "gmock/gmock.h"
#include "../include/kuka.hpp"
#include "../include/controller.hpp"

using namespace testing;
using namespace std;

TEST(KUKA, macanum_translation1) {
    //define state
    State state(4, 5, 0.01);
    Eigen::VectorXd configuration(12);
    configuration << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    Eigen::VectorXd speed(9);
    speed << 0, 0, 0, 0, 0, 10, 10, 10, 10;
    state(configuration, speed);
    //define macanum
    const double l=0.235; // axial distance 2l=0.47
    const double w=0.15; // wheel to wheel 2w=0.3
    const double r=0.0475;
    Eigen::Matrix4d Tb0;
    Tb0 << 1, 0, 0, 0.1662,
        0, 1, 0, 0,
        0, 0, 1, 0.0026,
        0, 0, 0, 1;
    Macanum macanum(Tb0, l, w, r);
    state.update_state(speed, macanum);
    // calculated  traveled distance
    auto chassis = state.get_chassis_state();
    auto distance = sqrt(pow(chassis(1),2) +  pow(chassis(2),2));
    std::cout << "chassis state: " << chassis
              << "distance: " << distance
              << std::endl;
    ASSERT_TRUE(abs(distance-0.475) < 0.00001);
}
/*
TEST(KUKA, macanum_translation2) {
    //Eigen::Vector4d u(-10,10,-10,10);
    //TODO test sidesways in y by 0.475
    //define state
    State state(4, 5, 0.01);
    Eigen::VectorXd configuration(12);
    configuration << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    Eigen::VectorXd speed(9);
    speed << 0, 0, 0, 0, 0, -10, 10, -10, 10;
    state(configuration, speed);
    //define macanum
    const double l=0.235; // axial distance 2l=0.47
    const double w=0.15; // wheel to wheel 2w=0.3
    const double r=0.0475;
    Eigen::Matrix4d Tb0;
    Tb0 << 1, 0, 0, 0.1662,
        0, 1, 0, 0,
        0, 0, 1, 0.0026,
        0, 0, 0, 1;
    Macanum macanum(Tb0, l, w, r);
    state.update_state(speed, macanum);
    // calculated  traveled distance
    auto chassis = state.get_chassis_state();
    auto distance = sqrt(pow(chassis(1),2) +  pow(chassis(2),2));
    std::cout << "chassis state: " << chassis
              << "distance: " << distance
              << std::endl;
    ASSERT_TRUE(abs(distance-0.475) < 0.00001);
}
TEST(KUKA, macanum_translation2_clipped) {
    //Eigen::Vector4d u(-10,10,-10,10);
    //TODO test sidesways in y by 0.475
    //define state
    double clip=10;
    State state(4, 5, 0.01, clip);
    Eigen::VectorXd configuration(12);
    configuration << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    Eigen::VectorXd speed(9);
    speed << 0, 0, 0, 0, 0, -10, 10, -10, 10;
    state(configuration, speed);
    //define macanum
    const double l=0.235; // axial distance 2l=0.47
    const double w=0.15; // wheel to wheel 2w=0.3
    const double r=0.0475;
    Eigen::Matrix4d Tb0;
    Tb0 << 1, 0, 0, 0.1662,
        0, 1, 0, 0,
        0, 0, 1, 0.0026,
        0, 0, 0, 1;
    Macanum macanum(Tb0, l, w, r);
    state.update_state(speed, macanum);
    // calculated  traveled distance
    auto chassis = state.get_chassis_state();
    auto distance = sqrt(pow(chassis(1),2) +  pow(chassis(2),2));
    std::cout << "chassis state: " << chassis
              << "distance: " << distance
              << std::endl;
    ASSERT_TRUE(abs(distance-0.475/2) < 0.00001);
}

TEST(KUKA, macanum_rotation) {
    //Eigen::Vector4d u(-10,10,10,-10);
    //test spin counterclockwise in place by 1.234 radians.
    //define state
    State state(4, 5, 0.01);
    Eigen::VectorXd configuration(12);
    configuration << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    Eigen::VectorXd speed(9);
    speed << 0, 0, 0, 0, 0, -10, 10, 10, -10;
    state(configuration, speed);
    //define macanum
    const double l=0.235; // axial distance 2l=0.47
    const double w=0.15; // wheel to wheel 2w=0.3
    const double r=0.0475;
    Eigen::Matrix4d Tb0;
    Tb0 << 1, 0, 0, 0.1662,
        0, 1, 0, 0,
        0, 0, 1, 0.0026,
        0, 0, 0, 1;
    Macanum macanum(Tb0, l, w, r);
    state.update_state(speed, macanum);
    // calculated  traveled distance
    auto chassis = state.get_chassis_state();
    auto distance = sqrt(pow(chassis(1),2) +  pow(chassis(2),2));
    auto rotation = chassis(0);
    std::cout << "chassis state: " << chassis
              << "rotation: " << distance
              << "rotation: " << rotation
              << std::endl;
    ASSERT_TRUE(abs(distance-0) < 0.001);
    ASSERT_TRUE(abs(rotation-1.234) < 0.01);
}

TEST(KUKA, trajectoryGenerationScene8) {
    Eigen::Matrix4d Tb0;
    Tb0 << 1, 0, 0, 0.1662,
        0, 1, 0, 0,
        0, 0, 1, 0.0026,
        0, 0, 0, 1;
    Eigen::Matrix4d M0e;
    M0e << 1, 0, 0, 0.033,
        0, 1, 0, 0,
        0, 0, 1, 0.6546,
        0, 0, 0, 1;
    Eigen::MatrixXd Blist(6,5);
    Blist << 0,0,0,0,0,
        0,-1,-1,-1,0,
        1,0,0,0,1,
        0,0,0,0,0,
        0.033,-0.5076,-0.3526,-0.2176,0,
        0,0,0,0,0;
    Eigen::Matrix4d T_sc_initial;
    T_sc_initial << 1, 0, 0, 1,
        0, 1, 0, 0,
        0, 0, 1, 0.025,
        0, 0, 0, 1;
    Eigen::Matrix4d T_sc_d;
    T_sc_d << 0, 1, 0, 0,
        -1, 0, 0, -1,
        0, 0, 1, 0.025,
        0, 0, 0, 1;
    const double l=0.235; // axial distance 2l=0.47
    const double w=0.15; // wheel to wheel 2w=0.3
    const double r=0.0475;
    const double dt=0.01;
    BodyKinematics kinematics(Blist, M0e);
    Macanum macanum(Tb0, l, w, r);
    // simulation starts with initial block configuration at (x,y,theta) = (1,0,0)
// final block configuration (0,-1,-pi/2)
    State state(4, 5, dt);
    //Milestone 1
    // 1 second simulation
    // wheel control
    //Eigen::Vector4d u(10,10,10,10);
    //TODO set the configuration precisely
    Eigen::VectorXd configuration(12);
    configuration << 1, 0, 0,
        0, 0, 0, 0, 0,
        0, 0, 0, 0;
    Eigen::VectorXd speed(9);
    speed << 0, 0, 0, 0, 0,
        0, 0, 0, 0;
    state(configuration, speed);
    auto Tse = state.Tse(kinematics);
    double intervals[] = {10, 2, 0.63, 2, 10, 2, 0.63, 2};
    PayloadMissionPlanner trajectory(Tse, intervals);
    trajectory(T_sc_initial, T_sc_d);
    std::cout << "trajectory size: "
              << trajectory.size() << std::endl;
    //for (int i=0; trajectory.size(); i++) trajectory.print(i);
}

TEST(KUKA, Controller) {
    //Milestone 3
    Eigen::Matrix4d Xd;
    Xd << 0, 0, 1, 0.5,
        0, 1, 0, 0,
        -1, 0, 0, 0.5,
        0, 0, 0, 1;
    Eigen::Matrix4d Xd_next;
    Xd_next << 0, 0, 1, 0.6,
        0, 1, 0, 0,
        -1, 0, 0, 0.3,
        0, 0, 0, 1;
    Eigen::Matrix4d X;
    X << 0.170, 0, 0.985, 0.387,
        0, 1, 0, 0,
        -0.985, 0, 0.170, 0.570,
        0, 0, 0, 1;
    double Kp=0, Ki=0, dt=0.01;
    //
    Eigen::Matrix4d Tb0;
    Tb0 << 1, 0, 0, 0.1662,
        0, 1, 0, 0,
        0, 0, 1, 0.0026,
        0, 0, 0, 1;
    Eigen::Matrix4d M0e;
    M0e << 1, 0, 0, 0.033,
        0, 1, 0, 0,
        0, 0, 1, 0.6546,
        0, 0, 0, 1;
    Eigen::MatrixXd Blist(6,5);
    Blist << 0,0,0,0,0,
        0,-1,-1,-1,0,
        1,0,0,0,1,
        0,-0.5076,-0.3526,-0.2176,0,
        0.033,0,0,0,0,
        0,0,0,0,0;
    Eigen::Matrix4d T_sc_initial;
    T_sc_initial << 1, 0, 0, 1,
        0, 1, 0, 0,
        0, 0, 1, 0.025,
        0, 0, 0, 1;
    Eigen::Matrix4d T_sc_d;
    T_sc_d << 0, 1, 0, 0,
        -1, 0, 0, -1,
        0, 0, 1, 0.025,
        0, 0, 0, 1;
    const double l=0.235; // axial distance 2l=0.47
    const double w=0.15; // wheel to wheel 2w=0.3
    const double r=0.0475;
    BodyKinematics kinematics(Blist, M0e);
    Macanum macanum(Tb0, l, w, r);
    // simulation starts with initial block configuration at (x,y,theta) = (1,0,0)
    // final block configuration (0,-1,-pi/2)
    State state(4, 5, dt);
    Eigen::VectorXd configuration(12);
    configuration << 0, 0, 0,
        0, 0, 0.2, -1.6, 0,
        0, 0, 0, 0;
    Eigen::VectorXd speed(9);
    speed << 0, 0, 0, 0, 0,
        0, 0, 0, 0;
    state(configuration, speed);
    auto Tse = state.Tse(kinematics);
    double intervals[] = {10, 2, 0.63, 2, 10, 2, 0.63, 2};
    PayloadMissionPlanner trajectory(Tse, intervals, dt);
    trajectory(T_sc_initial, T_sc_d);
    ERROR_DEF fn = &Controller::diff_trans;
    PI_Controller controller(state, kinematics, fn, Kp, Ki, dt);
    Kuka kuka(kinematics, macanum, trajectory, state, controller);
    // TEST
    //milestone3 output
    Eigen::VectorXd Vd(6);
    Vd << 0,0,0,20,0,10;
    Eigen::VectorXd AdVd(6);
    AdVd << 0,0,0,21.409,0,6.455;
    Eigen::VectorXd V(6);
    V << 0,0,0,21.409,0,6.455;
    Eigen::VectorXd Xerr (6);
    Xerr << 0,0.171,0,0.080,0,0.107;
    Eigen::MatrixXd Je(6,9);
    Je << 0.03, -0.03, -0.03, 0.03, -0.985, 0, 0, 0, 0,
        0, 0, 0, 0, 0, -1, -1, -1, 0,
        -0.005, 0.005, 0.005, -0.005, 0.17, 0, 0, 0, 1,
        0.002, 0.002, 0.002, 0.002, 0, -0.24, -0.214, -0.218,  0,
        -0.024, 0.024, 0, 0, 0.221, 0, 0, 0, 0,
        0.012, 0.012, 0.012, 0.012, 0, -0.288, -0.135, 0, 0;
    Eigen::VectorXd _utheta(9);
    _utheta << 157.2, 157.2, 157.2, 157.2, 0, -652.9, 1398.6, -745.7, 0;
    //
    Eigen::MatrixXd _Vd = kuka.feedforward(Xd, Xd_next);
    std::cout << "_Vd: " << _Vd << std::endl
              << "Vd: " << Vd << std::endl;
    ASSERT_TRUE(_Vd.isApprox(Vd));
    Eigen::MatrixXd _AdVd = kuka.adj_feedforward(Xd, X, Xd_next);
    std::cout << "_AdVd: " << _AdVd << std::endl
              << "AdVd: " << AdVd << std::endl;
    ASSERT_TRUE(_AdVd.isApprox(AdVd, 5));
    Eigen::VectorXd Ve = controller(state, kinematics, AdVd, Xd);
    ASSERT_TRUE(Ve.isApprox(V));
    std::cout << "Ve: " << Ve << std::endl
              << "V: " << V << std::endl;
    auto _Je = kuka.get_Je();
    std::cout << "Je: " << Je << std::endl
              << "_Je: " << _Je << std::endl;
    ASSERT_TRUE(_Je.isApprox(Je,4));
    //update speed arm speed
    Eigen::VectorXd udtheta = kuka.Jeff_inv()*Ve;
    std::cout << "udtheta: " << udtheta << std::endl
              << "utheta: " << _utheta << std::endl;
    ASSERT_TRUE(udtheta.isApprox(_utheta,4));
}
*/
TEST(KUKA, missionPlanning) {
    Eigen::Matrix4d Tb0;
    Tb0 << 1, 0, 0, 0.1662,
        0, 1, 0, 0,
        0, 0, 1, 0.0026,
        0, 0, 0, 1;
    Eigen::Matrix4d M0e;
    M0e << 1, 0, 0, 0.033,
        0, 1, 0, 0,
        0, 0, 1, 0.6546,
        0, 0, 0, 1;
    Eigen::MatrixXd Blist(6,5);
    Blist << 0,0,0,0,0,
        0,-1,-1,-1,0,
        1,0,0,0,1,
        0,0,0,0,0,
        0.033,-0.5076,-0.3526,-0.2176,0,
        0,0,0,0,0;
    Eigen::Matrix4d T_sc_initial;
    T_sc_initial << 1, 0, 0, 1,
        0, 1, 0, 0,
        0, 0, 1, 0.025,
        0, 0, 0, 1;
    Eigen::Matrix4d T_sc_d;
    T_sc_d << 0, 1, 0, 0,
        -1, 0, 0, -1,
        0, 0, 1, 0.025,
        0, 0, 0, 1;
    const double l=0.235; // axial distance 2l=0.47
    const double w=0.15; // wheel to wheel 2w=0.3
    const double r=0.0475;
    const double dt=0.01;
    BodyKinematics kinematics(Blist, M0e);
    Macanum macanum(Tb0, l, w, r);
    // simulation starts with initial block configuration at (x,y,theta) = (1,0,0)
    // final block configuration (0,-1,-pi/2)
    State state(4, 5, dt);
    //TODO set the configuration precisely
    Eigen::VectorXd configuration(12);
    configuration << M_PI/4, -0.3, 0.2,
        0, 0, 0.2, -1.6, 0,
        0,0,0,0;
    Eigen::VectorXd speed(9);
    speed << 0, 0, 0, 0, 0,
        0, 0 ,0 ,0;
    state(configuration, speed);
    auto Tse = state.Tse(kinematics);
    double intervals[] = {10, 2, 0.63, 2, 10, 2, 0.63, 2};
    PayloadMissionPlanner trajectory(Tse, intervals);
    trajectory(T_sc_initial, T_sc_d);
    PI_Controller controller(state, kinematics, Controller::diff_trans, 1, 1, dt);
    //PI_Controller controller(state, kinematics, Controller::diff_trans, 0, 0, dt);
    Kuka kuka(kinematics, macanum, trajectory, state, controller);
    kuka();
}

int main(int argc, char **args) {
    InitGoogleTest(&argc, args);
    return RUN_ALL_TESTS();
}
