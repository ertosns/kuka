#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include "gmock/gmock.h"
#include "../include/kuka.hpp"

using namespace testing;
using namespace std;

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
    Macanum macanum(l, w, r);
    assert(macanum.initialized());
    // simulation starts with initial block configuration at (x,y,theta) = (1,0,0)
// final block configuration (0,-1,-pi/2)
    State state(4, 5, dt);
    //Milestone 1
    // 1 second simulation
    // wheel control
    //Eigen::Vector4d u(10,10,10,10);
    Eigen::VectorXd configuration(12);
    configuration << 1, 0, 0,
        0, 0, 0, 0, 0,
        0, 0, 0, 0;
    Eigen::VectorXd speed(9);
    speed << 0, 0, 0, 0, 0,
        10, 10, 10, 10;
    state(configuration, speed);
    //TODO test forward movement by 0.475
    //Eigen::Vector4d u(-10,10,-10,10);
    //TODO test sidesways in y by 0.475
    //Eigen::Vector4d u(-10,10,10,-10);
    //TODO test spin counterclockwise in place by 1.234 radians.
    //TODO try cipping speed to 5, and distance should be have this distance
    //TODO add missionplanner
    auto Tse = state.Tse(kinematics);
    //PayloadMissionPlanner trajectory(Tse);
    //trajectory(T_sc_initial, T_sc_d);
    InPlaceMissionPlanner trajectory(Tse);
    trajectory(3);
    std::cout << "trajector size: "
              << trajectory.size() << std::endl;
    std::cout << "target size: "
              << trajectory.target().size() << std::endl;
    Kuka kuka(kinematics, macanum, trajectory, state, Tb0);
}
/*
TEST(KUKA, milestone2) {
    //Milestone 2
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
    //output
    Eigen::VectorXd Vd(0,0,0,20,0,10);
    Eigen::VectorXd AdVd(0,0,0,21.409,0,6.455);
    Eigen::VectorXd V(0,0,0,21.409,0,6.455);
    Eigen::VectorXd Xerr(0,0.171,0,0.080,0,0.107);
    Eigen::MatrixXd Je(6,9);
    << 0.03, -0.03, -0.03, 0.03, -0.985, 0, 0, 0, 0,
           0, 0, 0, 0, 0, -1, -1, -1, 0,
           -0.005, 0.005, 0.005, -0.005, 0.17, 0, 0, 0, 1,
           0.002, 0.002, 0.002, 0.002, 0, -0.24, -0.214, -0.218,  0,
           -0.024, 0.024, 0, 0, 0.221, 0, 0, 0, 0,
           0.012, 0.012, 0.012, 0.012, 0, -0.288, -0.135, 0, 0;
    Eigen::VectorXd utheta(157.2, 157.2, 157.2, 157.2, 0, -652.9, 1398.6, -745.7, 0);
}
*/
int main(int argc, char **args) {
    InitGoogleTest(&argc, args);
    return RUN_ALL_TESTS();
}
