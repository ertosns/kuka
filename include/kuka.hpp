#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/QR>
#include "kinetics/trajectory.hpp"
#include "kinetics/kinematics.hpp"
#include "algebra/algebra.hpp"
#include "../include/vehicle.hpp"
#include "../include/utils.hpp"
#include "../include/mission_planner.hpp"

using namespace std;

//TODO create general mission planning api
// create vehicle mission planning api
//TODO Implement controller

class Kuka {
public:
    /* Kuka constructor
     *
     * @param kinematics: open-arm kinematics
     * @param Vehicle: macanum vehicle
     * @param state: configuration state of the robot
     * @param Tb0: transformation matrix from base of the arm to the body frame
     * TOOD move Kp, Ki, dt to the controller
     */
    Kuka(Kinematics kinematics,
         Vehicle macanum,
         MissionPlanner trajectory,
         State state, Eigen::MatrixXd Tb0,
         double Kp=0.1, double Ki=0.1, double dt=0.01)
        : kinematics(kinematics),
          macanum(macanum),
          trajectory(trajectory),
          Tb0(Tb0),
          state(state),
          eint(Eigen::VectorXd::Zero(6)),
          Kp(Kp), Ki(Ki), dt(dt) {
        Eigen::MatrixXd cur, next;
        Eigen::VectorXd Ve;
        auto T_d = trajectory.target();
        for (int i=0; i<trajectory.size()-1; i++) {
            cur= trajectory.get(i);
            next=trajectory.get(i+1);
            Ve = controlled_Twist(cur, T_d, next);
            cur = next;
            //TODO do i need to update the speed state?
            //speed arm speed
            Eigen::VectorXd udtheta = Jeff_inv()*Ve;
            auto wheel_dstate = udtheta.block(0,0,4,1);
            auto arm_dstate = udtheta.block(4,0,5,1);
            state.set_arm_dstate(arm_dstate);
            state.set_base_dstate(wheel_dstate);
            //update state
            state.update_state(macanum);
            state.save_state(trajectory.gripper_state(i));
        }
    }
private:
    Kinematics kinematics;
    Vehicle macanum;
    MissionPlanner trajectory;
    State state;
    Eigen::MatrixXd Tb0;
    Eigen::VectorXd eint;
    double Ki, Kp, dt;

    /* inverse of the kuka Jacobian or the sensitivity of end effector to vehicle, and arm
     *
     *@return Jacobian pseudo inverse.
     */
    Eigen::MatrixXd Jeff_inv() {
        // T0e
        Eigen::VectorXd thetalist = state.get_arm_state();
        Eigen::MatrixXd Tbe = kinematics.ForwardKin(thetalist);
        Eigen::MatrixXd T0e = Algebra::TransInv(Tb0)*Tbe;
        //
        auto T_0e = T0e;
        //input of vehicle wheels to end effector
        auto jbase = macanum.Jbase(T_0e, Tb0); //6X4
        //input of arm joints to end effector
        auto jarm = kinematics.Jacobian(thetalist);//6X5
        Eigen::MatrixXd Jeff(6,9);
        Jeff << jbase, jarm;
        //return 9X6 jacobian matrix, the pseudo inverse of the J_end_effector
        return Jeff.completeOrthogonalDecomposition().pseudoInverse();
    }
    /*
     * PI controller Twist.
     *
     * @param T_se: current transformation matrix of end-effector {e} relative to space frame at t=current_time.
     * @param T_se_d: end-effector {e} relative to space frame {s} at the destination/target time t=T(total time).
     * @param T_se_d_next: following transformation after dt time step.
     * @param Kp: proportional gain in PI controller.
     * @param Ki: integral gain in PI controller.
     * @return V: current screw state.
     */
    Eigen::MatrixXd controlled_Twist(Eigen::MatrixXd T_se,
                                     Eigen::MatrixXd T_se_d,
                                     Eigen::MatrixXd T_se_d_next) {
        Eigen::MatrixXd X = T_se;
        Eigen::MatrixXd Xd = T_se_d;
        Eigen::MatrixXd Xd_next = T_se_d_next;
        // Xerr is the same as the screw state before controller
        // meaning Xerr=Vs
        //TODO kinematics object should be able to subtract two transition states from each others, move this to either algebra, or kinetics library (in kinematics file)
        Eigen::VectorXd Xerr = Algebra::se3ToVec(Algebra::MatrixLog6(Algebra::TransInv(X)*Xd));
        int nrows = Xerr.rows();
        //integral error
        eint +=Xerr*dt;
        //TODO why 1/dt0
        Eigen::MatrixXd KP = Kp*Eigen::MatrixXd::Identity(nrows, nrows);
        Eigen::MatrixXd KI = Ki*Eigen::MatrixXd::Identity(nrows, nrows);
        Eigen::VectorXd  Vd = 1/dt * Algebra::se3ToVec(Algebra::MatrixLog6(Algebra::TransInv(Xd)*Xd_next));
        Eigen::VectorXd Ve = Algebra::Adjoint(Algebra::TransInv(X)*Xd)*Vd +KP*Xerr + KI*(eint+Xerr);
        return Ve;
    }

};
