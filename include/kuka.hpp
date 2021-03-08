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
#include "../include/controller.hpp"

using namespace std;

class Kuka {
public:
    /* Kuka constructor
     *
     * @param kinematics: open-arm kinematics
     * @param Vehicle: macanum vehicle
     * @param state: configuration state of the robot
     * @param Tb0: transformation matrix from base of the arm to the body frame
     */
    Kuka(Kinematics kinematics,
         Vehicle macanum,
         MissionPlanner trajectory,
         State state,
         Controller controller)
        : kinematics(kinematics),
          macanum(macanum),
          trajectory(trajectory),
          state(state),
          controller(controller) {
    }
    void operator()() {
        Eigen::MatrixXd Xd, X, Xd_next;
        Eigen::VectorXd Ve;
        Eigen::MatrixXd Vd;
        auto dt= trajectory.time_step();
        Xd = trajectory.get(0);
        for (int i=1; i<trajectory.size()-1; i++) {
            X=state.Tse(kinematics, macanum);
            Xd=trajectory.get(i);
            Xd_next=trajectory.get(i+1);
            Vd = adj_feedforward(Xd, X, Xd_next);
            //Ve = Vd;
            //TODO (fix) the controller oscillates, and multiply the error! first fix the twist error
            Ve = controller(state, kinematics, macanum, Vd, Xd);
            //update speed arm speed
            //TODO reverse Jeff to conside with the speed vector
            Eigen::VectorXd udtheta = Jeff_inv()*Ve;
            //update state
            state.update_state(udtheta, macanum);
            state.save_state(trajectory.gripper_state(i));
        }
    }
    //TODO (adhock) for testing purpose.
    //private:
    Kinematics kinematics;
    Vehicle macanum;
    MissionPlanner trajectory;
    State state;
    Controller controller;
    /*feedforward without Adjoint, used for testing purpose
     *
     */
    Eigen::MatrixXd feedforward(Eigen::MatrixXd Xd,
                                Eigen::MatrixXd Xd_next) {
        //Xd, and dXd_next, for this reason 1/dt
        double dt_inv=1/trajectory.time_step();
        Eigen::MatrixXd Vd = dt_inv*kinematics.diff(Xd,Xd_next);
        return Vd;
    }
    Eigen::MatrixXd adj_feedforward(Eigen::MatrixXd Xd,
                                    Eigen::MatrixXd X,
                                    Eigen::MatrixXd Xd_next) {
        double dt=trajectory.time_step();
        //feedforward
        Eigen::MatrixXd Vd = Algebra::se3ToVec(Algebra::MatrixLog6(Algebra::TransInv(Xd)*Xd_next)/dt);
        //adjoint
        Eigen::MatrixXd diff=Algebra::TransInv(X)*Xd;
        Eigen::MatrixXd Adj = Algebra::Adjoint(diff);
        return Adj*Vd;
    }
    Eigen::MatrixXd get_Je() {
        Eigen::VectorXd thetalist = state.get_arm_state();
        //input of vehicle wheels to end-effector
        //6X4
        auto jbase = macanum.Jbase(kinematics, thetalist);
        // arm joints Twist effect on the end effector
        auto jarm = kinematics.Jacobian(thetalist);//6X5
        Eigen::MatrixXd Jeff(6,9);
        Jeff << jbase, jarm;
        return Jeff;
    }
    /* inverse of the kuka Jacobian or the sensitivity of end effector to vehicle, and arm
     *
     *@return Jacobian pseudo inverse.
     */
    Eigen::MatrixXd Jeff_inv() {
        Eigen::MatrixXd Jeff = get_Je();
        //return 9X6 jacobian matrix, the pseudo inverse of the J_end_effector
        return Jeff.completeOrthogonalDecomposition().pseudoInverse();
    }
};
