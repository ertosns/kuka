#pragma once
#include <Eigen/Dense>
#include <functional>
#include "algebra/algebra.hpp"
#include "../include/vehicle.hpp"
#include "kinetics/logger.hpp"

using namespace std;
using namespace Eigen;

#define ERROR_DEF function<MatrixXd(MatrixXd,MatrixXd)>

class Controller {
public:
    /* Controller Constructor
     *
     * @param state: State of current actual configuration
     * @param error_fn: takes two parameters first being desired configuration, and second is target configurations
     * @param Kp: proportional factor
     * @param Ki: integration factor
     * @param Kd: derivative factor
     */
    Controller(State state,
               ERROR_DEF &fn,
               double Kp=0, double Ki=0, double Kd=0, double _dt=0.01) :
        Kp(Kp), Ki(Ki), Kd(Kd), dt(_dt),
        log("controller.err") {
        error_fn=fn;
        Eigen::MatrixXd e = Eigen::VectorXd::Zero(6);
        int nrows=6;
        KP = Kp*Eigen::MatrixXd::Identity(nrows, nrows);
        KI = Ki*Eigen::MatrixXd::Identity(nrows, nrows);
        KD = Ki*Eigen::MatrixXd::Identity(nrows, nrows);
        eint = e;
        std::cout << "controller constructed!" << std::endl;
    }
    Controller(const Controller &copy)
        : eint(copy.eint),
          Kp(copy.Kp), Ki(copy.Ki), Kd(copy.Kd), dt(copy.dt),
          KP(copy.KP), KI(copy.KI), KD(copy.KD),
          log(copy.log) {
        error_fn=copy.error_fn;
        std::cout << "controller constructed!" << std::endl;
    }
    virtual Eigen::MatrixXd
    operator() (State &state,
                Kinematics &kinematics,
                Vehicle &vehicle,
                Eigen::MatrixXd V_feedforward,
                Eigen::MatrixXd reference,
                Eigen::MatrixXd dconfig=Eigen::MatrixXd::Zero(0,0)) {
        Eigen::MatrixXd actual = state.Tse(kinematics, vehicle);
        Eigen::MatrixXd e = error_fn(actual, reference);
        //  std::cout << "e: " << std::endl
        //<< e << std::endl;
        log.write(e);
        eint += e*dt;
        //Eigen::MatrixXd d = Eigen::MatrixXd::Zero(e.rows(), e.cols());
        //TODO PID controller
        //if (Kd!=0 && dd.size()>0 && dconfig.size()>0)
        //  d = error_fn(dd, dconfig);
        //KP * e + KI * (eint + e) + KD * d;
        //if e is vector then no difference between Ki, and KI!
        //Eigen::MatrixXd V = V_feedforward + KP * e + KI * eint + KD * d;
        Eigen::MatrixXd V = V_feedforward +
            KP * e +
            KI * eint;
        //+KD * d;
        return V;
    }
    static Eigen::MatrixXd diff_std (Eigen::MatrixXd X,
                                     Eigen::MatrixXd Xd) {
        assert(Xd.cols()==X.cols());
        assert(Xd.rows()==X.rows());
        return Xd - X;
    };
    static Eigen::MatrixXd diff_trans (Eigen::MatrixXd X,
                                       Eigen::MatrixXd Xd) {
        assert(Xd.cols()==X.cols() && X.size()==16);
        assert(Xd.rows()==X.rows() && Xd.size()==16);
        return Algebra::se3ToVec(Algebra::MatrixLog6(Algebra::TransInv(X)*Xd));
    };
protected:
    ERROR_DEF error_fn;
    Eigen::MatrixXd eint; //integral sum
    const double Kp;
    const double Ki;
    const double Kd;
    const double dt;
    Eigen::MatrixXd KP;
    Eigen::MatrixXd KI;
    Eigen::MatrixXd KD;
    Logger log;
};

class P_Controller : public Controller {
public:
    P_Controller(State state,
                 ERROR_DEF fn,
                 double Kp=0, double dt=0.01)  :
        Controller(state,
                   fn,
                   Kp, 0, 0, dt) {
        //
    }
    Eigen::MatrixXd
    operator()(State state,
               Kinematics kinematics,
               Vehicle vehicle,
               Eigen::MatrixXd feedforward,
               Eigen::MatrixXd config) {
        return static_cast<Controller&>(*this)(state,
                                               kinematics,
                                               vehicle,
                                               feedforward,
                                               config,
                                               Eigen::MatrixXd::Zero(0,0));
    }
};

class PI_Controller : public Controller {
public:
    PI_Controller(State state,
                  ERROR_DEF fn,
                  double Kp=0, double Ki=0, double dt=0.01) :
        Controller(state,
                   fn,
                   Kp, Ki, 0, dt) {
    }
    Eigen::MatrixXd
    operator()(State state,
               Kinematics kinematics,
               Vehicle vehicle,
               Eigen::MatrixXd feedforward,
               Eigen::MatrixXd config) {
        return static_cast<Controller&>(*this)(state,
                                               kinematics,
                                               vehicle,
                                               feedforward,
                                               config,
                                               Eigen::MatrixXd::Zero(0,0));
    }
private:
};

typedef Controller PID_Controller;
