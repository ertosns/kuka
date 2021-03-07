#pragma once
#include <Eigen/Dense>
#include <Eigen/QR>
#include <Eigen/LU>
#include "kinetics/kinematics.hpp"
#include <cmath>
#include "../include/vehicle.hpp"
#include "kinetics/logger.hpp"

using namespace std;

class Vehicle : public Logger {
public:
    /* Vehicle constructor
     *
     * @param Tb0: transformation matrix from {0} frame (attached robot-arm) to body frame
     * @param F: odometry matrix,inverse of H(0)
     */
    Vehicle(){
        /*empty constructor for preparation of H,F*/
    }
    /*
    Vehicle(Eigen::MatrixXd H,
            Eigen::MatrixXd F,
            Eigen::MatrixXd Tb0) :
        H(H), F(F), Tb0(Tb0) {
        assert(Tb0.size()>0);
    }
    Vehicle (const Vehicle &V) : Tb0(V.Tb0), F(V.F), H(V.H) {
    assert(V.Tb0.size()>0);
    }
    */
    Vehicle(Eigen::MatrixXd F,
            Eigen::MatrixXd _Tb0) :
        F(F), Tb0(_Tb0) {
        assert(Tb0.size()>0);
    }
    Vehicle (const Vehicle &V) :
        Tb0(V.Tb0), F(V.F) {
        assert(V.Tb0.size()>0);
    }
    bool initialized() {
        return F.size()>0;
    }
    /* calculate odometry of the vehicle
     *
     * consult Modern robotics equation 13.35 for more details
     * @param phi: phase between body, and space frames in radians
     * @return vehicle configuration state q
     */
    Eigen::MatrixXd odometry(Eigen::VectorXd du,
                             double phi) {
        Eigen::VectorXd Vb =  F*du;
        auto wz = Vb(2);
        auto vx = Vb(3);
        auto vy = Vb(4);
        if (Algebra::Epsilon(wz)) wz=0;
        Eigen::Vector3d dq_b;
        if (wz==0)
            dq_b << 0, vx, vy;
        else
            dq_b << wz,
                (vx*sin(wz) + vy*(cos(wz)-1))/wz,
                (vy*sin(wz) + vx*(1-cos(wz)))/wz;
        Eigen::Matrix3d Rb(3,3);
        Rb << 1, 0, 0,
            0, cos(phi), -1*sin(phi),
            0, sin(phi), cos(phi);
        auto dq = Rb*dq_b;
        //std::cout << "Vb: " << Vb << std::endl << "dq_b: " << dq_b << std::endl << "dq: " << dq << std::endl;
        return dq;
    }
    /* Jacobian of base vehicle, or the effect of the vehicle wheels in the  the open-arm Twist.
     *
     * @return Jacobian
     */
    Eigen::MatrixXd Jbase(Kinematics &kinematics,
                          Eigen::VectorXd thetalist) {
        assert(Tb0.size()>0);
        Eigen::MatrixXd T0e = kinematics.ForwardKin(thetalist);
        Eigen::MatrixXd lie_adjoint_eb =
            Algebra::Adjoint(Algebra::TransInv(T0e)*
                             Algebra::TransInv(Tb0));
        return lie_adjoint_eb*F;
    }
    Eigen::MatrixXd get_Tb0() {
        return Tb0;
    }
protected:
    Eigen::MatrixXd Tb0;
    //Eigen::MatrixXd H;
    Eigen::MatrixXd F;
};

/////////////////////
// Macanum Vehicle //
/////////////////////
class Macanum : public Vehicle {
public:
    /* Macanum Vehicle constructor
     *
     * @param l
     * @param w
     * @param r
     * @param u: Macanum vehicle omni-wheel control input speed
     */
    Macanum(Eigen::MatrixXd T_b0,
            double l, double w, double r) :
        l(l), w(w), r(r) {
        assert(T_b0.size()>0);
        //Eigen::MatrixXd H_(4,3);
        //NOTE! F  is pseudo inverse of H see section 13.4.
        //consult Modern robotics equation 13.10
        /*H_ << -l-w, 1, -1,
            l+w, 1, 1,
            l+w, 1, -1,
            -l-w, 1, 1;
        H = 1/r * H_;
        */
        Eigen::MatrixXd F_(6,4);
        F_ << 0, 0, 0, 0,
            0, 0, 0, 0,
            -1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w),
            1, 1, 1, 1,
            -1, 1, -1, 1,
            0, 0, 0, 0;
        F = r/4 * F_;
        Tb0=T_b0;
        //Vehicle(H,F, Tb0);
        Vehicle(F, Tb0);
    }
protected:
    const double l;
    const double w;
    const double r;
};

class ThreeOmniwheel : public Vehicle {
public:
    ThreeOmniwheel(Eigen::MatrixXd T_b0,
                    double d, double r) :
        d(d), r(r) {
        Tb0=T_b0;
        /*
        Eigen::MatrixXd H(3,3);
        H << -1*d, 1, 0,
            -1*d, -1/2, -1*sin(M_PI/3),
            -1*d, -1/2, sin(M_PI/3);
        H = 1/r * H;
        */
        Eigen::MatrixXd F(6,3);
        F << 0, 0, 0,
            0, 0, 0,
            -1/(3*d), -1/(3*d), -1/(3*d),
            2/3, -1/3, -1/3,
            0, -1/(2*sin(M_PI/3)), 1/(2*sin(M_PI/3)),
            0, 0, 0;
        F = r*F;
        //Vehicle(H, F, Tb0);
        Vehicle(F, Tb0);
    }
protected:
    const double d;
    const double r;
};

class State {
public:
    /* zero State constructor
     *
     * state space or robot with 3 body configuration states, chassis phi, chassis x, chassis y, and open arm joints, and vehicle wheel joints.
     *
     * @param nwheels: number of wheels
     * @param narm_joints: number of open arm joints
     */
    State(int nwheels, int narm_joints,
          double _dt, double _clip=INFINITY) :
        nJ(narm_joints), nW(nwheels), dt(_dt), clip(_clip),
        state(Eigen::VectorXd::Zero(3+nW+nJ)),
        speed(Eigen::VectorXd::Zero(nW+nJ)),
        log ("coppeliasim.csv" ) {
        //std::cout << "state constructed!" << std::endl;
    }
    State(const State &copy) : nJ(copy.nJ),
                               nW(copy.nW),
                               state(copy.state),
                               speed(copy.speed),
                               dt(copy.dt),
                               clip(copy.clip),
                               log(copy.log) {
        //std::cout << "state copy constructed" << std::endl;
    }
    void operator() (Eigen::VectorXd _state,
                     Eigen::VectorXd _speed) {
        state=_state;
        speed=_speed;
        clip_speed();
    }
    void clip_speed() {
        // clip speed values (size=9)
        for (int i=0; i<speed.size(); i++) {
            speed(i) = std::min(clip, speed(i));
            speed(i) = std::max(-1*clip, speed(i));
        }
    }
    Eigen::MatrixXd Tsb() {
        //body to space
        Eigen::MatrixXd chassis=get_chassis_state();
        double phi=chassis(0);
        double x = chassis(1);
        double y = chassis(2);
        Eigen::Matrix4d Tsb;
        Tsb << cos(phi), -1*sin(phi), 0, x,
            sin(phi), cos(phi), 0, y,
            0, 0, 1, 0.0963,
            0, 0, 0, 1;
        return Tsb;
    }
    /* get actual current end-effector configuration
     * estimated from actuators sensors.
     *
     */
    Eigen::MatrixXd Tse(Kinematics &kinematics, Vehicle &vehicle) {
        Eigen::VectorXd thetalist = get_arm_state();
        Eigen::MatrixXd T0e = kinematics.ForwardKin(thetalist);
        Eigen::MatrixXd Tb0 = vehicle.get_Tb0();
        return Tsb()*Tb0*T0e;
    }
    /////////////
    // chassis //
    /////////////
    Eigen::VectorXd get_chassis_state() {
        //return state.block<3, 1>(0,0);
        return state.block(0,0,3,1);
    }
    /////////
    // arm //
    /////////
    void set_chassis_state(Eigen::VectorXd q_chassis) {
        q_chassis(0) = trim(q_chassis(0));
        state.block(0,0,3,1) << q_chassis;
    }
    void set_arm_state(Eigen::VectorXd q_arm) {
        assert(q_arm.size()==nJ);
        //set arm joints 3,4 to limit 0.2 to avoid self-collision
         //arm limits
        if (q_arm(2)>0.2)
            q_arm(2) = std::min(0.2, q_arm(2));
        else if (q_arm(2)<-0.2)
            q_arm(2) = std::max(-0.2, q_arm(2));
        //
        if (q_arm(3)>0.2)
            q_arm(3) = std::min(0.2, q_arm(3));
        else if (q_arm(3)<-0.2)
            q_arm(3) = std::max(-0.2, q_arm(3));

        modulas(q_arm);
        state.block(3,0,nJ,1) << q_arm;
    }
    Eigen::VectorXd get_arm_state() {
        //std::cout << "sample value: " << state(3,0) << std::endl;
        return state.block(3, 0, nJ, 1);
    }
    void set_arm_dstate(Eigen::VectorXd dq_arm) {
        assert(dq_arm.size()==nJ);
        speed.block(0,0,nJ,1) << dq_arm;
        clip_speed();
    }
    Eigen::VectorXd get_arm_dstate() {
        return speed.block(0, 0, nJ, 1);
    }
    //////////
    // base //
    //////////
    void set_base_state(Eigen::VectorXd q_base) {
        assert(q_base.size()==nW);
        modulas(q_base);
        state.block(3+nJ,0,nW,1) << q_base;
    }
    Eigen::VectorXd get_base_state() {
        return state.block(3+nJ, 0, nW, 1);
    }
    void set_base_dstate(Eigen::VectorXd dq_base) {
        assert(dq_base.size()==nW);
        speed.block(nJ,0,nW,1) << dq_base;
        clip_speed();
    }
    Eigen::VectorXd get_base_dstate() {
        return speed.block(nJ, 0, nW, 1);
    }
    /*
    Eigen::VectorXd get_delta_wheels() {
        return dt*get_base_dstate();
    }
    */
    // state_space
    void update_state(Eigen::VectorXd U,
                      Vehicle &vehicle) {
        auto wheel_speed = U.block(0,0,4,1);
        auto arm_speed = U.block(4,0,5,1);
        //  update speed
        set_arm_dstate(arm_speed);
        set_base_dstate(wheel_speed);
        // set wheel, and call themback in order to get clipped!
        wheel_speed=get_base_dstate();
        arm_speed=get_arm_dstate();
        auto arm_state = get_arm_state();
        auto wheel_state = get_base_state();
        //TODO (res) doesn't pass the test
        auto wheel_delta = wheel_speed*dt;
        auto arm_delta = arm_speed*dt;
        //TODO (res) passes the test, although not correct!
        //auto wheel_delta = wheel_speed;
        //auto arm_delta = arm_speed;
        // update robot configuration
        Eigen::MatrixXd arm_next = arm_state + arm_delta;
        Eigen::MatrixXd base_next = wheel_state + wheel_delta;
        set_arm_state(arm_next);
        set_base_state(base_next);
        // update chassis configuration
        Eigen::MatrixXd chassis = get_chassis_state();
        double phi = chassis(0);
        Eigen::VectorXd du = wheel_delta;
        Eigen::MatrixXd dq = vehicle.odometry(du, phi);
        Eigen::MatrixXd chassis_next = chassis + dq;
        //std::cout << "chassis: " << chassis << std::endl << "chassis_next: " << chassis_next << std::endl << "du: " << du << std::endl << "dq: " << dq << std::endl;
        set_chassis_state(chassis_next);
    }
    /* save state vector
     *
     * @param state: chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4
     */
    void save_state(bool gripper_state) {
        //TODO edit logger, and pass state, and gripper state at once.
        vector<double> S;
        for (int i=0; i < 12; i++) S.push_back(state(i));
        S.push_back(gripper_state?1:0);
        for (int i=0; i < S.size(); i++) {
            if (i==S.size()-1) {
                std::cout << S[i] << std::endl;
                log.write(S[i], true);
            }
            else {
                std::cout << S[i] << ",";
                log.write(S[i], false);
            }
        }
        log.flush();
        //TODO save/publish/write state
        //TODO write coppeliasim API
    }
private:
    const int nJ;
    const int nW;
    Eigen::VectorXd state;
    Eigen::VectorXd speed;
    const double dt;
    const double clip;
    Logger log;
    static double trim(double v) {
        double CIRCLE = 2*3.14;
        if (v >= CIRCLE) {
            //v -= CIRCLE;
            v =fmod(v,CIRCLE);
        }
        if (v <= -1*CIRCLE) {
            //v += CIRCLE;
            v = fmod(v,CIRCLE);
        }
        return v;
    }
    /* Trim V values by 360 degree, V expected to be in radians
     *
     * @param V: vector for joint in radians
     * @return V: trimmed vector
     */
    static Eigen::VectorXd modulas(Eigen::VectorXd &V) {
        for (int i=0; i<V.size(); i++) {
            V(i) = trim(V(i));
        }
        return V;
    }
};
