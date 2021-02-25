#include <Eigen/Dense>
#include <Eigen/QR>
#include <Eigen/LU>
#include "kinetics/kinematics.hpp"
#include <cmath>

using namespace std;

class Vehicle {
public:
    /* Vehicle constructor
     *
     * @param H: ???
     * @param F: ???
     */
    Vehicle() {
        /**/
    }
    Vehicle(Eigen::MatrixXd H, Eigen::MatrixXd F) :
        H(H), F(F) {
        /**/
    }
    Vehicle (const Vehicle &V) : F(V.F), H(V.H){
    }
    bool initialized() {
        return F.size()>0 &&  H.size()>0;
    }
    /* calculate odometry of the vehicle
     *
     * @param phi: phase between body, and space frames in radians
     * @return vehicle configuration state q
     */
    Eigen::MatrixXd odometry(Eigen::VectorXd u,
                             double phi=0) {
        Eigen::MatrixXd Hinv =
            H.completeOrthogonalDecomposition().
            pseudoInverse();
        Eigen::Vector3d V3 =  Hinv*u;
        auto wz = V3(0);
        auto vx = V3(1);
        auto vy = V3(2);
        //consult Modern robotics equation 13.35
        Eigen::Vector3d dq_b;
        dq_b << wz,
            (vx*sin(wz) + vy*(cos(wz)-1))/wz,
            (vy*sin(wz) + vx*(1-cos(wz)))/wz;
        Eigen::Matrix3d Rb(3,3);
        Rb << 1, 0, 0,
            0, cos(phi), -1*sin(phi),
            0, sin(phi), cos(phi);
        auto dq = Rb*dq_b;
        return dq;
    }
    /* Jacobian of base vehicle, or the input of vehicle wheel the open-arm.
     *
     * @return Jacobian
     */
    Eigen::MatrixXd Jbase(Eigen::MatrixXd T_0e,
                          Eigen::MatrixXd T_b0) {
        Eigen::MatrixXd lie_adjoint_eb =
            Algebra::Adjoint(Algebra::TransInv(T_0e)*
                             Algebra::TransInv(T_b0));
        return lie_adjoint_eb*F;
    }
protected:
    Eigen::MatrixXd H;
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
    Macanum(double l, double w, double r) :
        l(l), w(w), r(r) {
        Eigen::MatrixXd H_(4,3);
        //NOTE! F  is pseudo inverse of H see section 13.4.
        //consult Modern robotics equation 13.10
        H_ << -l-w, 1, -1,
            l+w, 1, 1,
            l+w, 1, -1,
            -l-w, 1, 1;
        H = 1/r * H_;
        Eigen::MatrixXd F_(6,4);
        F_ << 0, 0, 0, 0,
            0, 0, 0, 0,
            -1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w),
            1, 1, 1, 1,
            -1, 1, -1, 1,
            0, 0, 0, 0;
        F = r/4 * F_;
    }
protected:
    const double l;
    const double w;
    const double r;
};

class ThreeOmniwheel : public Vehicle {
public:
    ThreeOmniwheel (double d, double r) : d(d), r(r) {
        Eigen::MatrixXd H(3,3);
        H << -1*d, 1, 0,
            -1*d, -1/2, -1*sin(M_PI/3),
            -1*d, -1/2, sin(M_PI/3);
        H = 1/r * H;
        Eigen::MatrixXd F(6,3);
        F << 0, 0, 0,
            0, 0, 0,
            -1/(3*d), -1/(3*d), -1/(3*d),
            2/3, -1/3, -1/3,
            0, -1/(2*sin(M_PI/3)), 1/(2*sin(M_PI/3)),
            0, 0, 0;
        F = r*F;
        Vehicle(H, F);
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
          double dt, double dlip=INFINITY) :
        nJ(narm_joints), nW(nwheels), dt(dt), clip(clip),
        state(Eigen::VectorXd::Zero(3+nwheels+narm_joints)),
        speed(Eigen::VectorXd::Zero(nwheels+narm_joints)){}
    void operator() (Eigen::VectorXd state,
                     Eigen::VectorXd speed) {
        state=state;
        speed=speed;
        // clip speed values (size=9)
        for (int i=0; i<speed.size(); i++) {
            speed(i) = std::min(clip, speed(i));
            speed(i) = std::max(-1*clip, speed(i));
        }
    }
    Eigen::MatrixXd Tsb() {
        //body to space
        auto chassis=get_chassis_state();
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
    Eigen::MatrixXd Tse(Kinematics &kinematics) {
        Eigen::VectorXd thetalist = get_arm_state();
        auto Tbe = kinematics.ForwardKin(thetalist);
        return Tsb()*Tbe;
    }
    /////////////
    // chassis //
    /////////////
    void set_chassis_state(Eigen::VectorXd q_chassis) {
        state.block(0,0,3,1) << q_chassis;
    }
    Eigen::VectorXd get_chassis_state() {
        //return state.block<3, 1>(0,0);
        return state.block(0,0,3,1);
    }
    /////////
    // arm //
    /////////
    void set_arm_state(Eigen::VectorXd q_arm) {
        assert(q_arm.size()==nJ);
        state.block(3,0,nJ,1) << q_arm;
    }
    Eigen::VectorXd get_arm_state() {
        return state.block(3, 0, nJ, 1);
    }
    void set_arm_dstate(Eigen::VectorXd dq_arm) {
        assert(dq_arm.size()==nJ);
        speed.block(0,0,nJ,1) << dq_arm;
    }
    Eigen::VectorXd get_arm_dstate() {
        return speed.block(0, 0, nJ, 1);
    }
    //////////
    // base //
    //////////
    void set_base_state(Eigen::VectorXd q_base) {
        assert(q_base.size()==nW);
        state.block(3+nJ,0,nW,1) << q_base;
    }
    Eigen::VectorXd get_base_state() {
        return state.block(3+nJ, 0, nW, 1);
    }
    void set_base_dstate(Eigen::VectorXd dq_base) {
        assert(dq_base.size()==nW);
        speed.block(nJ,0,nW,1) << dq_base;
    }
    Eigen::VectorXd get_base_dstate() {
        return speed.block(nJ, 0, nW, 1);
    }
    // state_space
    void update_state(Vehicle vehicle) {
        // arm next
        auto arm_next = get_arm_state() +
            dt*get_arm_dstate();
        set_arm_state(arm_next);
        // base next
        auto base_next = get_base_state() +
            dt*get_base_dstate();
        set_base_state(base_next);
        // chassis_next
        Eigen::Vector4d u = get_base_dstate();
        auto dq = vehicle.odometry(u);
        auto chassis_next = get_chassis_state() + dq;
        set_chassis_state(chassis_next);
    }
    /* save state vector
     *
     * @param state: chassis phi, chassis x, chassis y, J1, J2, J3, J4, J5, W1, W2, W3, W4
     */
    void save_state(bool gripper_state) {
        vector<double> S;
        for (int i=0; i < 12; i++) S.push_back(state(i));
        S.push_back(gripper_state?1:0);
        for (int i=0; i < S.size(); i++) {
            if (i==S.size()-1) std::cout << S[i];
            else std::cout << S[i] << ",";
        }
        std::cout << std::endl;
        //TODO save/publish/write state
        //TODO write coppelasim API
    }

private:
    const int nJ;
    const int nW;
    Eigen::VectorXd state;
    Eigen::VectorXd speed;
    const double dt;
    const double clip;
};
