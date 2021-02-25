#include <Eigen/Dense>

#include <vector>
#include "kinetics/trajectory.hpp"
#include "utils.hpp"

using namespace std;
class MissionPlanner {
public:
    MissionPlanner(Eigen::MatrixXd T_se)
        : T_se(T_se) {
    }
    MissionPlanner(const MissionPlanner &copy)
        : T_se(copy.T_se), T_d(copy.T_d) {
        for (int j=0; j<copy.grippers.size(); j++)
            grippers.push_back(copy.grippers[j]);
        for (int i=0; i < copy.trajectory.size(); i++) {
            trajectory.push_back(copy.trajectory[i]);
        }
    }
    bool gripper_state(int time) {
        return grippers[time];
    }
    Eigen::MatrixXd get (int time,
                         bool gripper_state=false) {
        if (gripper_state)
            gripper_state = grippers[time];
        return trajectory[time];
    }
    int size() {
        return trajectory.size();
    }
    Eigen::MatrixXd target() {
        return T_d;
    }
protected:
    Eigen::MatrixXd T_se;
    Eigen::MatrixXd T_d; //desired target/destination
    vector<Eigen::MatrixXd> trajectory;
    vector<bool> grippers;
};

/* class InPlaceMissionPlanner
 *  keeps the robot open-arm end-effector in place
 *
*/
class InPlaceMissionPlanner : public MissionPlanner {
public:
    InPlaceMissionPlanner(Eigen::MatrixXd T_se)
        : MissionPlanner(T_se) {
        T_d=T_se;
    }
    void operator() (int Time, int k=1) {
        double Tf=Time;
        int N = static_cast<int>(Tf*100*k);
        Trajectory  inplace_traj(Tf, N);
        auto path = inplace_traj.
            ScrewTrajectory(T_se, T_se);
        for (int i=0; i < path.size(); i++) {
            trajectory.push_back(path[i]);
            grippers.push_back(false);
        }
    }
};
/* TIME
 * dt = Tf / (N-1)
 * given k resolution per 0.01 second
 * dt = 0.01/k
 * N_second = 100*k
 * N = Tf*N_second
 */
//TODO optimal time trajectory you save you all this trouble to choosing the time, but notice, this might not be always the case, othertimes you don't care about the time, and it's more convenient to optimize.
class PayloadMissionPlanner : public MissionPlanner {
public:
    PayloadMissionPlanner(Eigen::MatrixXd T_se)
        : MissionPlanner(T_se) {
    }
    /* trajectory generation
     *
     * @param T_sc_initial: payload transformation relative to {s}
     * @param T_se_d: target transformation relative to {s}
     * @param gripper_idx: index for referencing grippers
     * @param k: resolution in time step dt
     */
    void operator()(Eigen::MatrixXd T_sc_initial,
                    Eigen::MatrixXd T_se_d,
                    int gripper_idx=0,
                    int k=1) {
        //resize vectors
        trajectory.resize(0);
        grippers.resize(0);
        //
        T_d = T_se_d;
        //TODO (fix) total time is 30 seconds.
        vector<Eigen::MatrixXd> trajectory;
        // (1) TO STANDOFF POSITION
        double Tf=5;
        int N = static_cast<int>(Tf*100*k);
        Trajectory  stand_off_traj(Tf, N);
        //TODO (res) generalize this assumption
        auto T_ce_grasp=Eigen::Matrix4d::Identity();
        auto T_ce_standoff=standoff(T_ce_grasp);
        auto T_se_standoff = T_sc_initial*T_ce_standoff;
        auto T_se_initial = T_se;
        auto path = stand_off_traj.
            ScrewTrajectory(T_se_initial, T_se_standoff);
        for (int i=0; i < path.size(); i++) {
            trajectory.push_back(path[i]);
            grippers.push_back(false);
        }
        // (2) FROM STANDOFF TO GRASP
        Tf=2;
        N = static_cast<int>(Tf*100*k);
        Trajectory  tograsp_traj(Tf, N);
        auto T_se_grasp = T_sc_initial*T_ce_grasp;
        path = tograsp_traj.
            ScrewTrajectory(T_se_standoff, T_se_grasp);
        for (int i=0; i < path.size(); i++) {
            trajectory.push_back(path[i]);
            grippers.push_back(false);
        }
        // (3) GRASPING
        // the same trajectory for 0.625( or 63 state) 1
        auto last_idx=path.size()-1;
        for (int _=0; _ < 63; _++) {
            trajectory.push_back(path[last_idx]);
            grippers.push_back(true);
        }
        // (4) FROM GRASP TO STANDOFF
        Tf=3;
        N = static_cast<int>(Tf*100*k);
        Trajectory tostandoff_traj(Tf, N);
        path = tostandoff_traj.
            ScrewTrajectory(T_se_grasp, T_se_standoff);
        for (int i=0; i < path.size(); i++) {
            trajectory.push_back(path[i]);
            grippers.push_back(true);
        }
        // (5) FROM STANDOFF TO TARGET STANDOFF
        Tf=10;
        N = static_cast<int>(Tf*100*k);
        Trajectory  totarget_standoff_traj(Tf, N);
        auto T_se_d_standoff = standoff(T_se_d);
        path = totarget_standoff_traj.
            ScrewTrajectory(T_se_standoff, T_se_d_standoff);
        for (int i=0; i < path.size(); i++) {
            trajectory.push_back(path[i]);
            grippers.push_back(true);
        }
        // (6) FROM TARGET STANDOFF TO TARGET
        Tf=3;
        N = static_cast<int>(Tf*100*k);
        Trajectory  totarget_traj(Tf, N);
        path = totarget_traj.
            ScrewTrajectory(T_se_d_standoff, T_se_d);
        for (int i=0; i < path.size(); i++) {
            trajectory.push_back(path[i]);
            grippers.push_back(true);
        }
        // (7) RELEASE PAYLOAD
        // the same trajectory for 0.625( or 63 state) 0
        last_idx=path.size()-1;
        for (int _=0; _ < 63; _++) {
            trajectory.push_back(path[last_idx]);
            grippers.push_back(false);
        }
        // (8) BACK TO STAND OFF
        Tf=3;
        N = static_cast<int>(Tf*100*k);
        Trajectory  target_standoff_traj(Tf, N);
        path = target_standoff_traj.
            ScrewTrajectory(T_se_d, T_se_d_standoff);
        for (int i=0; i < path.size(); i++) {
            trajectory.push_back(path[i]);
            grippers.push_back(false);
        }
        T_se = trajectory[trajectory.size()-1];
    }
};
