#include <Eigen/Dense>
#include <vector>
#include "kinetics/trajectory.hpp"
#include "utils.hpp"
#include <numeric>

using namespace std;
class MissionPlanner {
public:
    MissionPlanner(Eigen::MatrixXd _T_se, double _dt=0.01)
        : T_se(_T_se), dt(_dt) {
        std::cout << "MissionPlanner constructed" << std::endl;
    }
    MissionPlanner(const MissionPlanner &copy)
        : T_se(copy.T_se),
          dt(copy.dt) {
        for (int j=0; j<copy.grippers.size(); j++)
            grippers.push_back(copy.grippers[j]);
        for (int i=0; i < copy.trajectory.size(); i++) {
            trajectory.push_back(copy.trajectory[i]);
        }
        std::cout << "MissionPlanner copy constructed" << std::endl;
    }
    bool gripper_state(int time) {
        return grippers[time];
    }
    Eigen::MatrixXd get(int time, bool gripper_state=false) {
        if (gripper_state)
            gripper_state = grippers[time];
        return trajectory[time];
    }
    int size() {
        return trajectory.size();
    }
    void print(int idx) {
        if (idx>=size()) return;
        Eigen::MatrixXd cur = get(idx);
        int s = grippers[idx]?1:0;
        std::cout << cur(0,0) <<  "," <<  cur(0,1) << ","
                  << cur(0,2)<< "," << cur(1,0) << ","
                  << cur(1,1) << "," << cur(1,2) << ","
                  << cur(2,0) << "," << cur(2,1) << ","
                  << cur(2,2) << "," << cur(0,3) << ","
                  << cur(1,3) << "," << cur(2,3) << ","
                  << s << std::endl;
    }
    double time_step() {
        return dt;
    }
protected:
    const double dt;
    Eigen::MatrixXd T_se; // current reference configuration
    vector<Eigen::MatrixXd> trajectory;
    vector<bool> grippers;
};

/* class InPlaceMissionPlanner
 *  keeps the robot open-arm end-effector in place
 *
*/
class InPlaceMissionPlanner : public MissionPlanner {
public:
    InPlaceMissionPlanner(Eigen::MatrixXd T_se, double dt=0.01)
        : MissionPlanner(T_se, dt) {
        std::cout << "InPlace MissionPlanner constructed" << std::endl;
    }
    void operator() (int Time, int k=1) {
        double Tf=Time;
        int N = static_cast<int>(Tf*100*k);
        Trajectory  inplace_traj(Tf, N);
        Eigen::MatrixXd T_d = T_se;
        auto path = inplace_traj.
            ScrewTrajectory(T_se, T_d);
        for (int i=0; i < path.size(); i++) {
            trajectory.push_back(path[i]);
            grippers.push_back(false);
        }
    }
};

/* class PayloadMissionPlanner Payload mission planner for wheeled open arm, on eight interval
 *
 * intervals:
 * 1) to payload stand off
 * 2) to payload grasp
 * 3) grasp
 * 4) to payload stand off
 * 5) to target stand off
 * 6) to target ground
 * 7) release
 * 8) to target stand off
 */
class PayloadMissionPlanner : public MissionPlanner {
public:
    /* constructor PayloadMissionPlanner Payload mission planner for wheeled open arm, on eight interval.
     *
     * intervals:
     * 1) to payload stand off
     * 2) to payload grasp
     * 3) grasp
     * 4) to payload stand off
     * 5) to target stand off
     * 6) to target ground
     * 7) release
     * 8) to target stand off
     *
     * @param T_se: configuration of end-effector in {s} frame.
     * @param duration: duration of each interval in seconds.
     * @param dt: resolution of the Trajectory in second.
     */
    PayloadMissionPlanner(Eigen::MatrixXd T_se,
                          double intervals[8],
                          double dt=0.01)
        : MissionPlanner(T_se, dt) {
        int total=0;
        for (int i=0; i < 8; i++) {
            assert(intervals[i]!=0);
            duration[i] = intervals[i];
        }
    }
    /* trajectory generation
     *
     * @param T_sc: payload transformation relative to {s}
     * @param T_se_d: target transformation relative to {s}
     * @param k: resolution in time step dt
     */
    void operator()(Eigen::MatrixXd T_sc,
                    Eigen::MatrixXd T_sc_d,
                    int k=1) {
        trajectory.resize(0);
        grippers.resize(0);
        //T_d = T_se_d;
        // (1) TO STANDOFF POSITION
        double Tf=duration[0];
        int N = static_cast<int>(Tf*k/dt);
        std::cout << "duration: " << Tf
                  << "dt: " << dt << std::endl;
        std::cout << "N: " << N << std::endl;
        double alpha=M_PI/5;
        Trajectory  stand_off_traj(Tf, N);
        Eigen::MatrixXd T_ce_grasp(4,4);
        T_ce_grasp << -1*sin(alpha), 0, cos(alpha), 0,
            0, 1, 0, 0,
            -1*cos(alpha), 0, -1*sin(alpha), 0,
            0, 0, 0, 1;
        //auto T_ce_grasp=Eigen::Matrix4d::Identity();
        //auto T_ce_standoff=standoff(T_ce_grasp);

        Eigen::MatrixXd T_ce_standoff(4,4);
        T_ce_standoff << -1*sin(alpha), 0, cos(alpha), 0,
            0,1,0,0,
            -1*cos(alpha), 0, -1*sin(alpha), 0.25,
            0,0,0,1;
        auto T_se_standoff = T_sc*T_ce_standoff;
        auto path = stand_off_traj.ScrewTrajectory(T_se, T_se_standoff);
        for (int i=0; i < path.size(); i++) {
            trajectory.push_back(path[i]);
            grippers.push_back(false);
        }
        // (2) FROM STANDOFF TO GRASP
        Tf=duration[1];
        N = static_cast<int>(Tf*k/dt);
        //TODO (res)
        Trajectory  tograsp_traj(Tf, N);
        auto T_se_grasp = T_sc*T_ce_grasp;
        path = tograsp_traj.ScrewTrajectory(T_se_standoff, T_se_grasp);
        for (int i=0; i < path.size(); i++) {
            trajectory.push_back(path[i]);
            grippers.push_back(false);
        }
        // (3) GRASPING
        // the same trajectory for 0.625( or 63 state) 1
        Tf=duration[2];
        N = static_cast<int>(Tf*k/dt);
        auto last_idx=path.size()-1;
        for (int _=0; _ < N; _++) {
            trajectory.push_back(path[last_idx]);
            grippers.push_back(true);
        }
        // (4) FROM GRASP TO STANDOFF
        Tf=duration[3];
        N = static_cast<int>(Tf*k/dt);
        Trajectory tostandoff_traj(Tf, N);
        path = tostandoff_traj.ScrewTrajectory(T_se_grasp, T_se_standoff);
        for (int i=0; i < path.size(); i++) {
            trajectory.push_back(path[i]);
            grippers.push_back(true);
        }
        // (5) FROM STANDOFF TO TARGET STANDOFF
        Tf=duration[4];
        N = static_cast<int>(Tf*k/dt);
        Trajectory  totarget_standoff_traj(Tf, N);
        auto T_se_d = T_sc_d * T_ce_grasp;
        //auto T_se_d = T_sc_d;
        //auto T_se_d_standoff = standoff(T_se_d);
        auto T_se_d_standoff = T_sc_d*T_ce_standoff;
        path = totarget_standoff_traj.ScrewTrajectory(T_se_standoff, T_se_d_standoff);
        for (int i=0; i < path.size(); i++) {
            trajectory.push_back(path[i]);
            grippers.push_back(true);
        }
        // (6) FROM TARGET STANDOFF TO TARGET
        Tf=duration[5];
        N = static_cast<int>(Tf*k/dt);
        Trajectory  totarget_traj(Tf, N);
        path = totarget_traj.ScrewTrajectory(T_se_d_standoff, T_se_d);
        for (int i=0; i < path.size(); i++) {
            trajectory.push_back(path[i]);
            grippers.push_back(true);
        }
        // (7) RELEASE PAYLOAD
        // the same trajectory for 0.625( or 63 state) 0
        Tf=duration[6];
        N = static_cast<int>(Tf*k/dt);
        last_idx=path.size()-1;
        for (int _=0; _ < N; _++) {
            trajectory.push_back(path[last_idx]);
            grippers.push_back(false);
        }
        // (8) BACK TO STAND OFF
        Tf=duration[7];
        N = static_cast<int>(Tf*k/dt);
        Trajectory  target_standoff_traj(Tf, N);
        path = target_standoff_traj.ScrewTrajectory(T_se_d, T_se_d_standoff);
        for (int i=0; i < path.size(); i++) {
            trajectory.push_back(path[i]);
            grippers.push_back(false);
        }
        T_se = trajectory[trajectory.size()-1];
    }
private:
    double duration[8];
};
