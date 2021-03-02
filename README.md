# kuka is open arm installed on omni-directional macanum wheeled vehicle

    build on top of:
        - [kinetics](https://github.com/ertosns/kinetics.git)
        - [algebra](http://github.com/ertosns/algebra.git)


# example

```cpp
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
    PI_Controller controller(state, kinematics, Controller::diff_trans, 5, 10, dt);
    Kuka kuka(kinematics, macanum, trajectory, state, controller);
    kuka();
```

# output
![alt text](https://github.com/ertosns/kuka/blob/main/data/ezgif.com-optimize.gif)
