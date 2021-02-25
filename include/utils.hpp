#pragma once
#include <Eigen/Dense>
#include <vector>
#include <iostream>

using namespace std;


/* calculate the standoff position for payload with given configuration
 *
 * @param T: payload configuration/transformation matrix
 */
Eigen::MatrixXd standoff(Eigen::MatrixXd T) {
    Eigen::MatrixXd tmp = T;
    tmp(2,3) = T(2,3)+0.1; // 10cm, if T is configuration of an object, make sure it's the tip of the object.
    return tmp;
}
