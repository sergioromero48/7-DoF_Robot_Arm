#ifndef IK_H
#define IK_H

#include <vector>
#include <cmath>

template <typename T>
T abs(T x) {
    if (x < 0) {
        return -x;
    }
}

// FABRIK Inverse Kinematics
class IK {
public:
    IK();
    ~IK();

    // This function checks if the given point is reachable by the robot
    // Returns true if reachable, false otherwise
    bool isReachable(double x, double y, double z);




private:
    struct joint {
        double length;
        std::vector<double> position;
        std::vector<double> axis;
        joint* parent;
        joint* child;

        joint(double l, std::vector<double> p, std::vector<double> a, joint* par, joint* ch) {
            length = l;
            position = p;
            axis = a;
            parent = par;
            child = ch;
        }
    }
}

#include "ik.hpp"
#endif // IK_H