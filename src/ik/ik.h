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

    // This function calculates the directional vector for forward and backwards reaching
    std::vector<double> unitVector(std::vector<double> v);




private:
    // Vector3: Represents a point or vector in 3D space
    struct Vector3 {
        double x, y, z;

        Vector3(double x = 0.0, double y = 0.0, double z = 0.0) : x(x), y(y), z(z) {}

        // Add two vectors
        Vector3 operator+(const Vector3& v) const {
            return Vector3(x + v.x, y + v.y, z + v.z);
        }

        // Subtract two vectors
        Vector3 operator-(const Vector3& v) const {
            return Vector3(x - v.x, y - v.y, z - v.z);
        }

        // Multiply vector by a scalar
        Vector3 operator*(double scalar) const {
            return Vector3(x * scalar, y * scalar, z * scalar);
        }

        // Divide vector by a scalar
        Vector3 operator/(double scalar) const {
            return Vector3(x / scalar, y / scalar, z / scalar);
        }

        // Normalize the vector
        void normalize() {
            double len = std::sqrt(x * x + y * y + z * z);
            x /= len; y /= len; z /= len;
        }

        // Compute the dot product of two vectors
        static double dot(const Vector3& a, const Vector3& b) {
            return a.x * b.x + a.y * b.y + a.z * b.z;
        }

        // Compute the Euclidean distance between two points
        static double distance(const Vector3& a, const Vector3& b) {
            return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2));
        }
    };
    // Joint: Represents a joint in the kinematic chain
    struct Joint {
        Vector3 position;  // Position of the joint
        double length;     // Length to the next joint

        Joint(const Vector3& pos, double len) : position(pos), length(len) {}
    };
}

#include "ik.hpp"
#endif // IK_H