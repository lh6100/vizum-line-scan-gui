#ifndef TRANSFORM_UTILS_H
#define TRANSFORM_UTILS_H

#include <array>
#include <string>
#include <vector>

namespace weld_geometry {

struct Vec3 {
    double x;
    double y;
    double z;
};

struct Pose6D {
    double x;
    double y;
    double z;
    double rx;
    double ry;
    double rz;
};

struct Matrix4 {
    std::array<double, 16> m;
};

double degreeToRadian(double degree);
double radianToDegree(double radian);

Matrix4 identityMatrix();
Matrix4 rotationX(double degree);
Matrix4 rotationY(double degree);
Matrix4 rotationZ(double degree);
Matrix4 poseToMatrix(const Pose6D& pose);
Pose6D matrixToPose(const Matrix4& matrix);

Matrix4 multiply(const Matrix4& a, const Matrix4& b);
Matrix4 inverseRigid(const Matrix4& matrix);

Vec3 transformPoint(const Matrix4& matrix, const Vec3& point);
std::vector<Vec3> transformPoints(const Matrix4& matrix, const std::vector<Vec3>& points);

double distance(const Vec3& a, const Vec3& b);
bool isFinite(const Vec3& point);
std::string formatVec3(const Vec3& point);
std::string formatPose(const Pose6D& pose);

} // namespace weld_geometry

#endif // TRANSFORM_UTILS_H
