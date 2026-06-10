#include "TransformUtils.h"

#include <cmath>
#include <iomanip>
#include <sstream>

namespace weld_geometry {

namespace {
const double kPi = 3.14159265358979323846;

double at(const Matrix4& matrix, int row, int col) {
    return matrix.m[static_cast<size_t>(row * 4 + col)];
}

void set(Matrix4& matrix, int row, int col, double value) {
    matrix.m[static_cast<size_t>(row * 4 + col)] = value;
}
} // namespace

double degreeToRadian(double degree) {
    return degree * kPi / 180.0;
}

double radianToDegree(double radian) {
    return radian * 180.0 / kPi;
}

Matrix4 identityMatrix() {
    Matrix4 matrix;
    matrix.m = {{1.0, 0.0, 0.0, 0.0,
                 0.0, 1.0, 0.0, 0.0,
                 0.0, 0.0, 1.0, 0.0,
                 0.0, 0.0, 0.0, 1.0}};
    return matrix;
}

Matrix4 rotationX(double degree) {
    Matrix4 matrix = identityMatrix();
    const double r = degreeToRadian(degree);
    const double c = std::cos(r);
    const double s = std::sin(r);
    set(matrix, 1, 1, c);
    set(matrix, 1, 2, -s);
    set(matrix, 2, 1, s);
    set(matrix, 2, 2, c);
    return matrix;
}

Matrix4 rotationY(double degree) {
    Matrix4 matrix = identityMatrix();
    const double r = degreeToRadian(degree);
    const double c = std::cos(r);
    const double s = std::sin(r);
    set(matrix, 0, 0, c);
    set(matrix, 0, 2, s);
    set(matrix, 2, 0, -s);
    set(matrix, 2, 2, c);
    return matrix;
}

Matrix4 rotationZ(double degree) {
    Matrix4 matrix = identityMatrix();
    const double r = degreeToRadian(degree);
    const double c = std::cos(r);
    const double s = std::sin(r);
    set(matrix, 0, 0, c);
    set(matrix, 0, 1, -s);
    set(matrix, 1, 0, s);
    set(matrix, 1, 1, c);
    return matrix;
}

Matrix4 poseToMatrix(const Pose6D& pose) {
    Matrix4 matrix = multiply(multiply(rotationZ(pose.rz), rotationY(pose.ry)), rotationX(pose.rx));
    set(matrix, 0, 3, pose.x);
    set(matrix, 1, 3, pose.y);
    set(matrix, 2, 3, pose.z);
    return matrix;
}

Pose6D matrixToPose(const Matrix4& matrix) {
    Pose6D pose;
    pose.x = at(matrix, 0, 3);
    pose.y = at(matrix, 1, 3);
    pose.z = at(matrix, 2, 3);

    const double sy = -at(matrix, 2, 0);
    const double ry = std::asin(std::max(-1.0, std::min(1.0, sy)));
    const double cy = std::cos(ry);

    double rx = 0.0;
    double rz = 0.0;
    if (std::fabs(cy) > 1e-9) {
        rx = std::atan2(at(matrix, 2, 1), at(matrix, 2, 2));
        rz = std::atan2(at(matrix, 1, 0), at(matrix, 0, 0));
    } else {
        rx = 0.0;
        rz = std::atan2(-at(matrix, 0, 1), at(matrix, 1, 1));
    }

    pose.rx = radianToDegree(rx);
    pose.ry = radianToDegree(ry);
    pose.rz = radianToDegree(rz);
    return pose;
}

Matrix4 multiply(const Matrix4& a, const Matrix4& b) {
    Matrix4 result = identityMatrix();
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            double sum = 0.0;
            for (int k = 0; k < 4; ++k) {
                sum += at(a, row, k) * at(b, k, col);
            }
            set(result, row, col, sum);
        }
    }
    return result;
}

Matrix4 inverseRigid(const Matrix4& matrix) {
    Matrix4 inv = identityMatrix();
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            set(inv, row, col, at(matrix, col, row));
        }
    }

    for (int row = 0; row < 3; ++row) {
        const double t =
            -(at(inv, row, 0) * at(matrix, 0, 3) +
              at(inv, row, 1) * at(matrix, 1, 3) +
              at(inv, row, 2) * at(matrix, 2, 3));
        set(inv, row, 3, t);
    }
    return inv;
}

Vec3 transformPoint(const Matrix4& matrix, const Vec3& point) {
    Vec3 result;
    result.x = at(matrix, 0, 0) * point.x + at(matrix, 0, 1) * point.y + at(matrix, 0, 2) * point.z + at(matrix, 0, 3);
    result.y = at(matrix, 1, 0) * point.x + at(matrix, 1, 1) * point.y + at(matrix, 1, 2) * point.z + at(matrix, 1, 3);
    result.z = at(matrix, 2, 0) * point.x + at(matrix, 2, 1) * point.y + at(matrix, 2, 2) * point.z + at(matrix, 2, 3);
    return result;
}

std::vector<Vec3> transformPoints(const Matrix4& matrix, const std::vector<Vec3>& points) {
    std::vector<Vec3> result;
    result.reserve(points.size());
    for (size_t i = 0; i < points.size(); ++i) {
        result.push_back(transformPoint(matrix, points[i]));
    }
    return result;
}

double distance(const Vec3& a, const Vec3& b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool isFinite(const Vec3& point) {
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

std::string formatVec3(const Vec3& point) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3)
        << "[" << point.x << ", " << point.y << ", " << point.z << "]";
    return out.str();
}

std::string formatPose(const Pose6D& pose) {
    std::ostringstream out;
    out << std::fixed << std::setprecision(3)
        << "[" << pose.x << ", " << pose.y << ", " << pose.z << ", "
        << pose.rx << ", " << pose.ry << ", " << pose.rz << "]";
    return out.str();
}

} // namespace weld_geometry
