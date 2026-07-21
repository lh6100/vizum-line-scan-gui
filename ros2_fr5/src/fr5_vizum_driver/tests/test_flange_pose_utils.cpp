#include "fr5_vizum_driver/flange_pose_utils.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>

TEST(FlangePoseUtils, ConvertsFairinoMillimetersAndDegreesToRosUnits) {
    const std::array<double, 6> raw{{100.0, -250.0, 1234.5, 90.0, -45.0, 180.0}};

    const fr5_vizum_driver::FlangePoseValues pose =
        fr5_vizum_driver::fairinoFlangePoseToRos(raw);

    EXPECT_DOUBLE_EQ(pose.translationMeters[0], 0.1);
    EXPECT_DOUBLE_EQ(pose.translationMeters[1], -0.25);
    EXPECT_DOUBLE_EQ(pose.translationMeters[2], 1.2345);
    EXPECT_NEAR(pose.rpyRadians[0], M_PI / 2.0, 1e-12);
    EXPECT_NEAR(pose.rpyRadians[1], -M_PI / 4.0, 1e-12);
    EXPECT_NEAR(pose.rpyRadians[2], M_PI, 1e-12);
}

TEST(FlangePoseUtils, RejectsNonFiniteControllerValues) {
    std::array<double, 6> raw{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    EXPECT_TRUE(fr5_vizum_driver::allFinite(raw));

    raw[2] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(fr5_vizum_driver::allFinite(raw));
}
