#include "fr5_vizum_driver/joint_state_utils.hpp"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <string>
#include <vector>

TEST(JointStateUtils, ConvertsFairinoDegreesToRosRadians) {
    const std::array<double, 6> degrees{{0.0, 90.0, -90.0, 180.0, -180.0, 45.0}};

    const std::array<double, 6> radians = fr5_vizum_driver::degreesToRadians(degrees);

    EXPECT_DOUBLE_EQ(radians[0], 0.0);
    EXPECT_NEAR(radians[1], M_PI / 2.0, 1e-12);
    EXPECT_NEAR(radians[2], -M_PI / 2.0, 1e-12);
    EXPECT_NEAR(radians[3], M_PI, 1e-12);
    EXPECT_NEAR(radians[4], -M_PI, 1e-12);
    EXPECT_NEAR(radians[5], M_PI / 4.0, 1e-12);
}

TEST(JointStateUtils, UsesFairinoUrdfJointNamesInOrder) {
    const std::vector<std::string> names = fr5_vizum_driver::defaultJointNames();

    ASSERT_EQ(names.size(), 6u);
    EXPECT_EQ(names[0], "j1");
    EXPECT_EQ(names[1], "j2");
    EXPECT_EQ(names[2], "j3");
    EXPECT_EQ(names[3], "j4");
    EXPECT_EQ(names[4], "j5");
    EXPECT_EQ(names[5], "j6");
}
