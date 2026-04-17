#include <Eigen/Dense>

#include <array>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>

#include "ik.hpp"

namespace {

struct TestCase {
    std::string name;
    ur_kinematics::Vector6d q_true;
};

double max_abs_joint_error(const ur_kinematics::Vector6d& a, const ur_kinematics::Vector6d& b) {
    double max_err = 0.0;
    for (int i = 0; i < 6; ++i) {
        max_err = std::max(max_err, std::abs(a(i) - b(i)));
    }
    return max_err;
}

bool is_tool_vertical_down(const Eigen::Matrix4d& pose, double cosine_threshold = 0.98) {
    const Eigen::Vector3d tool_z_axis = pose.block<3, 1>(0, 2);
    const double alignment = tool_z_axis.dot(Eigen::Vector3d(0.0, 0.0, -1.0));
    return alignment >= cosine_threshold;
}

}  // namespace

int main() {
    ur_kinematics::UR10eKinematics kinematics(0.15);

    // Representative screw-above postures (left-upper / center / right-lower) with downward tool orientation.
    const std::array<TestCase, 3> tests = {
        TestCase{"left_upper_screw", (ur_kinematics::Vector6d() << -1.95, -2.20, 2.25, -1.62, -1.58, 0.20).finished()},
        TestCase{"center_screw", (ur_kinematics::Vector6d() << -1.57, -2.15, 2.20, -1.62, -1.57, 0.00).finished()},
        TestCase{"right_lower_screw", (ur_kinematics::Vector6d() << -1.20, -2.08, 2.14, -1.63, -1.56, -0.18).finished()},
    };

    std::ofstream csv_file("kinematic_validation_results.csv");
    if (!csv_file.is_open()) {
        std::cerr << "Failed to open kinematic_validation_results.csv for writing." << std::endl;
        return 1;
    }

    csv_file
        << "Test_Point,True_Q1,True_Q2,True_Q3,True_Q4,True_Q5,True_Q6,"
        << "Pose_X,Pose_Y,Pose_Z,Quat_X,Quat_Y,Quat_Z,Quat_W,"
        << "IK_Q1,IK_Q2,IK_Q3,IK_Q4,IK_Q5,IK_Q6,Max_Abs_Error\n";

    csv_file << std::fixed << std::setprecision(10);

    int success_count = 0;

    for (const auto& test : tests) {
        const Eigen::Matrix4d pose = kinematics.forward(test.q_true);
        const Eigen::Matrix3d rotation = pose.block<3, 3>(0, 0);
        Eigen::Quaterniond quat(rotation);
        quat.normalize();

        ur_kinematics::Vector6d q_ik_out = ur_kinematics::Vector6d::Zero();
        const bool ik_ok = kinematics.inverse(pose, test.q_true, q_ik_out);

        const double max_err = ik_ok ? max_abs_joint_error(test.q_true, q_ik_out)
                                     : std::numeric_limits<double>::quiet_NaN();

        if (ik_ok) {
            ++success_count;
        }

        if (!is_tool_vertical_down(pose)) {
            std::cerr << "Warning: " << test.name
                      << " is not strictly vertical-down by threshold." << std::endl;
        }

        csv_file << test.name;
        for (int i = 0; i < 6; ++i) {
            csv_file << ',' << test.q_true(i);
        }

        csv_file << ',' << pose(0, 3) << ',' << pose(1, 3) << ',' << pose(2, 3);
        csv_file << ',' << quat.x() << ',' << quat.y() << ',' << quat.z() << ',' << quat.w();

        for (int i = 0; i < 6; ++i) {
            csv_file << ',' << q_ik_out(i);
        }

        csv_file << ',' << max_err << '\n';
    }

    csv_file.close();

    std::cout << "Generated kinematic_validation_results.csv with " << tests.size() << " test cases." << std::endl;
    std::cout << "IK success: " << success_count << "/" << tests.size() << std::endl;
    return 0;
}
