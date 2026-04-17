#include <Eigen/Dense>

#include <cmath>
#include <fstream>
#include <iostream>
#include <random>

#include "ik.hpp"

int main() {
    constexpr int kSampleCount = 50000;
    constexpr double kPi = 3.14159265358979323846;

    // Use the project FK implementation with the requested tool offset.
    ur_kinematics::UR10eKinematics kinematics(0.15);

    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> joint_dist(-kPi, kPi);

    std::ofstream csv_file("workspace_data.csv");
    if (!csv_file.is_open()) {
        std::cerr << "Failed to open workspace_data.csv for writing." << std::endl;
        return 1;
    }

    for (int i = 0; i < kSampleCount; ++i) {
        ur_kinematics::Vector6d q;
        for (int j = 0; j < 6; ++j) {
            q(j) = joint_dist(rng);
        }

        const Eigen::Matrix4d pose = kinematics.forward(q);
        const double x = pose(0, 3);
        const double y = pose(1, 3);
        const double z = pose(2, 3);
        csv_file << x << ',' << y << ',' << z << '\n';
    }

    csv_file.close();
    std::cout << "Generated " << kSampleCount << " workspace samples -> workspace_data.csv" << std::endl;
    return 0;
}
