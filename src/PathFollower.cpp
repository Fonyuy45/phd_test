#include "PathFollower.hpp"
#include <iostream>

PathFollower::PathFollower(const Path& path, const double maxWheelSpeed, const Eigen::Vector2d &parcelSize) {}

void PathFollower::update(const Eigen::Isometry2d& curPose,
                          std::chrono::time_point<std::chrono::system_clock>& updateTime)
{
    std::cout << "Cur Pos : " << curPose.translation().transpose() << " Rotation " << Eigen::Rotation2Dd(curPose.rotation()).angle() / M_PI  * 180 << std::endl;
}

Twist2d PathFollower::computeNextCmd(std::chrono::time_point<std::chrono::system_clock>& curTime)
{
    return Twist2d{.rv = 0, .tv = Eigen::Vector2d::Zero()};
}
