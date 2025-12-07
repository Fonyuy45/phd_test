#pragma once
#include <chrono>
#include <Eigen/Geometry>
#include <vector>

struct Path
{
    // A series of poses in global frame
    std::vector<Eigen::Isometry2d> poses;
};

struct Twist2d
{
    // Rotational speed in rad/sec
    // Around the center of the parcel
    double rv;
    // Translational speed in m/sec
    // ATTENTION tv is in global frame !
    Eigen::Vector2d tv;
};

class PathFollower
{
public:
    /**
     * @param path The path that should be followed in global frame
     * @param maxWheelSpeed The maximum speed a wheel unter the parcel may reach
     * @param parcelSize The size of the parcel that should follow the box
     *
     * Hint: If looking a the extreme points on a parcel the upper speed boundary of each wheel can be determined
     *       for given the parcel size and a translational and rotational speed. Inverting this equation can be used
     *       to limit the twist given the maximum reachable wheel speed.
     */
    PathFollower(const Path& path, const double maxWheelSpeed, const Eigen::Vector2d &parcelSize);

    /**
     * Called every time a new position is available.
     * It can be assumed that this method is called in a fixed frequency of about
     * 100 hz
     *
     * @param curPose The Position and orientation in global frame
     * @param updateTime The time when the function was called
     */
    void update(const Eigen::Isometry2d& curPose, std::chrono::time_point<std::chrono::system_clock>& updateTime);

    /**
     * Called by an thread with 100 hz
     * This method should compute the next steering command for the object in
     * order to follow the given path.
     *
     * @param curTime The time when the function was called
     *
     * @return A twist in global frame
     */
    Twist2d computeNextCmd(std::chrono::time_point<std::chrono::system_clock>& curTime);
};
