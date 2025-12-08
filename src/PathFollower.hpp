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



private:
    // Path and system parameters
    Path path_;
    double maxWheelSpeed_;        // m/s (omniwheel surface speed)
    Eigen::Vector2d parcelSize_;  // [length, width] in meters
    
    static constexpr double MAX_ANGULAR_SPEED = 1.5;  ///< Max angular velocity (rad/s)
    
    // State variables
    size_t currentWaypoint_;      // Index into path_.poses
    Eigen::Isometry2d currentPose_;  // Current pose in global frame
    
    // Controller parameters (distances in meters, gains dimensionless)
    static constexpr double LOOKAHEAD_DISTANCE = 0.15;
    static constexpr double POSITION_GAIN = 2.0;
    static constexpr double HEADING_GAIN = 3.0;
    static constexpr double WAYPOINT_TOLERANCE = 0.02;
        
    /**
     * @brief Limit velocity to respect corner wheel speed constraints
     * @param desired Desired velocity command
     * @return Scaled velocity command if necessary
     * 
     * Computes velocity at all four corners and scales down the entire
     * twist if any corner exceeds maxWheelSpeed.
     */
    Twist2d limitVelocityForParcel(const Twist2d& desired) const;
    
    /**
     * @brief Find waypoint at lookahead distance along path
     * @return Index of target waypoint at or beyond LOOKAHEAD_DISTANCE from current position
     * 
     * Searches forward from currentWaypoint_, accumulating arc-length until
     * the lookahead distance is reached or path end is encountered.
     */
    size_t findLookaheadWaypoint() const;
    
    /**
     * @brief Extract yaw angle from rotation
     */
    static double getYaw(const Eigen::Rotation2Dd& rot);
    
    /**
     * @brief Normalize angle to [-π, π]
     */
    static double normalizeAngle(double angle);

};
