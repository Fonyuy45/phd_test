/**
 * @file PathFollower.cpp
 * @brief Implementation of holonomic lookahead controller with kinematic velocity limiting
 * @author Dieudonné YUFONYUY
 * @date December 2025
 * 
 * This implementation provides path following for the cellumation celluveyor system,
 * combining proportional control with lookahead prediction and corner velocity constraints.
 */

#include "PathFollower.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>

PathFollower::PathFollower(const Path& path, double maxWheelSpeed, 
                           const Eigen::Vector2d& parcelSize)
    : path_(path)
    , maxWheelSpeed_(maxWheelSpeed)
    , parcelSize_(parcelSize)
    , currentWaypoint_(0)
    , currentPose_(Eigen::Isometry2d::Identity())
{
    // Validate inputs
    if (path_.poses.empty()) {
        throw std::runtime_error("PathFollower: Path cannot be empty");
    }
    if (maxWheelSpeed_ <= 0.0) {
        throw std::runtime_error("PathFollower: Max wheel speed must be positive");
    }
    if (parcelSize_.x() <= 0.0 || parcelSize_.y() <= 0.0) {
        throw std::runtime_error("PathFollower: Parcel size must be positive");
    }
    
    // Initialize pose to start of path
    currentPose_ = path_.poses.front();
}

void PathFollower::update(const Eigen::Isometry2d& curPose,
                         std::chrono::time_point<std::chrono::system_clock>& updateTime)
{
    (void)updateTime;  // Time parameter unused in this simple implementation
    currentPose_ = curPose;
}

Twist2d PathFollower::computeNextCmd(std::chrono::time_point<std::chrono::system_clock>& curTime)
{
    (void)curTime;  // Time parameter unused in this simple implementation
    
    // 1. Find target waypoint with lookahead
    size_t targetIdx = findLookaheadWaypoint();
    
    // Defensive programming: ensure valid index (should never happen, but safe)
    if (targetIdx >= path_.poses.size()) {
        return Twist2d{.rv = 0.0, .tv = Eigen::Vector2d::Zero()};
    }
    
    const Eigen::Isometry2d& target = path_.poses[targetIdx];
    
    // 2. Compute desired translational velocity (proportional control)
    Eigen::Vector2d posError = target.translation() - currentPose_.translation();
    Eigen::Vector2d desiredTV = POSITION_GAIN * posError;
    
    // CRITICAL: Limit velocity magnitude to prevent instability
    // Without this, large position errors can cause excessive velocities
    const double maxDesiredSpeed = LOOKAHEAD_DISTANCE * POSITION_GAIN;
    if (desiredTV.norm() > maxDesiredSpeed) {
        desiredTV = desiredTV.normalized() * maxDesiredSpeed;
    }
    
    // 3. Compute desired rotational velocity (proportional heading control)
    double targetYaw = getYaw(Eigen::Rotation2Dd(target.rotation()));
    double currentYaw = getYaw(Eigen::Rotation2Dd(currentPose_.rotation()));
    double headingError = normalizeAngle(targetYaw - currentYaw);
    double desiredRV = HEADING_GAIN * headingError;
    desiredRV = std::clamp(desiredRV, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
    
    // 4. Apply kinematic velocity limiting based on parcel geometry
    // This ensures no corner wheel exceeds maxWheelSpeed
    Twist2d cmd{.rv = desiredRV, .tv = desiredTV};
    cmd = limitVelocityForParcel(cmd);
    
    // 5. Advance waypoint when close enough
    if (posError.norm() < WAYPOINT_TOLERANCE && currentWaypoint_ < path_.poses.size() - 1) {
        currentWaypoint_++;
    }
    
    return cmd;
}

Twist2d PathFollower::limitVelocityForParcel(const Twist2d& desired) const
{
    // Half dimensions of parcel
    const double a = parcelSize_.x() / 2.0;  // half-length
    const double b = parcelSize_.y() / 2.0;  // half-width
    
    // Four corner positions relative to center
    // Using stack array for simplicity (could be precomputed for optimization)
    const Eigen::Vector2d corners[4] = {
        { a,  b},   // Front-right
        { a, -b},   // Front-left
        {-a,  b},   // Back-right
        {-a, -b}    // Back-left
    };
    
    // Find maximum corner velocity
    // Physics: v_corner = v_center + ω × r
    // In 2D: (vx, vy) + ω × (dx, dy) = (vx - ω*dy, vy + ω*dx)
    double maxCornerSpeed = 0.0;
    for (const auto& corner : corners) {
        Eigen::Vector2d vCorner;
        vCorner.x() = desired.tv.x() - desired.rv * corner.y();
        vCorner.y() = desired.tv.y() + desired.rv * corner.x();
        
        double cornerSpeed = vCorner.norm();
        maxCornerSpeed = std::max(maxCornerSpeed, cornerSpeed);
    }
    
    // Scale down entire twist if any corner exceeds limit
    // Note: We scale both linear and angular velocities proportionally
    // to maintain the intended motion profile while respecting constraints
    if (maxCornerSpeed > maxWheelSpeed_) {
        const double scale = maxWheelSpeed_ / maxCornerSpeed;
        return Twist2d{
            .rv = desired.rv * scale,
            .tv = desired.tv * scale
        };
    }
    
    return desired;
}

size_t PathFollower::findLookaheadWaypoint() const
{
    // Handle edge case: already at or past end of path
    if (currentWaypoint_ >= path_.poses.size() - 1) {
        return path_.poses.size() - 1;
    }
    
    // Accumulate arc-length along path from current waypoint
    double accumulated = 0.0;
    for (size_t i = currentWaypoint_; i < path_.poses.size() - 1; ++i) {
        const Eigen::Vector2d segment = path_.poses[i + 1].translation() - 
                                        path_.poses[i].translation();
        accumulated += segment.norm();
        
        // Return waypoint when accumulated distance meets or exceeds lookahead
        if (accumulated >= LOOKAHEAD_DISTANCE) {
            return i + 1;
        }
    }
    
    // If lookahead distance exceeds remaining path, return last waypoint
    return path_.poses.size() - 1;
}

double PathFollower::getYaw(const Eigen::Rotation2Dd& rot)
{
    return rot.angle();
}

double PathFollower::normalizeAngle(double angle)
{
    // Normalize angle to [-π, π] using modulo arithmetic
    // More efficient than while loop for large angles
    angle = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (angle < 0.0) {
        angle += 2.0 * M_PI;
    }
    return angle - M_PI;
}