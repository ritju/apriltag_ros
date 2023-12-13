#pragma once

#include <apriltag/apriltag.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <unordered_map>


typedef std::function<geometry_msgs::msg::Transform(apriltag_detection_t* const, const std::array<double, 4>&, const double&)> estim_pose_f;

extern const std::unordered_map<std::string, estim_pose_f> estim_pose_fun;
