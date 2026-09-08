#pragma once

#include <apriltag/apriltag.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <opencv2/core.hpp>
#include <array>
#include <functional>
#include <unordered_map>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include "tf2/utils.h"


typedef std::function<geometry_msgs::msg::Transform(apriltag_detection_t* const, const std::array<double, 4>&, const double&, rclcpp::Node::SharedPtr node)> pose_estimation_f;

extern const std::unordered_map<std::string, pose_estimation_f> pose_estimation_methods;

// 与历史 pnp() 保持同一套角点对应，勿改顺序：否则 tf_real_to_dummy 会整体翻转。
// object[i] <-> detection->p[i]
//   p0: (-s/2, -s/2, 0)
//   p1: (+s/2, -s/2, 0)
//   p2: (+s/2, +s/2, 0)
//   p3: (-s/2, +s/2, 0)
std::vector<cv::Point3d> apriltag_object_points(double tagsize);

std::vector<cv::Point2d> apriltag_image_points(const apriltag_detection_t* detection);

cv::Matx33d camera_matrix_from_intr(const std::array<double, 4>& intr);

// 阶段1：IPPE 处理平面翻转歧义，再 ITERATIVE（Gauss-Newton）压重投影误差。
// 物体点顺序保持 apriltag_object_points()，因此不用 SOLVEPNP_IPPE_SQUARE。
bool solve_pnp_ippe_then_iterative(const std::vector<cv::Point3d>& object_pts,
                                   const std::vector<cv::Point2d>& image_pts,
                                   const cv::Matx33d& camera_matrix,
                                   cv::Mat& rvec,
                                   cv::Mat& tvec,
                                   const rclcpp::Logger& logger,
                                   const char* debug_tag);

double reprojection_rms(const std::vector<cv::Point3d>& object_pts,
                        const std::vector<cv::Point2d>& image_pts,
                        const cv::Matx33d& camera_matrix,
                        const cv::Mat& rvec,
                        const cv::Mat& tvec);

geometry_msgs::msg::Transform tf_from_rt(const cv::Mat& tvec, const cv::Mat& rvec);

struct CornerRefineResult
{
    int n_edge_ok{0};
    int n_sample_ok{0};
    int n_sample_all{0};
    bool updated{false};
};

// 阶段3：在 AprilTag refine_edges 之后，用 Sigmoid 边缘扩散模型做亚像素修边。
CornerRefineResult refine_corners_esf(const cv::Mat& gray, double p[4][2]);
