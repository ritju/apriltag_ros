#include "pose_estimation.hpp"
#include <Eigen/Dense>
#include <apriltag/apriltag_pose.h>
#include <apriltag/common/homography.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/quaternion.hpp>
#include <cmath>


std::vector<cv::Point3d> apriltag_object_points(double tagsize)
{
    // 变更说明：顺序必须与历史 pnp() / detection->p 一一对应，不能改成 IPPE_SQUARE 的
    // (-s/2,+s/2) 约定，否则 charger / dummy 坐标系会翻转。
    return {
        {-tagsize / 2, -tagsize / 2, 0},
        {+tagsize / 2, -tagsize / 2, 0},
        {+tagsize / 2, +tagsize / 2, 0},
        {-tagsize / 2, +tagsize / 2, 0},
    };
}

std::vector<cv::Point2d> apriltag_image_points(const apriltag_detection_t* detection)
{
    return {
        {detection->p[0][0], detection->p[0][1]},
        {detection->p[1][0], detection->p[1][1]},
        {detection->p[2][0], detection->p[2][1]},
        {detection->p[3][0], detection->p[3][1]},
    };
}

cv::Matx33d camera_matrix_from_intr(const std::array<double, 4>& intr)
{
    cv::Matx33d cameraMatrix;
    cameraMatrix(0, 0) = intr[0]; // fx
    cameraMatrix(1, 1) = intr[1]; // fy
    cameraMatrix(0, 2) = intr[2]; // cx
    cameraMatrix(1, 2) = intr[3]; // cy
    return cameraMatrix;
}

geometry_msgs::msg::Transform tf_from_rt(const cv::Mat& tvec, const cv::Mat& rvec)
{
    cv::Mat t64, r64;
    tvec.convertTo(t64, CV_64F);
    rvec.convertTo(r64, CV_64F);
    const cv::Quat<double> q = cv::Quat<double>::createFromRvec(r64);

    geometry_msgs::msg::Transform t;
    t.translation.x = t64.at<double>(0);
    t.translation.y = t64.at<double>(1);
    t.translation.z = t64.at<double>(2);
    t.rotation.w = q.w;
    t.rotation.x = q.x;
    t.rotation.y = q.y;
    t.rotation.z = q.z;
    return t;
}

geometry_msgs::msg::Transform tf_from_apriltag_pose(const apriltag_pose_t& pose)
{
    const Eigen::Quaterniond q(Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(pose.R->data));

    geometry_msgs::msg::Transform t;

    t.translation.x = pose.t->data[0];
    t.translation.y = pose.t->data[1];
    t.translation.z = pose.t->data[2];
    t.rotation.w = q.w();
    t.rotation.x = q.x();
    t.rotation.y = q.y();
    t.rotation.z = q.z();

    return t;
}

double reprojection_rms(const std::vector<cv::Point3d>& object_pts,
                        const std::vector<cv::Point2d>& image_pts,
                        const cv::Matx33d& camera_matrix,
                        const cv::Mat& rvec,
                        const cv::Mat& tvec)
{
    if(object_pts.empty() || object_pts.size() != image_pts.size()) {
        return 1e9;
    }
    std::vector<cv::Point2d> proj;
    cv::projectPoints(object_pts, rvec, tvec, camera_matrix, cv::noArray(), proj);
    double sse = 0.0;
    for(size_t i = 0; i < proj.size(); ++i) {
        const double dx = proj[i].x - image_pts[i].x;
        const double dy = proj[i].y - image_pts[i].y;
        sse += dx * dx + dy * dy;
    }
    return std::sqrt(sse / static_cast<double>(proj.size()));
}

bool solve_pnp_ippe_then_iterative(const std::vector<cv::Point3d>& object_pts,
                                   const std::vector<cv::Point2d>& image_pts,
                                   const cv::Matx33d& camera_matrix,
                                   cv::Mat& rvec,
                                   cv::Mat& tvec,
                                   const rclcpp::Logger& logger,
                                   const char* debug_tag)
{
    // 图像已去畸变，distCoeffs 传空。
    bool ok = cv::solvePnP(object_pts, image_pts, camera_matrix, cv::noArray(),
                           rvec, tvec, false, cv::SOLVEPNP_IPPE);
    const char* init_method = "IPPE";
    if(!ok) {
        ok = cv::solvePnP(object_pts, image_pts, camera_matrix, cv::noArray(),
                          rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
        init_method = "ITERATIVE_fallback";
    }
    if(!ok) {
        RCLCPP_WARN(logger, "[pose_opt] %s solvePnP failed (n=%zu)", debug_tag, object_pts.size());
        return false;
    }

    const double rms_before = reprojection_rms(object_pts, image_pts, camera_matrix, rvec, tvec);

    // Gauss-Newton / LM 精修，降低重投影误差。
    cv::solvePnP(object_pts, image_pts, camera_matrix, cv::noArray(),
                 rvec, tvec, true, cv::SOLVEPNP_ITERATIVE);

    const double rms_after = reprojection_rms(object_pts, image_pts, camera_matrix, rvec, tvec);
    RCLCPP_DEBUG(logger, "[pose_opt] %s init=%s n=%zu rms_before=%.3f px rms_after=%.3f px",
                 debug_tag, init_method, object_pts.size(), rms_before, rms_after);
    return true;
}

geometry_msgs::msg::Transform
homography(apriltag_detection_t* const detection, const std::array<double, 4>& intr, double tagsize, rclcpp::Node::SharedPtr node)
{
    (void)node;
    apriltag_detection_info_t info = {detection, tagsize, intr[0], intr[1], intr[2], intr[3]};

    apriltag_pose_t pose;
    estimate_pose_for_tag_homography(&info, &pose);

    return tf_from_apriltag_pose(pose);
}

geometry_msgs::msg::Transform
pnp(apriltag_detection_t* const detection, const std::array<double, 4>& intr, double tagsize, rclcpp::Node::SharedPtr node)
{
    const std::vector<cv::Point3d> objectPoints = apriltag_object_points(tagsize);
    const std::vector<cv::Point2d> imagePoints = apriltag_image_points(detection);
    const cv::Matx33d cameraMatrix = camera_matrix_from_intr(intr);

    cv::Mat rvec, tvec;
    if(!solve_pnp_ippe_then_iterative(objectPoints, imagePoints, cameraMatrix, rvec, tvec, node->get_logger(), "single")) {
        // 保持旧行为：即便失败也尽量返回默认 solvePnP 结果
        cv::solvePnP(objectPoints, imagePoints, cameraMatrix, cv::noArray(), rvec, tvec);
    }

    const double rms = reprojection_rms(objectPoints, imagePoints, cameraMatrix, rvec, tvec);

    const cv::Quat<double> q_ = cv::Quat<double>::createFromRvec(rvec);
    tf2::Quaternion q(q_.x, q_.y, q_.z, q_.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    RCLCPP_DEBUG(node->get_logger(), "=====================================================");
    RCLCPP_DEBUG(node->get_logger(), "tag_size: %f", tagsize);
    RCLCPP_DEBUG(node->get_logger(), "fx: %f, fy: %f, cx: %f, cy: %f", cameraMatrix(0, 0), cameraMatrix(1, 1), cameraMatrix(0, 2), cameraMatrix(1, 2));
    RCLCPP_DEBUG(node->get_logger(), "p0: (%f, %f), p1: (%f, %f)",
                 detection->p[0][0], detection->p[0][1],
                 detection->p[1][0], detection->p[1][1]);
    RCLCPP_DEBUG(node->get_logger(), "p2: (%f, %f), p3: (%f, %f)",
                 detection->p[2][0], detection->p[2][1],
                 detection->p[3][0], detection->p[3][1]);
    RCLCPP_DEBUG(node->get_logger(), "[pose_opt] single rms: %f px", rms);
    RCLCPP_DEBUG(node->get_logger(), "t_x: %f, t_y: %f, t_z: %f", tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
    RCLCPP_DEBUG(node->get_logger(), "roll: %f, pitch: %f, yaw: %f", roll, pitch, yaw);

    return tf_from_rt(tvec, rvec);
}

const std::unordered_map<std::string, pose_estimation_f> pose_estimation_methods{
    {"homography", homography},
    {"pnp", pnp},
};
