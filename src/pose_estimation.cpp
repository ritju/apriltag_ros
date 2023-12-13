#include "pose_estimation.hpp"
#include <Eigen/Dense>
#include <apriltag/apriltag_pose.h>
#include <apriltag/common/homography.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/quaternion.hpp>


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

geometry_msgs::msg::Transform tf_from_cv(const cv::Mat_<double>& tvec, const cv::Mat_<double>& rvec)
{
    const cv::Quat<double> q = cv::Quat<double>::createFromRvec(rvec);

    geometry_msgs::msg::Transform t;

    t.translation.x = tvec.at<double>(0);
    t.translation.y = tvec.at<double>(1);
    t.translation.z = tvec.at<double>(2);
    t.rotation.w = q.w;
    t.rotation.x = q.x;
    t.rotation.y = q.y;
    t.rotation.z = q.z;

    return t;
}

pose_estimation_f apriltag_homography = [](apriltag_detection_t* const detection, const std::array<double, 4>& intr, double tagsize) -> geometry_msgs::msg::Transform {
    apriltag_detection_info_t info = {detection, tagsize, intr[0], intr[1], intr[2], intr[3]};

    apriltag_pose_t pose;
    estimate_pose_for_tag_homography(&info, &pose);

    return tf_from_apriltag_pose(pose);
};

pose_estimation_f solve_pnp = [](apriltag_detection_t* const detection, const std::array<double, 4>& intr, double tagsize) -> geometry_msgs::msg::Transform {
    const double half_tagsize = 0.5 * tagsize;
    const std::vector<cv::Point3d> objectPoints{{-half_tagsize, -half_tagsize, 0}, {+half_tagsize, -half_tagsize, 0}, {+half_tagsize, +half_tagsize, 0}, {-half_tagsize, +half_tagsize, 0}};

    std::vector<cv::Point2d> imagePoints;
    constexpr double tag_x[4] = {-1, 1, 1, -1};
    constexpr double tag_y[4] = {1, 1, -1, -1};
    for(int i = 0; i < 4; i++) {
        // Homography projection taking tag local frame coordinates to image pixels
        double im_x, im_y;
        homography_project(detection->H, tag_x[i], tag_y[i], &im_x, &im_y);
        imagePoints.push_back(cv::Point2d(im_x, im_y));
    }

    cv::Mat rvec, tvec;
    cv::Matx33d cameraMatrix;
    cameraMatrix(0, 0) = intr[0];// fx
    cameraMatrix(1, 1) = intr[1];// fy
    cameraMatrix(0, 2) = intr[2];// cx
    cameraMatrix(1, 2) = intr[3];// cy
    // with "SOLVEPNP_IPPE_SQUARE"?
    cv::solvePnP(objectPoints, imagePoints, cameraMatrix, {}, rvec, tvec);

    return tf_from_cv(tvec, rvec);
};

const std::unordered_map<std::string, pose_estimation_f> pose_estimation_methods{
    {"homography", apriltag_homography},
    {"pnp", solve_pnp},
};
