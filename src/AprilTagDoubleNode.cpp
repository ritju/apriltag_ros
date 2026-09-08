// ros
#include "pose_estimation.hpp"
#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#ifdef cv_bridge_HPP
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <opencv2/imgproc.hpp>
#include <image_transport/camera_subscriber.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include <sstream>
#include "aruco_msgs/msg/pose_with_id.hpp"
#include "aruco_msgs/msg/marker_and_mac.hpp"
#include "aruco_msgs/msg/marker_and_mac_vector.hpp"
#include "std_msgs/msg/string.hpp"
#include "capella_ros_service_interfaces/msg/charge_marker_visible.hpp"
#include <math.h>
#include <algorithm>
#include <cmath>
#include <string>

// 新增服务头文件
#include "capella_ros_service_interfaces/srv/start_detect_apriltag.hpp"
#include "capella_ros_service_interfaces/srv/stop_detect_apriltag.hpp"
#include "capella_ros_service_interfaces/srv/get_detect_apriltag_status.hpp"
#include "capella_ros_service_interfaces/srv/is_in_charger_range.hpp"

// apriltag
#include "tag_functions.hpp"
#include <apriltag.h>
#include <angles/angles.h>

#include <visualization_msgs/msg/marker.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <future>
#include <rclcpp/node_interfaces/node_topics_interface.hpp>

#define IF(N, V) \
    if(assign_check(parameter, N, V)) continue;

template<typename T>
void assign(const rclcpp::Parameter& parameter, T& var)
{
    var = parameter.get_value<T>();
}

template<typename T>
void assign(const rclcpp::Parameter& parameter, std::atomic<T>& var)
{
    var = parameter.get_value<T>();
}

template<typename T>
bool assign_check(const rclcpp::Parameter& parameter, const std::string& name, T& var)
{
    if(parameter.get_name() == name) {
        assign(parameter, var);
        return true;
    }
    return false;
}

rcl_interfaces::msg::ParameterDescriptor
descr(const std::string& description, const bool& read_only = false)
{
    rcl_interfaces::msg::ParameterDescriptor descr;
    descr.description = description;
    descr.read_only = read_only;
    return descr;
}

class AprilTagDoubleNode : public rclcpp::Node {
public:
    AprilTagDoubleNode(const rclcpp::NodeOptions& options);
    ~AprilTagDoubleNode() override;

private:
    const OnSetParametersCallbackHandle::SharedPtr cb_parameter;

    apriltag_family_t* tf;
    apriltag_detector_t* const td;

    std::mutex mutex;
    double tag_edge_size;
    std::atomic<int> max_hamming;
    std::atomic<bool> profile;
    std::unordered_map<int, std::string> tag_frames;
    std::unordered_map<int, double> tag_sizes;

    std::function<void(apriltag_family_t*)> tf_destructor;

    // 修改：图像和相机信息订阅
    image_transport::Subscriber image_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr last_camera_info_;

    // 新增：缓存最新图像和相机信息（用于延迟处理）
    std::mutex img_mutex_;
    sensor_msgs::msg::Image::ConstSharedPtr last_img_;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr last_ci_;
    rclcpp::Time last_img_time_;

    rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr pub_detections;
    tf2_ros::TransformBroadcaster tf_broadcaster;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

    pose_estimation_f estimate_pose = nullptr;

    void onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img,
                  const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_ci);

    rcl_interfaces::msg::SetParametersResult onParameter(const std::vector<rclcpp::Parameter>& parameters);

    tf2::Transform tf_real_to_dummy;
    tf2::Transform tf_base_link_to_dummy_base_link;
    bool getTransform(const std::string & refFrame, const std::string & childFrame,
                      geometry_msgs::msg::TransformStamped & transform);
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    rclcpp::Publisher<aruco_msgs::msg::PoseWithId>::SharedPtr pose_with_id_pub;
    rclcpp::Publisher<aruco_msgs::msg::MarkerAndMacVector>::SharedPtr id_and_mac_pub;
    rclcpp::Publisher<capella_ros_service_interfaces::msg::ChargeMarkerVisible>::SharedPtr detect_status;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    std::vector<geometry_msgs::msg::Point> rect_corners_;

    aruco_msgs::msg::MarkerAndMacVector msgs;
    aruco_msgs::msg::MarkerAndMac msg;
    std::string charger_id_;
    bool id_selected = false;
    std::vector<std::string> marker_id_and_bluetooth_mac_vector;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr charger_id_sub;

    int marker_id;
    int marker_id_correction;
    bool marker_visible_last = false;
    bool marker_visible_pub = false;
    float marker_frame_translation;
    capella_ros_service_interfaces::msg::ChargeMarkerVisible marker_detect_status;
    std::string apriltag_family_name;

    void charger_id_callback(std_msgs::msg::String msg);
    void marker_visible_callback();
    bool in_idRanges(std::vector<int> ids);

    tf2::Transform tf_marker1_to_charger;
    tf2::Transform tf_camera_to_marker1, tf_camera_to_marker2;
    tf2::Transform tf_marker1_to_marker2_fixed;
    tf2::Stamped<tf2::Transform> stamped_tf_camera_to_marker1;
    tf2::Stamped<tf2::Transform> stamped_tf_camera_to_marker2;
    std::vector<std::pair<int, geometry_msgs::msg::TransformStamped>> id_and_tf_vec;
    float similarity_threshold;
    float radius_threshold;
    float base_link_dummy_transform_x, base_link_dummy_transform_y, base_link_dummy_transform_z;

    // 调试用（保留）
    tf2::Stamped<tf2::Transform> camera_pose_last1;
    tf2::Stamped<tf2::Transform> camera_pose_last2;
    tf2::Stamped<tf2::Transform> camera_pose_current1;
    tf2::Stamped<tf2::Transform> camera_pose_current2;
    bool pose_inited = false;

    tf2::Transform tf_baselink_to_camera;

    int frame_all = 0;
    int frame_detected = 0;
    int frame_not_detected = 0;
    int frame_error = 0;

    double now_time = 0.0, last_time_camera_topic_received = 0.0;

    // ========== 新增成员变量 ==========
    std::atomic<bool> detecting_apriltag_{false};           // 是否正在检测
    std::atomic<bool> detecting_in_charger_range_{false};   // 是否在充电桩范围内
    rclcpp::TimerBase::SharedPtr process_timer_;            // 处理定时器

    // 充电桩范围参数（参考 manual.cpp）
    float pose_x_min_, pose_x_max_, pose_y_min_, pose_y_max_, yaw_min_, yaw_max_;

    // 新增服务
    rclcpp::Service<capella_ros_service_interfaces::srv::StartDetectApriltag>::SharedPtr start_srv_;
    rclcpp::Service<capella_ros_service_interfaces::srv::StopDetectApriltag>::SharedPtr stop_srv_;
    rclcpp::Service<capella_ros_service_interfaces::srv::GetDetectApriltagStatus>::SharedPtr status_srv_;
    rclcpp::Service<capella_ros_service_interfaces::srv::IsInChargerRange>::SharedPtr range_srv_;

    // 服务回调函数
    void startDetectApriltag(
        const std::shared_ptr<capella_ros_service_interfaces::srv::StartDetectApriltag::Request> req,
        std::shared_ptr<capella_ros_service_interfaces::srv::StartDetectApriltag::Response> res);
    void stopDetectApriltag(
        const std::shared_ptr<capella_ros_service_interfaces::srv::StopDetectApriltag::Request> req,
        std::shared_ptr<capella_ros_service_interfaces::srv::StopDetectApriltag::Response> res);
    void getDetectApriltagStatus(
        const std::shared_ptr<capella_ros_service_interfaces::srv::GetDetectApriltagStatus::Request> req,
        std::shared_ptr<capella_ros_service_interfaces::srv::GetDetectApriltagStatus::Response> res);
    void isInChargerRange(
        const std::shared_ptr<capella_ros_service_interfaces::srv::IsInChargerRange::Request> req,
        std::shared_ptr<capella_ros_service_interfaces::srv::IsInChargerRange::Response> res);

    // 定时器回调：处理图像
    void processImageCallback();

    void updateRectCorners();
    void publishRectMarker();

    void start_img_and_info_sub();
    void stop_img_and_info_sub();

    bool getOneImageAndInfo(
        const std::string& image_topic,          // 实际图像话题名
        const std::string& camera_info_topic,    // 实际相机信息话题名
        sensor_msgs::msg::Image::ConstSharedPtr& img,
        sensor_msgs::msg::CameraInfo::ConstSharedPtr& ci,
        const std::chrono::seconds timeout);

    // ========== [pose_opt] 精度优化：ESF / 联合 PnP / SE(2) 滤波 ==========
    bool enable_esf_refine_{true};
    bool enable_joint_pnp_{true};
    bool enable_pose_filter_{true};
    double pose_rms_threshold_{1.5};
    double pose_hold_sec_{0.3};
    double pose_gate_xy_{0.05};
    double pose_gate_yaw_{0.087};
    double pose_filter_alpha_max_{0.4};

    bool pose_filter_inited_{false};
    bool last_good_pose_valid_{false};
    double filt_x_{0.0};
    double filt_y_{0.0};
    double filt_yaw_{0.0};
    double filt_vx_{0.0};
    double filt_vy_{0.0};
    double filt_omega_{0.0};
    rclcpp::Time filt_stamp_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_good_pose_stamp_{0, 0, RCL_ROS_TIME};
    tf2::Transform last_tf_charger_to_baselink_dummy_;

    int frame_joint_ok_{0};
    int frame_joint_fail_{0};
    int frame_filter_hold_{0};
    int frame_filter_gate_{0};

    static double wrapPi(double a);
    void resetPoseFilter();
    bool solveJointTagPnP(apriltag_detection_t* det1,
                          apriltag_detection_t* det2,
                          const std::array<double, 4>& intr,
                          double tagsize,
                          tf2::Transform& tf_cam_to_m1,
                          double& rms);
    bool updatePoseFilter(const rclcpp::Time& stamp,
                          double x_meas,
                          double y_meas,
                          double yaw_meas,
                          double rms,
                          bool measurement_valid,
                          double& x_out,
                          double& y_out,
                          double& yaw_out,
                          std::string& dbg);
    static tf2::Transform replaceSe2(const tf2::Transform& src, double x, double y, double yaw);
};

RCLCPP_COMPONENTS_REGISTER_NODE(AprilTagDoubleNode)

AprilTagDoubleNode::AprilTagDoubleNode(const rclcpp::NodeOptions& options)
  : Node("apriltag", options),
    cb_parameter(add_on_set_parameters_callback(std::bind(&AprilTagDoubleNode::onParameter, this, std::placeholders::_1))),
    td(apriltag_detector_create()),
    pub_detections(create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>("detections", rclcpp::QoS(1))),
    tf_broadcaster(this)
{
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 创建静态广播器
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // 参数声明（原有部分省略，保留不变）
    apriltag_family_name = declare_parameter("family", "36h11", descr("tag family", true));
    tag_edge_size = declare_parameter("size", 1.0, descr("default tag size", true));
    const auto ids = declare_parameter("tag.ids", std::vector<int64_t>{}, descr("tag ids", true));
    const auto frames = declare_parameter("tag.frames", std::vector<std::string>{}, descr("tag frame names per id", true));
    const auto sizes = declare_parameter("tag.sizes", std::vector<double>{}, descr("tag sizes per id", true));
    estimate_pose = pose_estimation_methods.at(declare_parameter("pose_estimation_method", "pnp", descr("pose estimation method: \"pnp\" (more accurate) or \"homography\" (faster)", true)));
    declare_parameter("detector.threads", td->nthreads, descr("number of threads"));
    declare_parameter("detector.decimate", td->quad_decimate, descr("decimate resolution for quad detection"));
    declare_parameter("detector.blur", td->quad_sigma, descr("sigma of Gaussian blur for quad detection"));
    declare_parameter("detector.refine", td->refine_edges, descr("snap to strong gradients"));
    declare_parameter("detector.sharpening", td->decode_sharpening, descr("sharpening of decoded images"));
    declare_parameter("detector.debug", td->debug, descr("write additional debugging images to working directory"));
    // [pose_opt] declare 不会保证把 yaml 覆盖值写进 td，这里显式读回
    td->nthreads = get_parameter("detector.threads").as_int();
    td->quad_decimate = static_cast<float>(get_parameter("detector.decimate").as_double());
    td->quad_sigma = static_cast<float>(get_parameter("detector.blur").as_double());
    td->refine_edges = get_parameter("detector.refine").as_bool();
    td->decode_sharpening = get_parameter("detector.sharpening").as_double();
    td->debug = get_parameter("detector.debug").as_bool();
    RCLCPP_INFO(get_logger(),
                "[pose_opt] detector threads=%d decimate=%.2f blur=%.2f refine=%d sharpening=%.2f",
                td->nthreads, td->quad_decimate, td->quad_sigma, td->refine_edges ? 1 : 0, td->decode_sharpening);
    declare_parameter("max_hamming", 0, descr("reject detections with more corrected bits than allowed"));
    declare_parameter("profile", false, descr("print profiling information to stdout"));
    declare_parameter("marker_id_and_bluetooth_mac_vec", std::vector<std::string>(), descr("the vector of marker id and bluetooth mac"));
    declare_parameter("marker_frame_translation", -0.04, descr("The translation distance on Y axis from charge frame to marker1 frame."));
    declare_parameter("similarity_threshold", 0.96, descr("similarity_threshold to check tf's validity"));
    declare_parameter("radius_threshold", 0.013, descr("radius_threshold to check tf's validity"));
    declare_parameter("base_link_dummy_transform_x", -0.374, descr("base_link_dummy_transform_x"));
    declare_parameter("base_link_dummy_transform_y", 0.0, descr("base_link_dummy_transform_y"));
    declare_parameter("base_link_dummy_transform_z", 0.50, descr("base_link_dummy_transform_z"));

    // 新增充电桩范围参数声明
    declare_parameter("pose_x_min", -0.85);
    declare_parameter("pose_x_max", -0.1);
    declare_parameter("pose_y_min", -0.3);
    declare_parameter("pose_y_max", 0.3);
    declare_parameter("yaw_min", -3.14);
    declare_parameter("yaw_max", 3.14);

    this->get_parameter_or<float>("pose_x_min", pose_x_min_, -0.85);
    this->get_parameter_or<float>("pose_x_max", pose_x_max_, -0.1);
    this->get_parameter_or<float>("pose_y_min", pose_y_min_, -0.3);
    this->get_parameter_or<float>("pose_y_max", pose_y_max_, 0.3);
    this->get_parameter_or<float>("yaw_min", yaw_min_, -3.14);
    this->get_parameter_or<float>("yaw_max", yaw_max_, 3.14);

    // 原有参数获取（保持不变）
    this->get_parameter_or<std::vector<std::string>>("marker_id_and_bluetooth_mac_vec", marker_id_and_bluetooth_mac_vector, {"0:1/94:C9:60:43:BE:01"});
    this->get_parameter_or<float>("marker_frame_translation", marker_frame_translation, -0.04);
    this->get_parameter_or<float>("similarity_threshold", similarity_threshold, 0.96);
    this->get_parameter_or<float>("radius_threshold", radius_threshold, 0.014);
    this->get_parameter_or<float>("base_link_dummy_transform_x", base_link_dummy_transform_x, -0.374);
    this->get_parameter_or<float>("base_link_dummy_transform_y", base_link_dummy_transform_y, 0.0);
    this->get_parameter_or<float>("base_link_dummy_transform_z", base_link_dummy_transform_z, 0.50);

    // [pose_opt] 精度优化参数（可用 cfg/params.yaml 或运行时覆盖）
    this->declare_parameter("enable_esf_refine", true);
    this->declare_parameter("enable_joint_pnp", true);
    this->declare_parameter("enable_pose_filter", true);
    this->declare_parameter("pose_rms_threshold", 1.5);
    this->declare_parameter("pose_hold_sec", 0.3);
    this->declare_parameter("pose_gate_xy", 0.05);
    this->declare_parameter("pose_gate_yaw", 0.087);
    this->declare_parameter("pose_filter_alpha_max", 0.4);
    this->get_parameter("enable_esf_refine", enable_esf_refine_);
    this->get_parameter("enable_joint_pnp", enable_joint_pnp_);
    this->get_parameter("enable_pose_filter", enable_pose_filter_);
    this->get_parameter("pose_rms_threshold", pose_rms_threshold_);
    this->get_parameter("pose_hold_sec", pose_hold_sec_);
    this->get_parameter("pose_gate_xy", pose_gate_xy_);
    this->get_parameter("pose_gate_yaw", pose_gate_yaw_);
    this->get_parameter("pose_filter_alpha_max", pose_filter_alpha_max_);
    RCLCPP_INFO(get_logger(),
                "[pose_opt] esf=%d joint_pnp=%d filter=%d rms_th=%.2f hold=%.2fs gate_xy=%.3f gate_yaw=%.3f alpha_max=%.2f",
                enable_esf_refine_ ? 1 : 0, enable_joint_pnp_ ? 1 : 0, enable_pose_filter_ ? 1 : 0,
                pose_rms_threshold_, pose_hold_sec_, pose_gate_xy_, pose_gate_yaw_, pose_filter_alpha_max_);

    last_tf_charger_to_baselink_dummy_.setIdentity();

    // 解析 marker_id_and_bluetooth_mac_vector
    int id_mac_length = marker_id_and_bluetooth_mac_vector.size();
    RCLCPP_INFO(get_logger(), "marker_id_and_bluetooth_mac_vector size: %d", id_mac_length);
    for (int ids_index = 0; ids_index < id_mac_length; ids_index++)
    {
        std::string id_and_mac = marker_id_and_bluetooth_mac_vector[ids_index];
        int id_and_mac_length = id_and_mac.length();
        int pos = id_and_mac.find('/');
        int marker_id_, marker_id_correction_;
        std::string bluetooth_mac;
        std::string marker_id_strings = id_and_mac.substr(0, pos);
        int marker_id_strings_length = marker_id_strings.length();
        int pos2 = marker_id_strings.find(":");
        marker_id_ = atoi(marker_id_strings.substr(0, pos2).c_str());
        marker_id_correction_ = atoi(marker_id_strings.substr(pos2 + 1, marker_id_strings_length - pos2 - 1).c_str());
        bluetooth_mac = id_and_mac.substr(pos + 1, id_and_mac_length - pos -1);
        RCLCPP_INFO(this->get_logger(), "marker_id: %d", marker_id_);
        RCLCPP_INFO(this->get_logger(), "marker_id_correction: %d", marker_id_correction_);
        RCLCPP_INFO(this->get_logger(), "bluetooth_mac: %s", bluetooth_mac.c_str());
        msg.marker_id = marker_id_;
        msg.marker_id_correction = marker_id_correction_;
        msg.bluetooth_mac = bluetooth_mac;
        msgs.marker_and_mac_vector.push_back(msg);
    }

    if(!frames.empty()) {
        if(ids.size() != frames.size()) {
            throw std::runtime_error("Number of tag ids (" + std::to_string(ids.size()) + ") and frames (" + std::to_string(frames.size()) + ") mismatch!");
        }
        for(size_t i = 0; i < ids.size(); i++) { tag_frames[ids[i]] = frames[i]; }
    }
    if(!sizes.empty()) {
        if(ids.size() != sizes.size()) {
            throw std::runtime_error("Number of tag ids (" + std::to_string(ids.size()) + ") and sizes (" + std::to_string(sizes.size()) + ") mismatch!");
        }
        for(size_t i = 0; i < ids.size(); i++) { tag_sizes[ids[i]] = sizes[i]; }
    }
    if(tag_fun.count(apriltag_family_name)) {
        tf = tag_fun.at(apriltag_family_name).first();
        tf_destructor = tag_fun.at(apriltag_family_name).second;
        apriltag_detector_add_family(td, tf);
    }
    else {
        throw std::runtime_error("Unsupported tag family: " + apriltag_family_name);
    }

    tf_real_to_dummy.setIdentity();
    tf2::Quaternion q_marker_real_to_dummy;
    q_marker_real_to_dummy.setRPY(-M_PI / 2.0, M_PI / 2.0, 0.0);
    tf_real_to_dummy.setRotation(q_marker_real_to_dummy);

    // 计算并发布 base_link 到 base_link_dummy 的静态变换
    // 创建base_link到base_link_dummy的静态变换
    tf_base_link_to_dummy_base_link.setIdentity();
    tf_base_link_to_dummy_base_link.setOrigin(tf2::Vector3(base_link_dummy_transform_x, base_link_dummy_transform_y, base_link_dummy_transform_z));
    tf2::Quaternion q_base_link_to_dummy_base_link;
    q_base_link_to_dummy_base_link.setRPY(0.0, 0.0, M_PI);
    tf_base_link_to_dummy_base_link.setRotation(q_base_link_to_dummy_base_link);

    geometry_msgs::msg::TransformStamped tf_baselink_to_baselink_dummy_msg;
    tf_baselink_to_baselink_dummy_msg.header.frame_id = std::string("base_link");
    tf_baselink_to_baselink_dummy_msg.header.stamp = rclcpp::Time(0); // 使用0表示静态变换
    tf_baselink_to_baselink_dummy_msg.child_frame_id = std::string("base_link_dummy");
    tf2::toMsg(tf_base_link_to_dummy_base_link, tf_baselink_to_baselink_dummy_msg.transform);
    // 发送静态变换
    static_broadcaster_->sendTransform(tf_baselink_to_baselink_dummy_msg);

    tf_marker1_to_charger.setIdentity();
    RCLCPP_INFO(get_logger(), "marker_frame_translation: %f", marker_frame_translation);
    tf_marker1_to_charger.setOrigin(tf2::Vector3(0., marker_frame_translation, 0.));
    tf2::Quaternion q_marker_to_charger;
    q_marker_to_charger.setRPY(0., 0., 0.);
    tf_marker1_to_charger.setRotation(q_marker_to_charger);

    tf_marker1_to_marker2_fixed.setIdentity();
    tf_marker1_to_marker2_fixed.setOrigin(tf2::Vector3(0., marker_frame_translation * 2, 0.));
    tf2::Quaternion q_marker1_to_marker2;
    q_marker1_to_marker2.setRPY(0., 0., 0.);
    tf_marker1_to_marker2_fixed.setRotation(q_marker1_to_marker2);

    pose_with_id_pub = this->create_publisher<aruco_msgs::msg::PoseWithId>("/pose_with_id", 100);
    detect_status = this->create_publisher<capella_ros_service_interfaces::msg::ChargeMarkerVisible>("marker_visible", rclcpp::QoS(1).reliable().transient_local());
    id_and_mac_pub = this->create_publisher<aruco_msgs::msg::MarkerAndMacVector>("/id_mac", rclcpp::QoS(1).reliable().transient_local());
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("charger_range_marker", rclcpp::QoS(1).reliable().transient_local());

    charger_id_sub = this->create_subscription<std_msgs::msg::String>("/charger/id", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local().reliable(),
                                                                      std::bind(&AprilTagDoubleNode::charger_id_callback, this, std::placeholders::_1));

    // ========== 新增：创建处理定时器（周期50ms） ==========
    process_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&AprilTagDoubleNode::processImageCallback, this));

    // ========== 新增：创建服务 ==========
    start_srv_ = this->create_service<capella_ros_service_interfaces::srv::StartDetectApriltag>(
        "/start_detect_apriltag",
        std::bind(&AprilTagDoubleNode::startDetectApriltag, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default);
    stop_srv_ = this->create_service<capella_ros_service_interfaces::srv::StopDetectApriltag>(
        "/stop_detect_apriltag",
        std::bind(&AprilTagDoubleNode::stopDetectApriltag, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default);
    status_srv_ = this->create_service<capella_ros_service_interfaces::srv::GetDetectApriltagStatus>(
        "/get_detect_apriltag_status",
        std::bind(&AprilTagDoubleNode::getDetectApriltagStatus, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default);
    range_srv_ = this->create_service<capella_ros_service_interfaces::srv::IsInChargerRange>(
        "/is_in_charger_range",
        std::bind(&AprilTagDoubleNode::isInChargerRange, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default);

    updateRectCorners();

    // pub topic /marker_visible with value false for init.
    RCLCPP_INFO(get_logger(), "pub topic /marker_visible with value false for init.");
    marker_detect_status.marker_id = -1;
    marker_detect_status.marker_id_correction = -1;
    marker_detect_status.marker_visible = false;
    detect_status->publish(marker_detect_status);
    marker_visible_last = marker_detect_status.marker_visible;
    
    // pub /id_mac for init
    id_and_mac_pub->publish(msgs);

    RCLCPP_INFO(get_logger(), "AprilTagDoubleNode initialized. Detection is stopped by default.");
}

void AprilTagDoubleNode::start_img_and_info_sub()
{
    // 修改图像订阅回调：仅缓存图像，不直接处理
    image_sub = image_transport::create_subscription(
        this, "/image_rect", 
        [this](const sensor_msgs::msg::Image::ConstSharedPtr& img) {
            std::lock_guard<std::mutex> lock(img_mutex_);
            last_img_ = img;
            last_img_time_ = this->get_clock()->now();
            // 相机信息可能尚未到达，等待即可
            if (last_camera_info_) {
                last_ci_ = last_camera_info_;
            }
            // 记录接收时间（用于超时判断）
            last_time_camera_topic_received = this->get_clock()->now().seconds();
        },
        "raw", rmw_qos_profile_sensor_data
    );

    // 相机信息回调：持续更新
    info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera_info", rclcpp::QoS(1).best_effort(),
        [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info) {
            last_camera_info_ = info;
            // 同时更新缓存的相机信息
            std::lock_guard<std::mutex> lock(img_mutex_);
            last_ci_ = info;
        }
    );
}

void AprilTagDoubleNode::stop_img_and_info_sub()
{
    image_sub.shutdown();
    info_sub.reset();
}

void AprilTagDoubleNode::updateRectCorners()
{
    rect_corners_.clear();
    geometry_msgs::msg::Point p;
    p.z = 0.0;  // 矩形位于地面上

    // 四个角点顺序
    p.x = pose_x_min_; p.y = pose_y_min_; rect_corners_.push_back(p);
    p.x = pose_x_max_; p.y = pose_y_min_; rect_corners_.push_back(p);
    p.x = pose_x_max_; p.y = pose_y_max_; rect_corners_.push_back(p);
    p.x = pose_x_min_; p.y = pose_y_max_; rect_corners_.push_back(p);
    // 闭合矩形，再添加第一个点
    p.x = pose_x_min_; p.y = pose_y_min_; rect_corners_.push_back(p);
}

void AprilTagDoubleNode::publishRectMarker()
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "charger";           // 相对于 charger 坐标系
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "charger_range";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.points = rect_corners_;
    marker.scale.x = 0.05;                        // 线宽
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker_pub_->publish(marker);
}

AprilTagDoubleNode::~AprilTagDoubleNode()
{
    apriltag_detector_destroy(td);
    tf_destructor(tf);
}

void AprilTagDoubleNode::charger_id_callback(std_msgs::msg::String msg)
{
    RCLCPP_INFO(this->get_logger(), "charger_id_callback");
    RCLCPP_INFO(this->get_logger(), "msgs.marker_and_mac_vector.size(): %ld", msgs.marker_and_mac_vector.size());
    for (size_t i = 0; i < msgs.marker_and_mac_vector.size(); i++)
    {
        RCLCPP_INFO(get_logger(), "Index %ld => marker_id: %ld, bluetooth_mac: %s", i, msgs.marker_and_mac_vector[i].marker_id, msgs.marker_and_mac_vector[i].bluetooth_mac.c_str());
    }
    charger_id_ = msg.data;
    if (charger_id_.compare("") == 0)
    {
        RCLCPP_INFO(get_logger(), "The topic /charger/id received is empty, set id_selected=false");
        id_selected = false;
    }
    else
    {   
        RCLCPP_INFO(get_logger(), "The topic /charger/id received is %s, set id_selected=true", charger_id_.c_str());
        id_selected = true;     
        RCLCPP_INFO(this->get_logger(), "/charger/id: %s", charger_id_.c_str());
        for (size_t i = 0; i < msgs.marker_and_mac_vector.size(); i++)
        {
            if(msg.data.compare(this->msgs.marker_and_mac_vector[i].bluetooth_mac) == 0)
            {                
                RCLCPP_INFO(this->get_logger(), "Found the charger/id: %s in marker_id_and_bluetooth_mac lists.", charger_id_.c_str());
                marker_id = msgs.marker_and_mac_vector[i].marker_id;
                marker_id_correction = msgs.marker_and_mac_vector[i].marker_id_correction;
                break;
            }
            else
            {
                if (i == msgs.marker_and_mac_vector.size() - 1)
                {
                    RCLCPP_INFO(get_logger(), "Not found the charger/id: %s in marker_id_and_bluetooth_mac lists. Please check the marker_id_and_bluetooth_mac environment in docker-compose.yml", charger_id_.c_str());
                }
            }
        }
    }
}

// ========== 新增：定时器处理回调 ==========
void AprilTagDoubleNode::processImageCallback()
{
    // 如果未开启检测，直接返回
    if (!detecting_apriltag_.load())
        return;

    // 获取缓存的图像和相机信息
    sensor_msgs::msg::Image::ConstSharedPtr img;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr ci;
    rclcpp::Time img_time;
    {
        std::lock_guard<std::mutex> lock(img_mutex_);
        if (!last_img_ || !last_ci_) {
            // 尚未收到图像或相机信息
            return;
        }
        img = last_img_;
        ci = last_ci_;
        img_time = last_img_time_;
    }

    // 检查图像新鲜度：与当前时间差小于1秒
    auto now_time = this->get_clock()->now();
    if ((now_time - img_time).seconds() > 1.0) {
        // 图像太旧，忽略
        RCLCPP_DEBUG(get_logger(), "Image too old, skip processing.");
        return;
    }

    // 调用实际处理函数
    onCamera(img, ci);
}

// ========== 新增服务实现 ==========
void AprilTagDoubleNode::startDetectApriltag(
    const std::shared_ptr<capella_ros_service_interfaces::srv::StartDetectApriltag::Request> /*req*/,
    std::shared_ptr<capella_ros_service_interfaces::srv::StartDetectApriltag::Response> res)
{
    RCLCPP_INFO(get_logger(), "StartDetectApriltag called, detection enabled.");
    start_img_and_info_sub();
    detecting_apriltag_ = true;
    res->success = true;
}

void AprilTagDoubleNode::stopDetectApriltag(
    const std::shared_ptr<capella_ros_service_interfaces::srv::StopDetectApriltag::Request> /*req*/,
    std::shared_ptr<capella_ros_service_interfaces::srv::StopDetectApriltag::Response> res)
{
    RCLCPP_INFO(get_logger(), "StopDetectApriltag called, detection disabled.");
    stop_img_and_info_sub();
    RCLCPP_INFO(get_logger(), "停止检测时，重置所有计数为0");
    frame_all = 0;
    frame_detected =0;
    frame_error = 0;
    detecting_apriltag_ = false;
    resetPoseFilter();
    // 停止检测时，将范围内标志重置为 false（因为不再更新）
    RCLCPP_INFO(get_logger(), "停止检测时，重置充电桩范围内标志为 false.");
    detecting_in_charger_range_ = false;
    res->success = true;
    // 重置marker_visible为false
    RCLCPP_INFO(get_logger(), "停止检测时, 重置/marker_visible为 false.");
    marker_detect_status.marker_visible = false;
    marker_detect_status.marker_id = -1;
    marker_detect_status.marker_id_correction = -1;
    detect_status->publish(marker_detect_status);
    marker_visible_last = marker_detect_status.marker_visible;
}

void AprilTagDoubleNode::getDetectApriltagStatus(
    const std::shared_ptr<capella_ros_service_interfaces::srv::GetDetectApriltagStatus::Request> /*req*/,
    std::shared_ptr<capella_ros_service_interfaces::srv::GetDetectApriltagStatus::Response> res)
{
    res->detecting_apriltag = detecting_apriltag_.load();
    RCLCPP_INFO(get_logger(), "GetDetectApriltagStatus called.");
}

// 在服务回调函数内调用此函数
bool AprilTagDoubleNode::getOneImageAndInfo(
    const std::string& image_topic,          
    const std::string& camera_info_topic,    
    sensor_msgs::msg::Image::ConstSharedPtr& img,
    sensor_msgs::msg::CameraInfo::ConstSharedPtr& ci,
    const std::chrono::seconds timeout)
{
    // 1. 创建独立的临时节点（不依赖当前节点）
    auto temp_node = std::make_shared<rclcpp::Node>("temp_image_listener");

    // 2. 用于存储收到的相机信息（在相机信息回调中更新）
    auto last_camera_info = std::make_shared<sensor_msgs::msg::CameraInfo::ConstSharedPtr>();

    // 3. 使用 promise/future 同步图像和相机信息
    std::promise<std::pair<
        sensor_msgs::msg::Image::ConstSharedPtr,
        sensor_msgs::msg::CameraInfo::ConstSharedPtr>> promise;
    auto future = promise.get_future();

    // 4. 创建临时图像订阅
    auto img_sub = image_transport::create_subscription(
        temp_node.get(), image_topic,
        [&promise, last_camera_info](const sensor_msgs::msg::Image::ConstSharedPtr& img_msg) {
            RCLCPP_INFO(rclcpp::get_logger("temp_image_listener"), "temp img_sub ");
            // 只有当相机信息也已收到时，才设置 promise
            if (*last_camera_info) {
                promise.set_value(std::make_pair(img_msg, *last_camera_info));
            }
        },
        "raw", rmw_qos_profile_sensor_data);

    // 5. 创建临时相机信息订阅
    auto info_sub = temp_node->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic, rclcpp::QoS(1).best_effort(),
        [last_camera_info](const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info_msg) {
            RCLCPP_INFO(rclcpp::get_logger("temp_image_listener"), "temp info_sub ");
            *last_camera_info = info_msg;
        });

    // 6. 创建独立的单线程执行器，只驱动临时节点
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(temp_node);

    // 7. 等待 future 完成（内部会驱动执行器处理回调，不会死锁）
    auto status = exec.spin_until_future_complete(future, timeout);

    if (status == rclcpp::FutureReturnCode::SUCCESS) {
        auto result = future.get();
        img = result.first;
        ci = result.second;
        RCLCPP_INFO(rclcpp::get_logger("temp_listener"), "image and camera info received.");
        return true;
    } else {
        RCLCPP_WARN(rclcpp::get_logger("temp_listener"), "Timeout waiting for image & camera info");
        return false;
    }
}

// 在服务的回调函数中使用
void AprilTagDoubleNode::isInChargerRange(
    const std::shared_ptr<capella_ros_service_interfaces::srv::IsInChargerRange::Request> /*req*/,
    std::shared_ptr<capella_ros_service_interfaces::srv::IsInChargerRange::Response> res)
{
    RCLCPP_INFO(get_logger(), "IsInChargerRange called.");

    // 获取经过 remapping 后的实际话题名
    std::string actual_image_topic = this->get_node_topics_interface()->resolve_topic_name("/image_rect");
    std::string actual_camera_info_topic = this->get_node_topics_interface()->resolve_topic_name("/camera_info");
    RCLCPP_INFO(get_logger(), "actual_image_topic name:%s ", actual_image_topic.c_str());
    RCLCPP_INFO(get_logger(), "actual_camera_info_topic name:%s ", actual_camera_info_topic.c_str());


    sensor_msgs::msg::Image::ConstSharedPtr img;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr ci;
    if (getOneImageAndInfo(actual_image_topic, actual_camera_info_topic, img, ci, std::chrono::seconds(2))) {
        last_time_camera_topic_received = this->get_clock()->now().seconds();
        onCamera(img, ci);
        res->is_in_range = detecting_in_charger_range_.load();
    } else {
        res->is_in_range = false;
    }
}

// ========== 修改 onCamera 函数，增加范围内判断 ==========
void AprilTagDoubleNode::onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img,
                            const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_ci)
{
    double now_time = this->get_clock()->now().seconds();
    if (now_time - last_time_camera_topic_received > 1.0)
    {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "image topic timeout.");
        if (marker_visible_last == true)
        {
            RCLCPP_INFO(get_logger(), "/marker_visible status change from true to false");
            marker_detect_status.marker_id = -1;
            marker_detect_status.marker_id_correction = -1;
            marker_detect_status.marker_visible = false;
            detect_status->publish(marker_detect_status);
            marker_visible_last = marker_detect_status.marker_visible;
        } 
        return;
    }
    // 记录相机话题接收时间（已在回调中记录，这里再记录一次以保一致）
    last_time_camera_topic_received = this->get_clock()->now().seconds();

    try
    {
        tf_camera_to_marker1.setIdentity();
        tf_camera_to_marker2.setIdentity();
        id_and_tf_vec = std::vector<std::pair<int, geometry_msgs::msg::TransformStamped>>();

        tf_baselink_to_camera.setIdentity();
        geometry_msgs::msg::TransformStamped stamped_tf_baselink_to_camera_msg;
        tf2::Stamped<tf2::Transform> stamped_tf_baselink_to_camera;
        if(getTransform(std::string("base_link"), msg_img->header.frame_id, stamped_tf_baselink_to_camera_msg))
        {
            tf2::fromMsg(stamped_tf_baselink_to_camera_msg, stamped_tf_baselink_to_camera);
            tf_baselink_to_camera = static_cast<tf2::Transform>(stamped_tf_baselink_to_camera);
        }

        const std::array<double, 4> intrinsics = {msg_ci->k.data()[0], msg_ci->k.data()[4], msg_ci->k.data()[2], msg_ci->k.data()[5]};

         // 详细打印图像信息（调试用）
        RCLCPP_DEBUG(get_logger(), "Image: %dx%d, step=%u, encoding=%s",
                    msg_img->width, msg_img->height, msg_img->step, msg_img->encoding.c_str());

        // 严格检查图像有效性
        if (!msg_img || msg_img->width == 0 || msg_img->height == 0) {
            RCLCPP_ERROR(get_logger(), "Invalid image dimensions");
            detecting_in_charger_range_ = false;
            return;
        }
        // 检查 step
        int channels = (msg_img->encoding == "mono8") ? 1 : 3;
        if (msg_img->step < msg_img->width * channels) {
            RCLCPP_ERROR(get_logger(), "Invalid step: %u < %d", msg_img->step, msg_img->width * channels);
            detecting_in_charger_range_ = false;
            return;
        }

        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg_img);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge conversion failed: %s", e.what());
            detecting_in_charger_range_ = false;
            return;
        }

        cv::Mat img_color = cv_ptr->image;
        cv::Mat img_uint8;
        if (cv_ptr->encoding == "mono8") {
            img_uint8 = img_color;
        } else if (cv_ptr->encoding == "bgr8") {
            cv::cvtColor(img_color, img_uint8, cv::COLOR_BGR2GRAY);
        } else if (cv_ptr->encoding == "rgb8") {
            // [pose_opt] 原代码 rgb8 误用 COLOR_BGR2GRAY，会损伤边缘梯度
            cv::cvtColor(img_color, img_uint8, cv::COLOR_RGB2GRAY);
        } else {
            RCLCPP_ERROR(get_logger(), "Unsupported encoding: %s", cv_ptr->encoding.c_str());
            detecting_in_charger_range_ = false;
            return;
        }

        // [pose_opt] stride 使用 step1()，避免非连续 Mat 把检测器读偏
        image_u8_t im{img_uint8.cols, img_uint8.rows, static_cast<int>(img_uint8.step1()), img_uint8.data};

        mutex.lock();
        double start_time = this->now().seconds();
        auto detections = apriltag_detector_detect(td, &im);
        double end_time = this->now().seconds();
        RCLCPP_DEBUG(get_logger(), "compute detections cost time: %d ms", (int)round((end_time - start_time) * 1000));
        mutex.unlock();

        if(profile)
            timeprofile_display(td->tp);

        apriltag_msgs::msg::AprilTagDetectionArray msg_detections;
        msg_detections.header = msg_img->header;
        std::vector<geometry_msgs::msg::TransformStamped> tfs;

        for(int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t* det;
            zarray_get(detections, i, &det);

            RCLCPP_DEBUG(get_logger(),
                         "detection %3d: id (%2dx%2d)-%-4d, hamming %d, margin %8.3f\n",
                         i, det->family->nbits, det->family->h, det->id,
                         det->hamming, det->decision_margin);

            if(!tag_frames.empty() && !tag_frames.count(det->id)) { continue; }
            if(det->hamming > max_hamming) { continue; }

            // [pose_opt 阶段3] Sigmoid ESF 亚像素修边，写回 det->p 后再 PnP
            if (enable_esf_refine_) {
                const double p_before[4][2] = {
                    {det->p[0][0], det->p[0][1]},
                    {det->p[1][0], det->p[1][1]},
                    {det->p[2][0], det->p[2][1]},
                    {det->p[3][0], det->p[3][1]},
                };
                const CornerRefineResult esf = refine_corners_esf(img_uint8, det->p);
                RCLCPP_DEBUG(get_logger(),
                             "[pose_opt] esf id=%d updated=%d edges=%d samples=%d/%d "
                             "dp0=(%.3f,%.3f) dp1=(%.3f,%.3f) dp2=(%.3f,%.3f) dp3=(%.3f,%.3f)",
                             det->id, esf.updated ? 1 : 0, esf.n_edge_ok, esf.n_sample_ok, esf.n_sample_all,
                             det->p[0][0] - p_before[0][0], det->p[0][1] - p_before[0][1],
                             det->p[1][0] - p_before[1][0], det->p[1][1] - p_before[1][1],
                             det->p[2][0] - p_before[2][0], det->p[2][1] - p_before[2][1],
                             det->p[3][0] - p_before[3][0], det->p[3][1] - p_before[3][1]);
            }

            apriltag_msgs::msg::AprilTagDetection msg_detection;
            msg_detection.family = std::string(det->family->name);
            msg_detection.id = det->id;
            msg_detection.hamming = det->hamming;
            msg_detection.decision_margin = det->decision_margin;
            msg_detection.centre.x = det->c[0];
            msg_detection.centre.y = det->c[1];
            std::memcpy(msg_detection.corners.data(), det->p, sizeof(double) * 8);
            std::memcpy(msg_detection.homography.data(), det->H->data, sizeof(double) * 9);
            msg_detections.detections.push_back(msg_detection);

            geometry_msgs::msg::TransformStamped stampedTransform_real_to_dummy;
            std::stringstream ss_parent, ss_child;
            ss_parent << "april" << det->family->name << ":" << det->id;
            ss_child << "april" << det->family->name << ":" << det->id << "_dummy";
            stampedTransform_real_to_dummy.header.frame_id = ss_parent.str();
            stampedTransform_real_to_dummy.header.stamp = msg_img->header.stamp;
            stampedTransform_real_to_dummy.child_frame_id = ss_child.str();
            tf2::toMsg(tf_real_to_dummy, stampedTransform_real_to_dummy.transform);

            geometry_msgs::msg::TransformStamped tf;
            tf.header = msg_img->header;
            tf.child_frame_id = tag_frames.count(det->id) ? tag_frames.at(det->id) : "april" + std::string(det->family->name) + ":" + std::to_string(det->id);
            const double size = tag_sizes.count(det->id) ? tag_sizes.at(det->id) : tag_edge_size;
            if(estimate_pose != nullptr) {
                tf.transform = estimate_pose(det, intrinsics, size, shared_from_this());
            }
            
            std::pair<int, geometry_msgs::msg::TransformStamped> id_tf_pair;
            id_tf_pair.first = det->id;
            id_tf_pair.second = tf;
            id_and_tf_vec.push_back(id_tf_pair);
            tfs.push_back(stampedTransform_real_to_dummy); // marker1_real_to_dummy, marker2_real_to_dummy ...
        }

        int detections_size = zarray_size(detections);
        frame_all++;
        if (detections_size < 2)
        {
            frame_not_detected++;
            marker_detect_status.marker_visible = false;
            marker_detect_status.marker_id = -1;
            marker_detect_status.marker_id_correction = -1;
        }
        else
        {
            auto marker_id_vector = std::vector<int>();
            for (int i = 0; i < detections_size; i++)
            {            
                apriltag_detection_t* det;
                zarray_get(detections, i, &det);
                if (det->hamming <= max_hamming.load())
                {
                    marker_id_vector.push_back(det->id);
                }
            }
            if (id_selected)
            {
                if (std::find(marker_id_vector.begin(), marker_id_vector.end(), marker_id) != marker_id_vector.end() && 
                    std::find(marker_id_vector.begin(), marker_id_vector.end(), marker_id_correction) != marker_id_vector.end())
                {
                    marker_detect_status.marker_visible = true;
                    marker_detect_status.marker_id = marker_id;
                    marker_detect_status.marker_id_correction = marker_id_correction;
                }
                else
                {
                    marker_detect_status.marker_visible = false;
                    marker_detect_status.marker_id = -1;
                    marker_detect_status.marker_id_correction = -1;
                }			
            }
            else
            {				
                if (in_idRanges(marker_id_vector))
                {
                    marker_detect_status.marker_visible = true;
                    marker_detect_status.marker_id = marker_id;
                    marker_detect_status.marker_id_correction = marker_id_correction;
                }
                else
                {
                    marker_detect_status.marker_visible = false;
                    marker_detect_status.marker_id = -1;
                    marker_detect_status.marker_id_correction = -1;
                }	
            }
        }
        
        // 默认不在范围内，只有通过验证且位姿在阈值内才设为 true
        bool in_range = false;
        const rclcpp::Time img_stamp(msg_img->header.stamp);
        bool measurement_valid = false;
        tf2::Transform tf_charger_to_baselink_dummy;
        tf_charger_to_baselink_dummy.setIdentity();
        float similarity = 0.0f;
        float error_radius = 1e9f;
        double joint_rms = 1e9;
        bool used_joint_pnp = false;

        if (marker_detect_status.marker_visible)
        {
            frame_detected++;
            geometry_msgs::msg::TransformStamped stampedTransform_marker1_to_charger;
            std::stringstream ss_marker1_frame;
            ss_marker1_frame << "apriltag" << apriltag_family_name << ":" << marker_id << "_dummy";
            stampedTransform_marker1_to_charger.header.frame_id = ss_marker1_frame.str();
            stampedTransform_marker1_to_charger.header.stamp = msg_img->header.stamp;
            stampedTransform_marker1_to_charger.child_frame_id = std::string("charger");
            tf2::toMsg(tf_marker1_to_charger, stampedTransform_marker1_to_charger.transform);
            tfs.push_back(stampedTransform_marker1_to_charger); // marker1_dummy_to_charger

            size_t tf_size = id_and_tf_vec.size();
            int index_marker1 = 0, index_marker2 = 0;
            for (size_t i = 0; i < tf_size; i++)
            {
                if (id_and_tf_vec[i].first == marker_id)
                {
                    index_marker1 = i;
                    tf2::fromMsg(id_and_tf_vec[i].second, stamped_tf_camera_to_marker1);
                    tf_camera_to_marker1 = static_cast<tf2::Transform>(stamped_tf_camera_to_marker1);
                }
                if (id_and_tf_vec[i].first == marker_id_correction)
                {
                    index_marker2 = i;
                    tf2::fromMsg(id_and_tf_vec[i].second, stamped_tf_camera_to_marker2);
                    tf_camera_to_marker2 = static_cast<tf2::Transform>(stamped_tf_camera_to_marker2);
                }
            }

            tf2::Transform tf_marker1_to_marker2_current;
            tf_marker1_to_marker2_current = tf_real_to_dummy.inverse() * tf_camera_to_marker1.inverse() * tf_camera_to_marker2 * tf_real_to_dummy;

            auto tf_fixed_to_current = tf_marker1_to_marker2_fixed.inverse() * tf_marker1_to_marker2_current;
            float error_x, error_y, error_z;
            error_x = tf_fixed_to_current.getOrigin()[0];
            error_y = tf_fixed_to_current.getOrigin()[1];
            error_z = tf_fixed_to_current.getOrigin()[2];
            error_radius = std::hypot(std::hypot(error_x, error_y), error_z);

            auto q_f = tf_marker1_to_marker2_fixed.getRotation();
            auto q_c = tf_marker1_to_marker2_current.getRotation();
            similarity = std::fabs(q_f.getW()*q_c.getW() + q_f.getX()*q_c.getX() + q_f.getY()*q_c.getY() + q_f.getZ()*q_c.getZ());

            RCLCPP_DEBUG(get_logger(),
                         "[pose_opt] dual-check similarity=%.5f (th=%.5f) radius=%.5f (th=%.5f)",
                         similarity, similarity_threshold, error_radius, radius_threshold);

            // [pose_opt 阶段2] 双标签 8 点刚体 PnP，用安装约束一次性求解 cam->marker1
            apriltag_detection_t* det1 = nullptr;
            apriltag_detection_t* det2 = nullptr;
            for (int i = 0; i < detections_size; ++i) {
                apriltag_detection_t* det = nullptr;
                zarray_get(detections, i, &det);
                if (det->id == marker_id) {
                    det1 = det;
                }
                if (det->id == marker_id_correction) {
                    det2 = det;
                }
            }

            if (enable_joint_pnp_ && det1 && det2) {
                const double size = tag_sizes.count(marker_id) ? tag_sizes.at(marker_id) : tag_edge_size;
                tf2::Transform tf_joint;
                if (solveJointTagPnP(det1, det2, intrinsics, size, tf_joint, joint_rms)) {
                    RCLCPP_DEBUG(get_logger(), "[pose_opt] joint8 rms=%.3f px (th=%.3f)", joint_rms, pose_rms_threshold_);
                    if (joint_rms < pose_rms_threshold_) {
                        tf_camera_to_marker1 = tf_joint;
                        used_joint_pnp = true;
                        measurement_valid = true;
                        frame_joint_ok_++;
                        tf2::toMsg(tf_camera_to_marker1, id_and_tf_vec[index_marker1].second.transform);
                    } else {
                        frame_joint_fail_++;
                        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                                             "[pose_opt] joint8 rms too large: %.3f >= %.3f, fallback to independent PnP gate",
                                             joint_rms, pose_rms_threshold_);
                    }
                } else {
                    frame_joint_fail_++;
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "[pose_opt] joint8 solvePnP failed, fallback");
                }
            }

            // 联合 PnP 未过门限时，回退到原来的双独立 PnP + similarity/radius 门控
            if (!measurement_valid) {
                if (similarity > similarity_threshold && error_radius < radius_threshold) {
                    measurement_valid = true;
                    RCLCPP_DEBUG(get_logger(), "[pose_opt] accepted by legacy similarity/radius gate");
                } else {
                    frame_error++;
                    if (similarity <= similarity_threshold)
                    {
                        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "error tf detected, similarity  : %f, threshold: %f", similarity, similarity_threshold);
                    }
                    if (error_radius >= radius_threshold)
                    {
                        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "error tf detected, error_raidus: %f, threshold: %f", error_radius, radius_threshold);
                    }
                }
            }

            if (measurement_valid) {
                tfs.push_back(id_and_tf_vec[index_marker1].second);   // depth_camera_to_marker1_real
                tfs.push_back(id_and_tf_vec[index_marker2].second);   // depth_camera_to_marker2_real
                tf_charger_to_baselink_dummy = tf_marker1_to_charger.inverse() * tf_real_to_dummy.inverse()
                    * tf_camera_to_marker1.inverse() * tf_baselink_to_camera.inverse() * tf_base_link_to_dummy_base_link;
            }
        }

        // [pose_opt 阶段4] 对 charger->base_link_dummy 的 (x,y,yaw) 做互补滤波 + 门控 + 短时 hold
        double x_f = 0.0, y_f = 0.0, yaw_f = 0.0;
        std::string filter_dbg = "none";
        const double x_meas = measurement_valid ? tf_charger_to_baselink_dummy.getOrigin().x() : 0.0;
        const double y_meas = measurement_valid ? tf_charger_to_baselink_dummy.getOrigin().y() : 0.0;
        const double yaw_meas = measurement_valid ? tf2::getYaw(tf_charger_to_baselink_dummy.getRotation()) : 0.0;
        const bool filter_ok = updatePoseFilter(img_stamp, x_meas, y_meas, yaw_meas,
                                                used_joint_pnp ? joint_rms : 0.5,
                                                measurement_valid, x_f, y_f, yaw_f, filter_dbg);

        if (filter_dbg == "hold_predict" || filter_dbg == "gate_reject_hold") {
            frame_filter_hold_++;
        }
        if (filter_dbg == "gate_reject" || filter_dbg == "gate_reject_hold") {
            frame_filter_gate_++;
        }

        if (filter_ok) {
            tf2::Transform tf_pub = measurement_valid ? tf_charger_to_baselink_dummy : last_tf_charger_to_baselink_dummy_;
            tf_pub = replaceSe2(tf_pub, x_f, y_f, yaw_f);
            last_tf_charger_to_baselink_dummy_ = tf_pub;

            aruco_msgs::msg::PoseWithId pose_with_id_msg;
            pose_with_id_msg.pose.header.stamp = msg_img->header.stamp;
            pose_with_id_msg.pose.header.frame_id = std::string("charger");
            pose_with_id_msg.marker_id = marker_id;
            pose_with_id_msg.similarity = similarity;
            pose_with_id_msg.radius = error_radius;
            tf2::toMsg(tf_pub, pose_with_id_msg.pose.pose);
            pose_with_id_pub->publish(pose_with_id_msg);
            publishRectMarker();

            RCLCPP_DEBUG(get_logger(),
                         "[pose_opt] pose pub src=%s joint=%d rms=%.3f meas=(%.4f,%.4f,%.4f) filt=(%.4f,%.4f,%.4f) dbg=%s",
                         measurement_valid ? "meas" : "hold", used_joint_pnp ? 1 : 0, joint_rms,
                         x_meas, y_meas, yaw_meas, x_f, y_f, yaw_f, filter_dbg.c_str());

            if (x_f > pose_x_min_ && x_f < pose_x_max_ &&
                y_f > pose_y_min_ && y_f < pose_y_max_ &&
                yaw_f > yaw_min_ && yaw_f < yaw_max_)
            {
                in_range = true;
            }

            // hold 期间维持 marker_visible，避免对接状态机因单帧丢检立刻退出
            if (!marker_detect_status.marker_visible && (filter_dbg == "hold_predict" || filter_dbg == "gate_reject_hold")) {
                marker_detect_status.marker_visible = true;
                marker_detect_status.marker_id = marker_id;
                marker_detect_status.marker_id_correction = marker_id_correction;
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                                     "[pose_opt] detection lost, holding pose (dbg=%s)", filter_dbg.c_str());
            }
        } else if (!measurement_valid) {
            RCLCPP_DEBUG(get_logger(), "[pose_opt] no pose this frame, filter_dbg=%s", filter_dbg.c_str());
        }

        if (marker_detect_status.marker_visible != marker_visible_last)
        {
            RCLCPP_INFO(get_logger(), "/marker_visible status change from %s to %s",
                marker_visible_last ? "true" : "false",
                marker_detect_status.marker_visible ? "true" : "false"
            );
            detect_status->publish(marker_detect_status);
            marker_visible_last = marker_detect_status.marker_visible;
        }

        detecting_in_charger_range_ = in_range;

        pub_detections->publish(msg_detections);
        tf_broadcaster.sendTransform(tfs);
        zarray_destroy(detections);
    }
    catch(const char* msg)
    {
        RCLCPP_INFO(get_logger(), "error: %s", msg);
        detecting_in_charger_range_ = false;
    }
    if (frame_detected > 0){
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000, "frame_detected_rate: %f", frame_detected / (float)frame_all);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000, "tf error rate: %f", frame_error / (float)frame_detected);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
                             "[pose_opt] joint_ok=%d joint_fail=%d hold=%d gate=%d",
                             frame_joint_ok_, frame_joint_fail_, frame_filter_hold_, frame_filter_gate_);
    }
    
}

double AprilTagDoubleNode::wrapPi(double a)
{
    return std::atan2(std::sin(a), std::cos(a));
}

void AprilTagDoubleNode::resetPoseFilter()
{
    pose_filter_inited_ = false;
    last_good_pose_valid_ = false;
    filt_x_ = 0.0;
    filt_y_ = 0.0;
    filt_yaw_ = 0.0;
    filt_vx_ = 0.0;
    filt_vy_ = 0.0;
    filt_omega_ = 0.0;
    last_tf_charger_to_baselink_dummy_.setIdentity();
    RCLCPP_INFO(get_logger(), "[pose_opt] pose filter reset");
}

tf2::Transform AprilTagDoubleNode::replaceSe2(const tf2::Transform& src, double x, double y, double yaw)
{
    tf2::Transform out = src;
    const tf2::Vector3 o = out.getOrigin();
    out.setOrigin(tf2::Vector3(x, y, o.z()));
    double roll = 0.0;
    double pitch = 0.0;
    double unused_yaw = 0.0;
    tf2::Matrix3x3(out.getRotation()).getRPY(roll, pitch, unused_yaw);
    (void)unused_yaw;
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    out.setRotation(q);
    return out;
}

bool AprilTagDoubleNode::solveJointTagPnP(apriltag_detection_t* det1,
                                          apriltag_detection_t* det2,
                                          const std::array<double, 4>& intr,
                                          double tagsize,
                                          tf2::Transform& tf_cam_to_m1,
                                          double& rms)
{
    const std::vector<cv::Point3d> obj1 = apriltag_object_points(tagsize);
    const std::vector<cv::Point2d> img1 = apriltag_image_points(det1);
    const std::vector<cv::Point3d> obj2_local = apriltag_object_points(tagsize);
    const std::vector<cv::Point2d> img2 = apriltag_image_points(det2);

    // dummy 系下 m1->m2 已知；变到 AprilTag real 系后再拼 8 个物体点
    // T_m1real_m2real = T_real_dummy * T_m1dummy_m2dummy * T_real_dummy^{-1}
    const tf2::Transform T_m1_m2_real =
        tf_real_to_dummy * tf_marker1_to_marker2_fixed * tf_real_to_dummy.inverse();

    std::vector<cv::Point3d> object_pts;
    std::vector<cv::Point2d> image_pts;
    object_pts.reserve(8);
    image_pts.reserve(8);
    object_pts.insert(object_pts.end(), obj1.begin(), obj1.end());
    image_pts.insert(image_pts.end(), img1.begin(), img1.end());
    for (size_t i = 0; i < obj2_local.size(); ++i) {
        const tf2::Vector3 p = T_m1_m2_real * tf2::Vector3(obj2_local[i].x, obj2_local[i].y, obj2_local[i].z);
        object_pts.emplace_back(p.x(), p.y(), p.z());
        image_pts.push_back(img2[i]);
    }

    const cv::Matx33d K = camera_matrix_from_intr(intr);
    cv::Mat rvec, tvec;
    if (!solve_pnp_ippe_then_iterative(object_pts, image_pts, K, rvec, tvec, get_logger(), "joint8")) {
        return false;
    }
    rms = reprojection_rms(object_pts, image_pts, K, rvec, tvec);
    tf2::fromMsg(tf_from_rt(tvec, rvec), tf_cam_to_m1);

    RCLCPP_DEBUG(get_logger(),
                 "[pose_opt] joint8 t=(%.4f,%.4f,%.4f) m2_in_m1real=(%.4f,%.4f,%.4f)",
                 tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2),
                 T_m1_m2_real.getOrigin().x(), T_m1_m2_real.getOrigin().y(), T_m1_m2_real.getOrigin().z());
    return true;
}

bool AprilTagDoubleNode::updatePoseFilter(const rclcpp::Time& stamp,
                                          double x_meas,
                                          double y_meas,
                                          double yaw_meas,
                                          double rms,
                                          bool measurement_valid,
                                          double& x_out,
                                          double& y_out,
                                          double& yaw_out,
                                          std::string& dbg)
{
    if (!enable_pose_filter_) {
        if (!measurement_valid) {
            dbg = "filter_disabled_no_meas";
            return false;
        }
        x_out = x_meas;
        y_out = y_meas;
        yaw_out = yaw_meas;
        last_good_pose_valid_ = true;
        last_good_pose_stamp_ = stamp;
        dbg = "filter_disabled";
        return true;
    }

    if (!measurement_valid) {
        if (!pose_filter_inited_ || !last_good_pose_valid_) {
            dbg = "hold_no_history";
            return false;
        }
        const double since_good = (stamp - last_good_pose_stamp_).seconds();
        if (since_good < 0.0 || since_good > pose_hold_sec_) {
            dbg = "hold_timeout";
            return false;
        }
        double dt = (stamp - filt_stamp_).seconds();
        if (dt < 0.0) {
            dt = 0.0;
        }
        dt = std::min(dt, pose_hold_sec_);
        x_out = filt_x_ + filt_vx_ * dt;
        y_out = filt_y_ + filt_vy_ * dt;
        yaw_out = wrapPi(filt_yaw_ + filt_omega_ * dt);
        filt_x_ = x_out;
        filt_y_ = y_out;
        filt_yaw_ = yaw_out;
        filt_stamp_ = stamp;
        dbg = "hold_predict";
        RCLCPP_DEBUG(get_logger(), "[pose_opt] filter HOLD dt=%.3f since_good=%.3f pose=(%.4f,%.4f,%.4f)",
                     dt, since_good, x_out, y_out, yaw_out);
        return true;
    }

    if (!pose_filter_inited_) {
        filt_x_ = x_meas;
        filt_y_ = y_meas;
        filt_yaw_ = yaw_meas;
        filt_vx_ = 0.0;
        filt_vy_ = 0.0;
        filt_omega_ = 0.0;
        filt_stamp_ = stamp;
        pose_filter_inited_ = true;
        last_good_pose_valid_ = true;
        last_good_pose_stamp_ = stamp;
        x_out = x_meas;
        y_out = y_meas;
        yaw_out = yaw_meas;
        dbg = "filter_init";
        RCLCPP_INFO(get_logger(), "[pose_opt] filter INIT meas=(%.4f,%.4f,%.4f) rms=%.3f",
                    x_meas, y_meas, yaw_meas, rms);
        return true;
    }

    double dt = (stamp - filt_stamp_).seconds();
    if (dt <= 1e-4 || dt > 1.0) {
        dt = 0.1;
    }

    const double x_pred = filt_x_ + filt_vx_ * dt;
    const double y_pred = filt_y_ + filt_vy_ * dt;
    const double yaw_pred = wrapPi(filt_yaw_ + filt_omega_ * dt);
    const double dx = x_meas - x_pred;
    const double dy = y_meas - y_pred;
    const double dyaw = wrapPi(yaw_meas - yaw_pred);
    const double dist = std::hypot(dx, dy);

    if (dist > pose_gate_xy_ || std::abs(dyaw) > pose_gate_yaw_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                             "[pose_opt] filter GATE reject dist=%.4f (th=%.4f) dyaw=%.4f (th=%.4f) rms=%.3f",
                             dist, pose_gate_xy_, dyaw, pose_gate_yaw_, rms);
        const double since_good = (stamp - last_good_pose_stamp_).seconds();
        if (last_good_pose_valid_ && since_good >= 0.0 && since_good <= pose_hold_sec_) {
            x_out = x_pred;
            y_out = y_pred;
            yaw_out = yaw_pred;
            filt_x_ = x_out;
            filt_y_ = y_out;
            filt_yaw_ = yaw_out;
            filt_stamp_ = stamp;
            dbg = "gate_reject_hold";
            return true;
        }
        dbg = "gate_reject";
        return false;
    }

    double alpha = pose_filter_alpha_max_ * std::exp(-std::max(0.0, rms) / 1.0);
    alpha = std::max(0.05, std::min(pose_filter_alpha_max_, alpha));

    x_out = alpha * x_meas + (1.0 - alpha) * x_pred;
    y_out = alpha * y_meas + (1.0 - alpha) * y_pred;
    yaw_out = wrapPi(yaw_pred + alpha * dyaw);

    const double vmax = 1.5;
    const double wmax = 1.0;
    filt_vx_ = std::max(-vmax, std::min(vmax, (x_out - filt_x_) / dt));
    filt_vy_ = std::max(-vmax, std::min(vmax, (y_out - filt_y_) / dt));
    filt_omega_ = std::max(-wmax, std::min(wmax, wrapPi(yaw_out - filt_yaw_) / dt));

    filt_x_ = x_out;
    filt_y_ = y_out;
    filt_yaw_ = yaw_out;
    filt_stamp_ = stamp;
    last_good_pose_valid_ = true;
    last_good_pose_stamp_ = stamp;
    dbg = "filter_fuse";
    RCLCPP_DEBUG(get_logger(),
                 "[pose_opt] filter FUSE dt=%.3f alpha=%.3f rms=%.3f meas=(%.4f,%.4f,%.4f) out=(%.4f,%.4f,%.4f)",
                 dt, alpha, rms, x_meas, y_meas, yaw_meas, x_out, y_out, yaw_out);
    return true;
}

bool AprilTagDoubleNode::in_idRanges(std::vector<int> ids)
{
    bool ret = false;
    for(size_t i = 0; i < msgs.marker_and_mac_vector.size(); i++)
    {
        marker_id = msgs.marker_and_mac_vector[i].marker_id;
        marker_id_correction = msgs.marker_and_mac_vector[i].marker_id_correction;
        if (std::find(ids.begin(), ids.end(), marker_id) != ids.end() && 
            std::find(ids.begin(), ids.end(), marker_id_correction) != ids.end())
        {
            ret = true;
            break;
        }
    }
    return ret;
}

rcl_interfaces::msg::SetParametersResult
AprilTagDoubleNode::onParameter(const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    mutex.lock();
    for(const rclcpp::Parameter& parameter : parameters) {
        RCLCPP_DEBUG_STREAM(get_logger(), "setting: " << parameter);
        IF("detector.threads", td->nthreads)
        IF("detector.decimate", td->quad_decimate)
        IF("detector.blur", td->quad_sigma)
        IF("detector.refine", td->refine_edges)
        IF("detector.sharpening", td->decode_sharpening)
        IF("detector.debug", td->debug)
        IF("max_hamming", max_hamming)
        IF("profile", profile)
        IF("enable_esf_refine", enable_esf_refine_)
        IF("enable_joint_pnp", enable_joint_pnp_)
        IF("enable_pose_filter", enable_pose_filter_)
        IF("pose_rms_threshold", pose_rms_threshold_)
        IF("pose_hold_sec", pose_hold_sec_)
        IF("pose_gate_xy", pose_gate_xy_)
        IF("pose_gate_yaw", pose_gate_yaw_)
        IF("pose_filter_alpha_max", pose_filter_alpha_max_)
    }
    mutex.unlock();
    result.successful = true;
    return result;
}

bool AprilTagDoubleNode::getTransform(
    const std::string & refFrame, const std::string & childFrame,
    geometry_msgs::msg::TransformStamped & transform)
{
    std::string errMsg;
    if (!tf_buffer_->canTransform(
            refFrame, childFrame, tf2::TimePointZero,
            tf2::durationFromSec(0.5), &errMsg))
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to get pose from TF: " << errMsg);
        return false;
    } else {
        try {
            transform = tf_buffer_->lookupTransform(
                refFrame, childFrame, tf2::TimePointZero, tf2::durationFromSec(0.5));
        } catch (const tf2::TransformException & e) {
            RCLCPP_ERROR_STREAM(
                this->get_logger(),
                "Error in lookupTransform of " << childFrame << " in " << refFrame << " : " << e.what());
            return false;
        }
    }
    return true;
}