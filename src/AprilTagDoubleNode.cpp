// ros
#include "pose_estimation.hpp"
#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#ifdef cv_bridge_HPP
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
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
        } else if (cv_ptr->encoding == "bgr8" || cv_ptr->encoding == "rgb8") {
            cv::cvtColor(img_color, img_uint8, cv::COLOR_BGR2GRAY);
        } else {
            RCLCPP_ERROR(get_logger(), "Unsupported encoding: %s", cv_ptr->encoding.c_str());
            detecting_in_charger_range_ = false;
            return;
        }

        image_u8_t im{img_uint8.cols, img_uint8.rows, img_uint8.cols, img_uint8.data};

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

        if (marker_detect_status.marker_visible != marker_visible_last)
        {
            RCLCPP_INFO(get_logger(), "/marker_visible status change from %s to %s", 
                marker_visible_last ? "true" : "false",
                marker_detect_status.marker_visible ? "true" : "false"
            );
            detect_status->publish(marker_detect_status);
            marker_visible_last = marker_detect_status.marker_visible;
        }

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
            
            // 调试信息（保留）
            // {
            //     {
            //         double x, y, theta;
            //         x = tf_marker1_to_marker2_fixed.getOrigin()[0];
            //         y = tf_marker1_to_marker2_fixed.getOrigin()[1];
            //         theta = tf2::getYaw(tf_marker1_to_marker2_fixed.getRotation());
            //         RCLCPP_INFO_ONCE(get_logger(), "x: %f, y: %f, theta: %f", x, y, theta);
            //     }
            //     {
            //         double x, y, theta;
            //         x = tf_marker1_to_marker2_current.getOrigin()[0];
            //         y = tf_marker1_to_marker2_current.getOrigin()[1];
            //         theta = tf2::getYaw(tf_marker1_to_marker2_current.getRotation());
            //         RCLCPP_INFO_ONCE(get_logger(), "x_c: %f, y_c: %f, theta_c: %f", x, y, theta);
            //     }
            //     {
            //         double x, y, theta;
            //         auto tf = tf_real_to_dummy.inverse() * tf_camera_to_marker1.inverse() * tf_baselink_to_camera.inverse() * tf_base_link_to_dummy_base_link;
            //         x = tf.getOrigin()[0];
            //         y = tf.getOrigin()[1];
            //         theta = tf2::getYaw(tf.getRotation());
            //         RCLCPP_INFO_ONCE(get_logger(), "x_1: %f, y_1: %f, theta_1: %f", x, y, theta);
            //     }
            //     {
            //         double x, y, theta;
            //         auto tf = tf_real_to_dummy.inverse() * tf_camera_to_marker2.inverse() * tf_baselink_to_camera.inverse() * tf_base_link_to_dummy_base_link;
            //         x = tf.getOrigin()[0];
            //         y = tf.getOrigin()[1];
            //         theta = tf2::getYaw(tf.getRotation());
            //         RCLCPP_INFO_ONCE(get_logger(), "x_2: %f, y_2: %f, theta_2: %f", x, y, theta);
            //     }
            // }

            auto tf_fixed_to_current = tf_marker1_to_marker2_fixed.inverse() * tf_marker1_to_marker2_current;
            float error_x, error_y, error_z;
            error_x = tf_fixed_to_current.getOrigin()[0];
            error_y = tf_fixed_to_current.getOrigin()[1];
            error_z = tf_fixed_to_current.getOrigin()[2];
            float error_radius = std::hypot(std::hypot(error_x, error_y), error_z);
            
            float w_f, x_f, y_f, z_f, w_c, x_c, y_c, z_c;
            auto q_f = tf_marker1_to_marker2_fixed.getRotation();
            auto q_c = tf_marker1_to_marker2_current.getRotation();
            w_f = q_f.getW();
            x_f = q_f.getX();
            y_f = q_f.getY();
            z_f = q_f.getZ();
            w_c = q_c.getW();
            x_c = q_c.getX();
            y_c = q_c.getY();
            z_c = q_c.getZ();
            float similarity = w_f * w_c + x_f * x_c + y_f * y_c + z_f * z_c;

            if (similarity > similarity_threshold && error_radius < radius_threshold)
            {
                tfs.push_back(id_and_tf_vec[index_marker1].second);   // depth_camera_to_marker1_real
                tfs.push_back(id_and_tf_vec[index_marker2].second);   // depth_camera_to_marker2_real

                auto end_time = this->get_clock()->now().seconds();
                auto start_time = rclcpp::Time(msg_img->header.stamp).seconds();
                auto delta_time = end_time - start_time;
                RCLCPP_DEBUG(get_logger(), "cost time: %f second.", delta_time);

                aruco_msgs::msg::PoseWithId pose_with_id_msg;
                pose_with_id_msg.pose.header.stamp = msg_img->header.stamp;
                pose_with_id_msg.pose.header.frame_id = std::string("charger");
                pose_with_id_msg.marker_id = marker_id;
                pose_with_id_msg.similarity = similarity;
                pose_with_id_msg.radius = error_radius;
                auto tf_charger_to_baselink_dummy = tf_marker1_to_charger.inverse() * tf_real_to_dummy.inverse()
                    * tf_camera_to_marker1.inverse() * tf_baselink_to_camera.inverse() * tf_base_link_to_dummy_base_link;
                tf2::toMsg(tf_charger_to_baselink_dummy, pose_with_id_msg.pose.pose);
                pose_with_id_pub->publish(pose_with_id_msg);

                // pub 充电桩范围内marker
                publishRectMarker();

                // ========== 判断是否在充电桩范围内 ==========
                // 提取相对于 base_link_dummy 的位姿
                double x = tf_charger_to_baselink_dummy.getOrigin().x();
                double y = tf_charger_to_baselink_dummy.getOrigin().y();
                double yaw = tf2::getYaw(tf_charger_to_baselink_dummy.getRotation());
                RCLCPP_DEBUG(get_logger(), "x: %.2f, y: %.2f, yaw: %.2f", x, y, yaw);
                RCLCPP_DEBUG(get_logger(), "pose_x_min_: %.2f, pose_x_max_: %.2f", pose_x_min_, pose_x_max_);
                RCLCPP_DEBUG(get_logger(), "pose_y_min_: %.2f, pose_y_max_: %.2f", pose_y_min_, pose_y_max_);
                RCLCPP_DEBUG(get_logger(), "yaw_min_: %.2f, yaw_max_: %.2f", yaw_min_, yaw_max_);
                if (x > pose_x_min_ && x < pose_x_max_ &&
                    y > pose_y_min_ && y < pose_y_max_ &&
                    yaw > yaw_min_ && yaw < yaw_max_)
                {
                    in_range = true;
                    RCLCPP_DEBUG(get_logger(), "in charger range");
                }
                else
                {
                    in_range = false;
                    RCLCPP_DEBUG(get_logger(), "not in charger range");
                }
            }
            else
            {
                frame_error++;
                if (similarity <= similarity_threshold)
                {
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "error tf detected, similarity  : %f, threshold: %f", similarity, similarity_threshold);
                }
                if (error_radius >= radius_threshold)
                {
                    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500, "error tf detected, error_raidus: %f, threshold: %f", error_radius, radius_threshold);
                }
                // 验证失败，不在范围内
                in_range = false;
            }
        }
        else
        {
            // 标记不可见，不在范围内
            in_range = false;
        }

        // 更新原子变量
        detecting_in_charger_range_ = in_range;

        // geometry_msgs::msg::TransformStamped tf_baselink_to_baselink_dummy_msg;
        // tf_baselink_to_baselink_dummy_msg.header.frame_id = std::string("base_link");
        // tf_baselink_to_baselink_dummy_msg.header.stamp = msg_img->header.stamp;
        // tf_baselink_to_baselink_dummy_msg.child_frame_id = std::string("base_link_dummy");
        // tf2::toMsg(tf_base_link_to_dummy_base_link, tf_baselink_to_baselink_dummy_msg.transform);
        // tfs.push_back(tf_baselink_to_baselink_dummy_msg); // base_link_to_base_link_dummy

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
    }
    
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