// ros
#include "pose_estimation.hpp"  // 包含位姿估计函数
#include <apriltag_msgs/msg/april_tag_detection.hpp>  // AprilTag检测消息
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>  // AprilTag检测数组消息
#ifdef cv_bridge_HPP
#include <cv_bridge/cv_bridge.hpp>  // ROS图像与OpenCV转换
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <image_transport/camera_subscriber.hpp>  // 图像传输订阅器
#include <image_transport/image_transport.hpp>    // 图像传输
#include <rclcpp/rclcpp.hpp>                      // ROS2核心
#include <rclcpp_components/register_node_macro.hpp> // 组件宏
#include <sensor_msgs/msg/camera_info.hpp>        // 相机内参消息
#include <sensor_msgs/msg/image.hpp>              // 图像消息
#include <tf2_ros/transform_broadcaster.h>        // TF广播器
#include "tf2_ros/buffer.h"                       // TF缓存
#include "tf2_ros/transform_listener.h"           // TF监听器
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // TF与Geometry消息转换
#include "tf2/utils.h"                            // TF实用函数
#include <sstream>                                // 字符串流
#include "aruco_msgs/msg/pose_with_id.hpp"        // 自定义位姿消息
#include "aruco_msgs/msg/marker_and_mac.hpp"      // 自定义标记与MAC消息
#include "aruco_msgs/msg/marker_and_mac_vector.hpp" // 自定义标记与MAC向量消息
#include "std_msgs/msg/string.hpp"                // 字符串消息
#include "capella_ros_service_interfaces/msg/charge_marker_visible.hpp" // 充电标记可见性消息
#include <math.h>                                 // 数学函数

// apriltag
#include "tag_functions.hpp"      // AprilTag族相关函数
#include <apriltag.h>             // AprilTag核心库
#include <angles/angles.h>        // 角度处理函数

// 宏定义，用于参数检查与赋值
#define IF(N, V) \
    if(assign_check(parameter, N, V)) continue;

// 模板函数：将参数赋值给变量
template<typename T>
void assign(const rclcpp::Parameter& parameter, T& var)
{
    var = parameter.get_value<T>();
}

// 原子类型的赋值特化
template<typename T>
void assign(const rclcpp::Parameter& parameter, std::atomic<T>& var)
{
    var = parameter.get_value<T>();
}

// 检查参数名并赋值
template<typename T>
bool assign_check(const rclcpp::Parameter& parameter, const std::string& name, T& var)
{
    if(parameter.get_name() == name) {
        assign(parameter, var);
        return true;
    }
    return false;
}

// 创建参数描述符
rcl_interfaces::msg::ParameterDescriptor
descr(const std::string& description, const bool& read_only = false)
{
    rcl_interfaces::msg::ParameterDescriptor descr;
    descr.description = description;
    descr.read_only = read_only;
    return descr;
}

/**
 * @brief AprilTag双标记节点类
 * 主要功能：
 * 1. 检测图像中的AprilTag
 * 2. 计算每个标记的位姿并发布TF
 * 3. 处理充电桩双标记的联合位姿验证和发布
 * 4. 发布标记可见性状态
 */
class AprilTagDoubleNode : public rclcpp::Node {
public:
    AprilTagDoubleNode(const rclcpp::NodeOptions& options);
    ~AprilTagDoubleNode() override;

private:
    const OnSetParametersCallbackHandle::SharedPtr cb_parameter;  // 参数回调句柄

    apriltag_family_t* tf;                // AprilTag族对象
    apriltag_detector_t* const td;        // AprilTag检测器对象

    // 参数相关（互斥锁保护）
    std::mutex mutex;
    double tag_edge_size;                 // 默认标记边长
    std::atomic<int> max_hamming;         // 最大汉明距离阈值
    std::atomic<bool> profile;            // 是否打印性能信息
    std::unordered_map<int, std::string> tag_frames;  // 标记ID到TF框架名的映射
    std::unordered_map<int, double> tag_sizes;        // 标记ID到边长的映射

    std::function<void(apriltag_family_t*)> tf_destructor; // 族对象析构函数

    // 图像和相机信息订阅
    image_transport::Subscriber image_sub;                 // 图像订阅器
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub; // 相机信息订阅器
    sensor_msgs::msg::CameraInfo::ConstSharedPtr last_camera_info_; // 最新相机信息缓存

    // 发布器
    const rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr pub_detections; // 检测结果发布
    tf2_ros::TransformBroadcaster tf_broadcaster; // TF广播器

    pose_estimation_f estimate_pose = nullptr; // 位姿估计方法函数指针

    // 图像回调函数
    void onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img,
                  const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_ci);

    // 参数变更回调
    rcl_interfaces::msg::SetParametersResult onParameter(const std::vector<rclcpp::Parameter>& parameters);

    // TF工具
    tf2::Transform tf_real_to_dummy;               // 真实标记到虚拟标记的变换（绕X轴-90°，绕Y轴90°）
    tf2::Transform tf_base_link_to_dummy_base_link; // base_link到虚拟base_link的变换（用于标记位姿）
    bool getTransform(const std::string & refFrame, const std::string & childFrame,
                      geometry_msgs::msg::TransformStamped & transform); // 获取TF变换
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_; // TF监听器
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;              // TF缓存

    // 自定义消息发布器
    rclcpp::Publisher<aruco_msgs::msg::PoseWithId>::SharedPtr pose_with_id_pub; // 发布标记位姿（带ID）
    rclcpp::Publisher<aruco_msgs::msg::MarkerAndMacVector>::SharedPtr id_and_mac_pub; // 发布标记ID与MAC对应关系
    rclcpp::Publisher<capella_ros_service_interfaces::msg::ChargeMarkerVisible>::SharedPtr detect_status; // 发布标记可见性

    // 定时器
    rclcpp::TimerBase::SharedPtr id_mac_timer_;     // 定期发布ID-MAC映射
    rclcpp::TimerBase::SharedPtr marker_timer;      // 定期发布标记可见性状态

    // 标记与MAC相关
    aruco_msgs::msg::MarkerAndMacVector msgs;                // 存储所有标记ID和MAC的向量
    aruco_msgs::msg::MarkerAndMac msg;                       // 单个标记ID与MAC
    std::string charger_id_;                                 // 当前充电器ID（蓝牙MAC）
    bool id_selected = false;                                // 是否已选择特定充电器
    std::vector<std::string> marker_id_and_bluetooth_mac_vector; // 配置的标记ID与MAC列表

    // 标记检测相关
    zarray_t detections;                     // 存储检测结果的数组
    int marker_id;                           // 当前充电器对应的主标记ID
    int marker_id_correction;                // 当前充电器对应的校正标记ID
    bool marker_visible_last = false;        // 上一帧标记可见性
    bool marker_visible_pub = false;         // 是否已发布过可见性（用于首次发布）
    float marker_frame_translation;          // 充电器坐标系到主标记坐标系的Y轴偏移
    capella_ros_service_interfaces::msg::ChargeMarkerVisible marker_detect_status; // 标记可见性消息
    std::string apriltag_family_name;        // AprilTag族名称（如"36h11"）

    // 回调函数
    void id_mac_callback();                  // 定期发布ID-MAC映射
    void charger_id_callback(std_msgs::msg::String msg); // 接收选择的充电器ID
    void marker_visible_callback();          // 定期发布标记可见性状态（处理超时）
    bool in_idRanges(std::vector<int> ids);  // 检查检测到的ID是否在配置的标记对中

    // 双标记验证相关
    tf2::Transform tf_marker1_to_charger;                     // 标记1到充电器坐标系的变换（固定）
    tf2::Transform tf_camera_to_marker1, tf_camera_to_marker2; // 相机到两个标记的变换
    tf2::Transform tf_marker1_to_marker2_fixed;               // 标记1到标记2的固定变换（由配置推导）
    tf2::Stamped<tf2::Transform> stamped_tf_camera_to_marker1; 
    tf2::Stamped<tf2::Transform> stamped_tf_camera_to_marker2; 
    std::vector<std::pair<int, geometry_msgs::msg::TransformStamped>> id_and_tf_vec; // 存储检测到的标记ID及其TF
    float similarity_threshold;  // 四元数相似度阈值，用于验证标记间变换的一致性
    float radius_threshold;      // 位置误差半径阈值
    float base_link_dummy_transform_x, base_link_dummy_transform_y, base_link_dummy_transform_z; // base_link到虚拟base_link的平移

    // 用于调试的上一帧相机位姿
    tf2::Stamped<tf2::Transform> camera_pose_last1;
    tf2::Stamped<tf2::Transform> camera_pose_last2;
    tf2::Stamped<tf2::Transform> camera_pose_current1;
    tf2::Stamped<tf2::Transform> camera_pose_current2;
    bool pose_inited = false; // 是否已初始化上一帧位姿

    tf2::Transform tf_baselink_to_camera; // base_link到相机坐标系的变换（动态）

    // 统计信息
    int frame_all = 0;        // 总处理帧数
    int frame_detected = 0;   // 成功检测到目标标记对的帧数
    int frame_not_detected = 0; // 未检测到标记的帧数
    int frame_error = 0;      // 检测到但验证失败的帧数

    // 解决相机话题未收到时/marker_visible状态超时问题
    double now_time = 0.0, last_time_camera_topic_received = 0.0;
};

// 注册节点组件
RCLCPP_COMPONENTS_REGISTER_NODE(AprilTagDoubleNode)

/**
 * @brief 构造函数，初始化节点参数、检测器、订阅器和发布器
 */
AprilTagDoubleNode::AprilTagDoubleNode(const rclcpp::NodeOptions& options)
  : Node("apriltag", options),
    // 初始化参数回调句柄
    cb_parameter(add_on_set_parameters_callback(std::bind(&AprilTagDoubleNode::onParameter, this, std::placeholders::_1))),
    td(apriltag_detector_create()), // 创建AprilTag检测器
    // 创建发布器（检测结果）
    pub_detections(create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>("detections", rclcpp::QoS(1))),
    tf_broadcaster(this)
{
    // 初始化TF缓存和监听器
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // 单独订阅图像话题，使用lambda手动处理时间戳和相机信息
    image_sub = image_transport::create_subscription(
        this, "/image_rect", 
        [this](const sensor_msgs::msg::Image::ConstSharedPtr& img) {
            // 如果有相机信息，则调用onCamera处理
            if (last_camera_info_) {
                info_sub.reset(); // 重置订阅器（避免重复处理？实际是避免循环，但此处代码似乎有误）
                this->onCamera(img, last_camera_info_);
            }
            else
            {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, "camera_info topic hasn't be received, waiting ...");
            }
        },
        "raw", rmw_qos_profile_sensor_data
    );

    // 单独订阅相机信息话题，缓存最新信息
    info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera_info", rclcpp::QoS(1).best_effort(),
        [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info) {
            last_camera_info_ = info;
        }
    );

    // 只读参数：AprilTag族名称
    apriltag_family_name = declare_parameter("family", "36h11", descr("tag family", true));
    // 默认标记边长（米）
    tag_edge_size = declare_parameter("size", 1.0, descr("default tag size", true));

    // 获取每个标记的ID、框架名和边长（可选）
    const auto ids = declare_parameter("tag.ids", std::vector<int64_t>{}, descr("tag ids", true));
    const auto frames = declare_parameter("tag.frames", std::vector<std::string>{}, descr("tag frame names per id", true));
    const auto sizes = declare_parameter("tag.sizes", std::vector<double>{}, descr("tag sizes per id", true));

    // 位姿估计方法（pnp或homography）
    estimate_pose = pose_estimation_methods.at(declare_parameter("pose_estimation_method", "pnp", descr("pose estimation method: \"pnp\" (more accurate) or \"homography\" (faster)", true)));

    // 检测器参数（位于"detector"命名空间）
    declare_parameter("detector.threads", td->nthreads, descr("number of threads"));
    declare_parameter("detector.decimate", td->quad_decimate, descr("decimate resolution for quad detection"));
    declare_parameter("detector.blur", td->quad_sigma, descr("sigma of Gaussian blur for quad detection"));
    declare_parameter("detector.refine", td->refine_edges, descr("snap to strong gradients"));
    declare_parameter("detector.sharpening", td->decode_sharpening, descr("sharpening of decoded images"));
    declare_parameter("detector.debug", td->debug, descr("write additional debugging images to working directory"));

    // 其他参数
    declare_parameter("max_hamming", 0, descr("reject detections with more corrected bits than allowed"));
    declare_parameter("profile", false, descr("print profiling information to stdout"));

    // 双标记验证相关参数
    declare_parameter("marker_id_and_bluetooth_mac_vec", std::vector<std::string>(), descr("the vector of marker id and bluetooth mac"));
    declare_parameter("marker_frame_translation", -0.04, descr("The translation distance on Y axis from charge frame to marker1 frame."));
    declare_parameter("similarity_threshold", 0.96, descr("similarity_threshold to check tf's validity"));
    declare_parameter("radius_threshold", 0.013, descr("radius_threshold to check tf's validity"));
    declare_parameter("base_link_dummy_transform_x", -0.374, descr("base_link_dummy_transform_x"));
    declare_parameter("base_link_dummy_transform_y", 0.0, descr("base_link_dummy_transform_y"));
    declare_parameter("base_link_dummy_transform_z", 0.50, descr("base_link_dummy_transform_z"));

    // 从参数服务器获取并赋值
    this->get_parameter_or<std::vector<std::string>>("marker_id_and_bluetooth_mac_vec", marker_id_and_bluetooth_mac_vector, {"0:1/94:C9:60:43:BE:01"});
    this->get_parameter_or<float>("marker_frame_translation", marker_frame_translation, -0.04);
    this->get_parameter_or<float>("similarity_threshold", similarity_threshold, 0.96);
    this->get_parameter_or<float>("radius_threshold", radius_threshold, 0.014);
    this->get_parameter_or<float>("base_link_dummy_transform_x", base_link_dummy_transform_x, -0.374);
    this->get_parameter_or<float>("base_link_dummy_transform_y", base_link_dummy_transform_y, 0.0);
    this->get_parameter_or<float>("base_link_dummy_transform_z", base_link_dummy_transform_z, 0.50);

    // 解析marker_id_and_bluetooth_mac_vector，构造marker与MAC对应消息
    int id_mac_length = marker_id_and_bluetooth_mac_vector.size();
    RCLCPP_INFO(get_logger(), "marker_id_and_bluetooth_mac_vector size: %d", id_mac_length);
    for (int ids_index = 0; ids_index < id_mac_length; ids_index++)
    {
        std::string id_and_mac = marker_id_and_bluetooth_mac_vector[ids_index];
        int id_and_mac_length = id_and_mac.length();
        int pos = id_and_mac.find('/');
        int marker_id_, marker_id_correction_;
        std::string bluetooth_mac;

        // 格式："主标记ID:校正标记ID/蓝牙MAC"，例如"0:1/94:C9:60:43:BE:01"
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

    // 配置标记ID到框架名的映射
    if(!frames.empty()) {
        if(ids.size() != frames.size()) {
            throw std::runtime_error("Number of tag ids (" + std::to_string(ids.size()) + ") and frames (" + std::to_string(frames.size()) + ") mismatch!");
        }
        for(size_t i = 0; i < ids.size(); i++) { tag_frames[ids[i]] = frames[i]; }
    }

    // 配置标记ID到边长的映射
    if(!sizes.empty()) {
        if(ids.size() != sizes.size()) {
            throw std::runtime_error("Number of tag ids (" + std::to_string(ids.size()) + ") and sizes (" + std::to_string(sizes.size()) + ") mismatch!");
        }
        for(size_t i = 0; i < ids.size(); i++) { tag_sizes[ids[i]] = sizes[i]; }
    }

    // 根据族名称添加对应的AprilTag族到检测器
    if(tag_fun.count(apriltag_family_name)) {
        tf = tag_fun.at(apriltag_family_name).first();
        tf_destructor = tag_fun.at(apriltag_family_name).second;
        apriltag_detector_add_family(td, tf);
    }
    else {
        throw std::runtime_error("Unsupported tag family: " + apriltag_family_name);
    }

    // 定义真实标记到虚拟标记的变换（用于调整坐标系方向）
    tf_real_to_dummy.setIdentity();
    tf2::Quaternion q_marker_real_to_dummy;
    q_marker_real_to_dummy.setRPY(-M_PI / 2.0, M_PI / 2.0, 0.0);
    tf_real_to_dummy.setRotation(q_marker_real_to_dummy);

    // 定义base_link到虚拟base_link的变换（用于标记位姿转换）
    tf_base_link_to_dummy_base_link.setIdentity();
    tf_base_link_to_dummy_base_link.setOrigin(tf2::Vector3(base_link_dummy_transform_x, base_link_dummy_transform_y, base_link_dummy_transform_z));
    tf2::Quaternion q_base_link_to_dummy_base_link;
    q_base_link_to_dummy_base_link.setRPY(0.0, 0.0, M_PI);
    tf_base_link_to_dummy_base_link.setRotation(q_base_link_to_dummy_base_link);

    // 定义标记1到充电器坐标系的固定变换（沿Y轴平移marker_frame_translation）
    tf_marker1_to_charger.setIdentity();
    RCLCPP_INFO(get_logger(), "marker_frame_translation: %f", marker_frame_translation);
    tf_marker1_to_charger.setOrigin(tf2::Vector3(0., marker_frame_translation, 0.));
    tf2::Quaternion q_marker_to_charger;
    q_marker_to_charger.setRPY(0., 0., 0.);
    tf_marker1_to_charger.setRotation(q_marker_to_charger);

    // 定义标记1到标记2的固定变换（基于两标记在充电器上的相对位置，此处假设标记2在标记1正Y方向两倍偏移）
    tf_marker1_to_marker2_fixed.setIdentity();
    tf_marker1_to_marker2_fixed.setOrigin(tf2::Vector3(0., marker_frame_translation * 2, 0.));
    tf2::Quaternion q_marker1_to_marker2;
    q_marker1_to_marker2.setRPY(0., 0., 0.);
    tf_marker1_to_marker2_fixed.setRotation(q_marker1_to_marker2);

    // 创建发布器
    pose_with_id_pub = this->create_publisher<aruco_msgs::msg::PoseWithId>("/pose_with_id", 100);
    detect_status = this->create_publisher<capella_ros_service_interfaces::msg::ChargeMarkerVisible>("marker_visible", rclcpp::QoS(1).reliable().transient_local());
    id_and_mac_pub = this->create_publisher<aruco_msgs::msg::MarkerAndMacVector>("/id_mac", 30);
    
    // 创建定时器，周期50ms，分别用于发布标记可见性和ID-MAC映射
    marker_timer = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&AprilTagDoubleNode::marker_visible_callback, this));
    id_mac_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&AprilTagDoubleNode::id_mac_callback, this));
    // 订阅充电器ID选择话题
    charger_id_sub = this->create_subscription<std_msgs::msg::String>("/charger/id", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local().reliable(),
                                                                       std::bind(&AprilTagDoubleNode::charger_id_callback, this, std::placeholders::_1));
}

/**
 * @brief 析构函数，释放检测器和AprilTag族资源
 */
AprilTagDoubleNode::~AprilTagDoubleNode()
{
    apriltag_detector_destroy(td);
    tf_destructor(tf);
}

/**
 * @brief 定期发布ID与MAC的映射关系
 */
void AprilTagDoubleNode::id_mac_callback()
{
    id_and_mac_pub->publish(msgs);
}

/**
 * @brief 接收/charger/id话题，设置当前使用的充电器ID（蓝牙MAC），并解析对应的两个标记ID
 */
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
        // 在映射表中查找匹配的蓝牙MAC，获取对应的两个标记ID
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

/**
 * @brief 定期发布标记可见性状态，并处理相机话题超时情况
 */
void AprilTagDoubleNode::marker_visible_callback()
{    
    now_time = now().seconds();

    // 如果超过0.5秒未收到相机图像，则发布不可见
    if (now_time - last_time_camera_topic_received > 0.5)
    {
        RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 5000, "--------------------------");
        RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 5000, "now_time: %f", now_time);
        RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 5000, "last_time_camera_topic_received: %f", last_time_camera_topic_received);
        RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 5000, "delta_time: %f", now_time - last_time_camera_topic_received);

        marker_detect_status.marker_visible = false;
        marker_detect_status.marker_id = -1;
        marker_detect_status.marker_id_correction = -1;
        RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000, "timeout, change marker_visible = false");
    }

    // 首次发布或状态变化时发布
    if (!marker_visible_pub)
    {
        RCLCPP_INFO(get_logger(), "pub topic /marker_visible first time.");
        detect_status->publish(marker_detect_status);
        marker_visible_pub = true;
        marker_visible_last = marker_detect_status.marker_visible;
    }
    else
    {
        if (marker_visible_last != marker_detect_status.marker_visible)
        {
            RCLCPP_INFO(get_logger(), "topic /marker_visible changed from %s to %s .",
                marker_visible_last ? "true": "false",
                marker_detect_status.marker_visible ? "true" : "false"
            );
            detect_status->publish(marker_detect_status);
            marker_visible_last = marker_detect_status.marker_visible;
        }
    }           
}

/**
 * @brief 核心图像处理回调函数：检测AprilTag，计算位姿，验证双标记，发布TF和消息
 */
void AprilTagDoubleNode::onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img,
                            const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_ci)
{
    // 记录相机话题接收时间
    last_time_camera_topic_received = this->get_clock()->now().seconds();
    try
    {
        // 初始化每帧数据
        tf_camera_to_marker1.setIdentity();
        tf_camera_to_marker2.setIdentity();
        id_and_tf_vec = std::vector<std::pair<int, geometry_msgs::msg::TransformStamped>>();

        // 获取base_link到相机坐标系的变换（动态）
        tf_baselink_to_camera.setIdentity();
        geometry_msgs::msg::TransformStamped stamped_tf_baselink_to_camera_msg;
        tf2::Stamped<tf2::Transform> stamped_tf_baselink_to_camera;
        if(getTransform(std::string("base_link"), msg_img->header.frame_id, stamped_tf_baselink_to_camera_msg))
        {
            tf2::fromMsg(stamped_tf_baselink_to_camera_msg, stamped_tf_baselink_to_camera);
            tf_baselink_to_camera = static_cast<tf2::Transform>(stamped_tf_baselink_to_camera);
        }

        // 从相机信息中提取内参（这里使用K矩阵，即焦距和主点）
        const std::array<double, 4> intrinsics = {msg_ci->k.data()[0], msg_ci->k.data()[4], msg_ci->k.data()[2], msg_ci->k.data()[5]};

        // 图像数据有效性检查
        if (!msg_img || msg_img->width <= 0 || msg_img->height <= 0) {
            RCLCPP_ERROR(get_logger(), "Invalid image message received");
            return;
        }
        cv::Mat img_uint8;
        try {
            // 将ROS图像转换为OpenCV的8位单通道图像
            img_uint8 = cv_bridge::toCvShare(msg_img, "mono8")->image;
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(get_logger(), "OpenCV exception: %s", e.what());
            return;
        }

        // 包装成AprilTag库所需的图像结构
        image_u8_t im{img_uint8.cols, img_uint8.rows, img_uint8.cols, img_uint8.data};

        // 检测AprilTag
        mutex.lock();
        double start_time = this->now().seconds();
        detections = *apriltag_detector_detect(td, &im);
        double end_time = this->now().seconds();
        RCLCPP_DEBUG(get_logger(), "compute detections cost time: %d ms", (int)round((end_time - start_time) * 1000));
        mutex.unlock();

        // 可选：打印性能信息
        if(profile)
            timeprofile_display(td->tp);

        // 准备发布检测结果消息
        apriltag_msgs::msg::AprilTagDetectionArray msg_detections;
        msg_detections.header = msg_img->header;

        std::vector<geometry_msgs::msg::TransformStamped> tfs; // 要广播的TF列表

        // 遍历所有检测到的标记
        for(int i = 0; i < zarray_size(&detections); i++) {
            apriltag_detection_t* det;
            zarray_get(&detections, i, &det);

            RCLCPP_DEBUG(get_logger(),
                         "detection %3d: id (%2dx%2d)-%-4d, hamming %d, margin %8.3f\n",
                         i, det->family->nbits, det->family->h, det->id,
                         det->hamming, det->decision_margin);

            // 如果指定了tag_frames（即只跟踪某些ID），则过滤掉不在映射中的标记
            if(!tag_frames.empty() && !tag_frames.count(det->id)) { continue; }

            // 根据汉明距离阈值过滤
            if(det->hamming > max_hamming) { continue; }

            // 构造检测消息
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

            // 创建真实标记到虚拟标记的TF（用于调整坐标系方向）
            geometry_msgs::msg::TransformStamped stampedTransform_real_to_dummy;
            std::stringstream ss_parent, ss_child;
            ss_parent << "april" << det->family->name << ":" << det->id;
            ss_child << "april" << det->family->name << ":" << det->id << "_dummy";
            stampedTransform_real_to_dummy.header.frame_id = ss_parent.str();
            stampedTransform_real_to_dummy.header.stamp = msg_img->header.stamp;
            stampedTransform_real_to_dummy.child_frame_id = ss_child.str();
            tf2::toMsg(tf_real_to_dummy, stampedTransform_real_to_dummy.transform);

            // 计算标记在相机坐标系下的位姿
            geometry_msgs::msg::TransformStamped tf;
            tf.header = msg_img->header;
            // 设置子框架名（如果配置了特定名称则使用，否则使用默认）
            tf.child_frame_id = tag_frames.count(det->id) ? tag_frames.at(det->id) : "april" + std::string(det->family->name) + ":" + std::to_string(det->id);
            const double size = tag_sizes.count(det->id) ? tag_sizes.at(det->id) : tag_edge_size;
            if(estimate_pose != nullptr) {
                tf.transform = estimate_pose(det, intrinsics, size, shared_from_this());
            }
            
            // 存储当前标记的ID和TF，供后续双标记验证使用
            std::pair<int, geometry_msgs::msg::TransformStamped> id_tf_pair;
            id_tf_pair.first = det->id;
            id_tf_pair.second = tf;
            id_and_tf_vec.push_back(id_tf_pair);
            // 将真实到虚拟的TF加入广播列表
            tfs.push_back(stampedTransform_real_to_dummy);
        }

        // 更新统计信息并确定标记可见性
        int detections_size = zarray_size(&detections);
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
            // 收集所有通过汉明距离过滤的标记ID
            for (int i = 0; i < detections_size; i++)
            {            
                apriltag_detection_t* det;
                zarray_get(&detections, i, &det);
                if (det->hamming <= max_hamming.load())
                {
                    marker_id_vector.push_back(det->id);
                }
            }
            if (id_selected)
            {
                // 如果已选择特定充电器，检查是否同时检测到对应的两个标记
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
                // 未选择特定充电器，则检查是否检测到任意一组配置的标记对
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
        
        // 如果标记可见，则进行双标记一致性验证，并发布充电器位姿
        if (marker_detect_status.marker_visible)
        {
            frame_detected++;
            // 添加充电器坐标系到标记1虚拟坐标系的TF
            geometry_msgs::msg::TransformStamped stampedTransform_marker1_to_charger;
            std::stringstream ss_marker1_frame;
            ss_marker1_frame << "apriltag" << apriltag_family_name << ":" << marker_id << "_dummy";
            stampedTransform_marker1_to_charger.header.frame_id = ss_marker1_frame.str();
            stampedTransform_marker1_to_charger.header.stamp = msg_img->header.stamp;
            stampedTransform_marker1_to_charger.child_frame_id = std::string("charger");
            tf2::toMsg(tf_marker1_to_charger, stampedTransform_marker1_to_charger.transform);
            tfs.push_back(stampedTransform_marker1_to_charger);

            // 在id_and_tf_vec中查找两个标记的TF
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

            // 计算当前检测中标记1到标记2的变换
            tf2::Transform tf_marker1_to_marker2_current;
            tf_marker1_to_marker2_current = tf_real_to_dummy.inverse() * tf_camera_to_marker1.inverse() * tf_camera_to_marker2 * tf_real_to_dummy;
            
            // 调试信息：打印固定变换和当前变换的差异（仅一次）
            {
                // marker1 to marker2 , fixed
                {
                    double x, y, theta;
                    x = tf_marker1_to_marker2_fixed.getOrigin()[0];
                    y = tf_marker1_to_marker2_fixed.getOrigin()[1];
                    theta = tf2::getYaw(tf_marker1_to_marker2_fixed.getRotation());
                    RCLCPP_INFO_ONCE(get_logger(), "x: %f, y: %f, theta: %f", x, y, theta);
                }
                // marker1 to marker2 , current
                {
                    double x, y, theta;
                    x = tf_marker1_to_marker2_current.getOrigin()[0];
                    y = tf_marker1_to_marker2_current.getOrigin()[1];
                    theta = tf2::getYaw(tf_marker1_to_marker2_current.getRotation());
                    RCLCPP_INFO_ONCE(get_logger(), "x_c: %f, y_c: %f, theta_c: %f", x, y, theta);
                }
                // marker1_dummy to base_link_dummy
                {
                    double x, y, theta;
                    auto tf = tf_real_to_dummy.inverse() * tf_camera_to_marker1.inverse() * tf_baselink_to_camera.inverse() * tf_base_link_to_dummy_base_link;
                    x = tf.getOrigin()[0];
                    y = tf.getOrigin()[1];
                    theta = tf2::getYaw(tf.getRotation());
                    RCLCPP_INFO_ONCE(get_logger(), "x_1: %f, y_1: %f, theta_1: %f", x, y, theta);
                }
                // marker2_dummy to base_link_dummy
                {
                    double x, y, theta;
                    auto tf = tf_real_to_dummy.inverse() * tf_camera_to_marker2.inverse() * tf_baselink_to_camera.inverse() * tf_base_link_to_dummy_base_link;
                    x = tf.getOrigin()[0];
                    y = tf.getOrigin()[1];
                    theta = tf2::getYaw(tf.getRotation());
                    RCLCPP_INFO_ONCE(get_logger(), "x_2: %f, y_2: %f, theta_2: %f", x, y, theta);
                }
            }

            // 计算当前变换与固定变换之间的差异（位置误差和旋转相似度）
            auto tf_fixed_to_current = tf_marker1_to_marker2_fixed.inverse() * tf_marker1_to_marker2_current;
            float error_x, error_y, error_z;
            error_x = tf_fixed_to_current.getOrigin()[0];
            error_y = tf_fixed_to_current.getOrigin()[1];
            error_z = tf_fixed_to_current.getOrigin()[2];
            float error_radius = std::hypot(std::hypot(error_x, error_y), error_z);
            
            float w_f, x_f, y_f, z_f, w_c, x_c, y_c, z_c; // f:fixed, c:current
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
            float similarity = w_f * w_c + x_f * x_c + y_f * y_c + z_f * z_c; // 四元数点积，表征旋转一致性

            // 如果一致性满足阈值，则发布有效位姿
            if (similarity > similarity_threshold && error_radius < radius_threshold)
            {
                // 将两个标记的TF加入广播列表
                tfs.push_back(id_and_tf_vec[index_marker1].second);
                tfs.push_back(id_and_tf_vec[index_marker2].second);

                // 计算充电器到base_link_dummy的变换，并发布位姿消息
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
            }
        }

        // 添加base_link到base_link_dummy的TF（固定）
        geometry_msgs::msg::TransformStamped tf_baselink_to_baselink_dummy_msg;
        tf_baselink_to_baselink_dummy_msg.header.frame_id = std::string("base_link");
        tf_baselink_to_baselink_dummy_msg.header.stamp = msg_img->header.stamp;
        tf_baselink_to_baselink_dummy_msg.child_frame_id = std::string("base_link_dummy");
        tf2::toMsg(tf_base_link_to_dummy_base_link, tf_baselink_to_baselink_dummy_msg.transform);
        tfs.push_back(tf_baselink_to_baselink_dummy_msg);

        // 发布检测结果和所有TF
        pub_detections->publish(msg_detections);
        tf_broadcaster.sendTransform(tfs);
    }
    catch(const char* msg)
    {
        RCLCPP_INFO(get_logger(), "error: %s", msg);
    }
    // 输出统计信息（每10秒）
    if (frame_detected > 0){
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000, "frame_detected_rate: %f", frame_detected / (float)frame_all);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000, "tf error rate: %f", frame_error / (float)frame_detected);
    }
}

/**
 * @brief 检查检测到的标记ID集合是否包含任意一组配置的标记对
 * @param ids 检测到的标记ID列表
 * @return 如果存在一组完整的标记对，返回true，并设置成员变量marker_id和marker_id_correction
 */
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

/**
 * @brief 参数动态更新回调函数
 */
rcl_interfaces::msg::SetParametersResult
AprilTagDoubleNode::onParameter(const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    mutex.lock();
    for(const rclcpp::Parameter& parameter : parameters) {
        RCLCPP_DEBUG_STREAM(get_logger(), "setting: " << parameter);
        // 使用宏IF处理可动态更新的参数
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

/**
 * @brief 获取两个坐标系之间的TF变换
 */
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