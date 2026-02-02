// ros
#include "pose_estimation.hpp"
#include <apriltag_msgs/msg/april_tag_detection.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#ifdef cv_bridge_HPP
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/transform_broadcaster.h>
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

// apriltag
#include "tag_functions.hpp"
#include <apriltag.h>

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>

#include <image_transport/camera_subscriber.hpp>
#include <image_transport/image_transport.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>

typedef sensor_msgs::msg::Image ImageT;
typedef message_filters::sync_policies::ApproximateTime<
      ImageT, ImageT>
      ImgImgApproxSync;
typedef sensor_msgs::msg::CameraInfo CameraInfoT;


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

class Cam2CamNode : public rclcpp::Node {
public:
    Cam2CamNode(const rclcpp::NodeOptions& options);

    ~Cam2CamNode() override;

private:

    //  parameters specified by launch file
    std::string imageRefTopic;
    std::string imageSourceTopic;
    std::string cameraInfoRefTopic;
    std::string cameraInfoSourceTopic;
    std::string base_frame_id;
    int calibration_id;
    int sync_size;
    std::string target_camera_type;

    tf2::Transform tf_camera_link_to_color_optical;
    void initializeCameraLinkToColorOptical();
    
    // Subscriber to imageRef topic
    image_transport::SubscriberFilter imageRefSubsc_;
    // Subscriber to imageSource topic
    message_filters::Subscriber<ImageT> imageSourceSubsc_;

    // message filter for approximated message synchronization for imageRef and imageSource message data
    std::shared_ptr<message_filters::Synchronizer<ImgImgApproxSync>> pImgImgApproxSync_;

    // output tf 
    tf2::Transform tf_marker2ref;
    tf2::Transform tf_marker2source;
    double translation_x, translation_y, translation_z;
    double rotation_roll, rotation_pitch, rotation_yaw;
    int data_number = 0;
    double translation_x_mean = 0.0, translation_y_mean = 0.0, translation_z_mean = 0.0;
    double rotation_roll_mean = 0.0, rotation_pitch_mean = 0.0, rotation_yaw_mean = 0.0;

    double calculate_mean(const double& mean_history, int& number, double new_value);

    void onSensorDataReceived(const ImageT::ConstSharedPtr& imgRefMsg, const ImageT::ConstSharedPtr& imgSourceMsg);

    tf2_ros::TransformBroadcaster tf_broadcaster;

    // get camera_info
    std::atomic<bool> camera_info_ref_received_{false};
    std::atomic<bool> camera_info_source_received_{false};
    rclcpp::TimerBase::SharedPtr timer_camera_info_ref_;
    rclcpp::TimerBase::SharedPtr timer_camera_info_source_; 
    void timer_camera_info_ref_callback();
    void timer_camera_info_source_callback();
    CameraInfoT camera_info_ref_msg;
    CameraInfoT camera_info_source_msg;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_ref_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_source_sub;
    void onCameraInfoRef(const sensor_msgs::msg::CameraInfo::SharedPtr msg_ci);
    void onCameraInfoSource(const sensor_msgs::msg::CameraInfo::SharedPtr msg_ci);

    bool initializeSubscribers();

    geometry_msgs::msg::TransformStamped generateTFStamped(const std::string &parent, const std::string &child, tf2::Transform, builtin_interfaces::msg::Time stamp);

    void printTransform(const std::string& name, const tf2::Transform& tf);

    const OnSetParametersCallbackHandle::SharedPtr cb_parameter;

    apriltag_family_t* tf;
    apriltag_detector_t* const td;

    // apriltag parameter
    std::mutex mutex;
    double tag_edge_size;
    std::atomic<int> max_hamming;
    std::atomic<bool> profile;
    std::unordered_map<int, std::string> tag_frames;
    std::unordered_map<int, double> tag_sizes;    

    std::function<void(apriltag_family_t*)> tf_destructor;

    pose_estimation_f estimate_pose = nullptr;

    bool onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img, const sensor_msgs::msg::CameraInfo& msg_cam_info, tf2::Transform & tf);

    rcl_interfaces::msg::SetParametersResult onParameter(const std::vector<rclcpp::Parameter>& parameters);

    bool getTransform(
        const std::string & refFrame, const std::string & childFrame,
        geometry_msgs::msg::TransformStamped & transform);
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    zarray_t* detections;
};

RCLCPP_COMPONENTS_REGISTER_NODE(Cam2CamNode)

Cam2CamNode::Cam2CamNode(const rclcpp::NodeOptions& options)
  : Node("apriltag", options),
    tf_broadcaster(this),
    // parameter
    cb_parameter(add_on_set_parameters_callback(std::bind(&Cam2CamNode::onParameter, this, std::placeholders::_1))),
    td(apriltag_detector_create())
{
    RCLCPP_INFO(get_logger(), "Cam2CamNode started.");

    detections = new zarray_t();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // read-only parameters
    const std::string tag_family = declare_parameter("family", "36h11", descr("tag family", true));
    tag_edge_size = declare_parameter("size", 1.0, descr("default tag size", true));

    // get tag names, IDs and sizes
    const auto ids = declare_parameter("tag.ids", std::vector<int64_t>{}, descr("tag ids", true));
    const auto frames = declare_parameter("tag.frames", std::vector<std::string>{}, descr("tag frame names per id", true));
    const auto sizes = declare_parameter("tag.sizes", std::vector<double>{}, descr("tag sizes per id", true));

    // get method for estimating tag pose
    estimate_pose = pose_estimation_methods.at(declare_parameter("pose_estimation_method", "pnp", descr("pose estimation method: \"pnp\" (more accurate) or \"homography\" (faster)", true)));

    // detector parameters in "detector" namespace
    declare_parameter("detector.threads", td->nthreads, descr("number of threads"));
    declare_parameter("detector.decimate", td->quad_decimate, descr("decimate resolution for quad detection"));
    declare_parameter("detector.blur", td->quad_sigma, descr("sigma of Gaussian blur for quad detection"));
    declare_parameter("detector.refine", td->refine_edges, descr("snap to strong gradients"));
    declare_parameter("detector.sharpening", td->decode_sharpening, descr("sharpening of decoded images"));
    declare_parameter("detector.debug", td->debug, descr("write additional debugging images to working directory"));
    declare_parameter("max_hamming", 0, descr("reject detections with more corrected bits than allowed"));
    declare_parameter("profile", false, descr("print profiling information to stdout"));

    // declare launch parameters
    imageRefTopic = declare_parameter("image_ref_topic", "/rgb_camera_front/image_raw", descr("topic of ref image", true));
    cameraInfoRefTopic = declare_parameter("camera_info_ref_topic", "/rgb_camera_front/camera_info", descr("topic of ref camera info", true));
    imageSourceTopic = declare_parameter("image_source_topic", "/camera2/color/image_raw", descr("topic of source image", true));
    cameraInfoSourceTopic = declare_parameter("camera_info_source_topic", "/camera2/color/camera_info", descr("topic of source camera info", true));
    calibration_id = declare_parameter("calibration_id", 0, descr("id of calibration marker", true));
    base_frame_id = declare_parameter("base_frame_id", "base_link", descr("base_frame id", true));
    sync_size = declare_parameter("sync_size", 5, descr("sync_size", true));
    target_camera_type = declare_parameter("target_camera_type", "rgb", descr("type of target camera", true));


    RCLCPP_INFO(get_logger(), "image_ref_topic           : %s", imageRefTopic.c_str());
    RCLCPP_INFO(get_logger(), "camera_info_ref_topic     : %s", cameraInfoRefTopic.c_str());
    RCLCPP_INFO(get_logger(), "image_source_topic        : %s", imageSourceTopic.c_str());
    RCLCPP_INFO(get_logger(), "camera_info_ref_topic     : %s", cameraInfoSourceTopic.c_str());
    RCLCPP_INFO(get_logger(), "calibration_id            : %d", calibration_id);
    RCLCPP_INFO(get_logger(), "apriltag marker size      : %.2f m", tag_edge_size);
    RCLCPP_INFO(get_logger(), "base_frame_id             : %s", base_frame_id.c_str());
    RCLCPP_INFO(get_logger(), "sync_size                 : %d", sync_size);
    RCLCPP_INFO(get_logger(), "target_camera_type        : %s", target_camera_type.c_str());

    if(!frames.empty()) {
        if(ids.size() != frames.size()) {
            throw std::runtime_error("Number of tag ids (" + std::to_string(ids.size()) + ") and frames (" + std::to_string(frames.size()) + ") mismatch!");
        }
        for(size_t i = 0; i < ids.size(); i++) { tag_frames[ids[i]] = frames[i]; }
    }

    if(!sizes.empty()) {
        // use tag specific size
        if(ids.size() != sizes.size()) {
            throw std::runtime_error("Number of tag ids (" + std::to_string(ids.size()) + ") and sizes (" + std::to_string(sizes.size()) + ") mismatch!");
        }
        for(size_t i = 0; i < ids.size(); i++) { tag_sizes[ids[i]] = sizes[i]; }
    }

    if(tag_fun.count(tag_family)) {
        tf = tag_fun.at(tag_family).first();
        tf_destructor = tag_fun.at(tag_family).second;
        apriltag_detector_add_family(td, tf);
    }
    else {
        throw std::runtime_error("Unsupported tag family: " + tag_family);
    }

    // add timer for get camera_info topic
    timer_camera_info_ref_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&Cam2CamNode::timer_camera_info_ref_callback, this));
    timer_camera_info_source_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&Cam2CamNode::timer_camera_info_source_callback, this)); 

    initializeCameraLinkToColorOptical();
    initializeSubscribers();   
}

Cam2CamNode::~Cam2CamNode()
{
    apriltag_detector_destroy(td);
    tf_destructor(tf);
}

void Cam2CamNode::printTransform(const std::string& name, const tf2::Transform& tf)
{
    auto origin = tf.getOrigin();
    auto rotation = tf.getRotation();
    tf2::Matrix3x3 mat(rotation);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);
    RCLCPP_DEBUG(get_logger(), "%s pos=(%f, %f, %f) rpy=(%f, %f, %f)",
                name.c_str(), origin.x(), origin.y(), origin.z(),
                roll, pitch, yaw);
}

geometry_msgs::msg::TransformStamped Cam2CamNode::generateTFStamped(const std::string &parent, const std::string &child, tf2::Transform tf, builtin_interfaces::msg::Time stamp)
{
    geometry_msgs::msg::TransformStamped tf_stamped_msg;
    tf_stamped_msg.header.frame_id = parent;
    tf_stamped_msg.header.stamp = stamp;
    tf_stamped_msg.child_frame_id = child;
    tf2::toMsg(tf, tf_stamped_msg.transform);
    return tf_stamped_msg;
    
}

void Cam2CamNode::initializeCameraLinkToColorOptical()
{
   tf_camera_link_to_color_optical.setIdentity();
   if (target_camera_type == "rgb")
    {        
        RCLCPP_INFO(get_logger(), "target_camera_tpye: %s", target_camera_type.c_str());
        tf_camera_link_to_color_optical.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
        tf2::Quaternion q;
        q.setRPY(M_PI / 2.0, -M_PI / 2.0, 0.0);
        tf_camera_link_to_color_optical.setRotation(q);
    }
    else if (target_camera_type == "depth")
    {
        RCLCPP_INFO(get_logger(), "target_camera_tpye: %s", target_camera_type.c_str());
        tf_camera_link_to_color_optical.setOrigin(tf2::Vector3(-0.014, 0.000, -0.002));
        tf2::Quaternion q;
        q.setRPY(M_PI / 2.0, -M_PI / 2.0, 0.0);
        tf_camera_link_to_color_optical.setRotation(q);
    }
    
}

bool Cam2CamNode::initializeSubscribers()
{
    //--- subscribe to topics
    imageRefSubsc_.subscribe(this, imageRefTopic, "raw", rmw_qos_profile_sensor_data); 
    imageSourceSubsc_.subscribe(this, imageSourceTopic, rmw_qos_profile_sensor_data);

    //--- initialize synchronizers
    pImgImgApproxSync_ =
        std::make_shared<message_filters::Synchronizer<ImgImgApproxSync>>(
        ImgImgApproxSync(sync_size), imageRefSubsc_, imageSourceSubsc_);
    pImgImgApproxSync_->registerCallback(
        std::bind(&Cam2CamNode::onSensorDataReceived, this,
                std::placeholders::_1, std::placeholders::_2));

    return true;
}

void Cam2CamNode::onSensorDataReceived(const ImageT::ConstSharedPtr& imgRefMsg, const ImageT::ConstSharedPtr& imgSourceMsg)
{
    if (!camera_info_ref_received_ || !camera_info_source_received_)
    {
        RCLCPP_INFO(get_logger(), "at least one camera_info topic hadn't received, wating ...");
        return;
    }

    bool marker2ref = onCamera(imgRefMsg, camera_info_ref_msg, tf_marker2ref);
    if (!marker2ref)
    {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "topic %s 未检测到apriltag %d", imageRefTopic.c_str(), calibration_id);
        return;
    }

    bool marker2source = onCamera(imgSourceMsg, camera_info_source_msg, tf_marker2source);
    if (!marker2source)
    {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "topic %s 未检测到apriltag %d", imageSourceTopic.c_str(), calibration_id);
        return;
    }

    geometry_msgs::msg::TransformStamped tf_msg_ref_to_base_frame;
    bool get_tf_ref2baseframe = getTransform(base_frame_id, imgRefMsg->header.frame_id, tf_msg_ref_to_base_frame);
    if (!get_tf_ref2baseframe)
    {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "can not get tf %s to %s", imgRefMsg->header.frame_id.c_str(), base_frame_id.c_str());
        return;
    }
    tf2::Transform tf_ref2baseframe;
    tf2::fromMsg(tf_msg_ref_to_base_frame.transform, tf_ref2baseframe);

    RCLCPP_DEBUG(get_logger(), "=== 完整TF链 ===");
    printTransform("tf_ref2baseframe          :", tf_ref2baseframe);
    printTransform("tf_marker2ref             :", tf_marker2ref);
    printTransform("tf_marker2source          :", tf_marker2source);
    printTransform("tf_marker2source.inverse():", tf_marker2source.inverse());
    printTransform("tf_camera_link_to_color_optical            :", tf_camera_link_to_color_optical);
    printTransform("tf_boader2base            :", tf_ref2baseframe * tf_marker2ref);
    printTransform("tf_source2base            :", tf_ref2baseframe * tf_marker2ref * tf_marker2source.inverse());
    printTransform("tf_source_dummy2base      :", tf_ref2baseframe * tf_marker2ref * tf_marker2source.inverse() * tf_camera_link_to_color_optical);

    auto tf_source2base = tf_ref2baseframe * tf_marker2ref * tf_marker2source.inverse() * tf_camera_link_to_color_optical;
    auto origin = tf_source2base.getOrigin();
    auto rotation = tf_source2base.getRotation();
    translation_x = origin.getX();
    translation_y = origin.getY();
    translation_z = origin.getZ();
    tf2::Matrix3x3 mat(rotation);
    mat.getRPY(rotation_roll, rotation_pitch, rotation_yaw);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "-----------------------------------------------");
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "origin      (x,y,z): (%f %f %f)", translation_x, translation_y, translation_z);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "rotation      rpy: (%f %f %f)", rotation_roll, rotation_pitch, rotation_yaw);
    
    translation_x_mean = calculate_mean(translation_x_mean, data_number, translation_x);
    translation_y_mean = calculate_mean(translation_y_mean, data_number, translation_y);
    translation_z_mean = calculate_mean(translation_z_mean, data_number, translation_z);
    rotation_roll_mean = calculate_mean(rotation_roll_mean, data_number, rotation_roll);
    rotation_pitch_mean = calculate_mean(rotation_pitch_mean, data_number, rotation_pitch);
    rotation_yaw_mean = calculate_mean(rotation_yaw_mean, data_number, rotation_yaw);
    data_number++;

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "***********************************************");
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "origin_mean (x,y,z): (%f %f %f)", translation_x_mean, translation_y_mean, translation_z_mean);
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "rotation_mean rpy: (%f %f %f)", rotation_roll_mean, rotation_pitch_mean, rotation_yaw_mean);
    
    tf2::Transform tf_source2base_fake;
    tf_source2base_fake.setIdentity();
    tf2::Quaternion q_source2base_fake;
    q_source2base_fake.setRPY(rotation_roll_mean, rotation_pitch_mean, rotation_yaw_mean);
    tf_source2base_fake.setOrigin(tf2::Vector3(translation_x_mean, translation_y_mean, translation_z_mean));
    tf_source2base_fake.setRotation(q_source2base_fake);

    // pub tf
    std::vector<geometry_msgs::msg::TransformStamped> tfs;
    tfs.push_back(generateTFStamped(imgRefMsg->header.frame_id, "boader", tf_marker2ref, imgRefMsg->header.stamp));
    tfs.push_back(generateTFStamped("boader", imgSourceMsg->header.frame_id + "_dummy", tf_marker2source.inverse(), imgRefMsg->header.stamp));
    tfs.push_back(generateTFStamped(base_frame_id.c_str(), imgSourceMsg->header.frame_id + "_fake", tf_source2base_fake, imgRefMsg->header.stamp));
    tf_broadcaster.sendTransform(tfs);
}

double Cam2CamNode::calculate_mean(const double& mean_history, int& number, double new_value)
{
    double mean = 0.0;
    double sum = mean_history * number + new_value;
    mean = sum / (number + 1);
    return mean;
}

void Cam2CamNode::timer_camera_info_ref_callback()
{
    if (!camera_info_ref_received_ && !camera_info_ref_sub)
    {
        RCLCPP_INFO(get_logger(), "Attempting to subscribe to %s", cameraInfoRefTopic.c_str());
        camera_info_ref_sub = this->create_subscription<CameraInfoT>(cameraInfoRefTopic, rclcpp::QoS(1).best_effort(), std::bind(&Cam2CamNode::onCameraInfoRef, this, std::placeholders::_1));
    }
}

void Cam2CamNode::timer_camera_info_source_callback()
{
    if (!camera_info_source_received_ && !camera_info_source_sub)
    {
        RCLCPP_INFO(get_logger(), "Attempting to subscribe to %s", cameraInfoSourceTopic.c_str());
        camera_info_source_sub = this->create_subscription<CameraInfoT>(cameraInfoSourceTopic, rclcpp::QoS(1).best_effort(), std::bind(&Cam2CamNode::onCameraInfoSource, this, std::placeholders::_1));
    }
}

void Cam2CamNode::onCameraInfoRef(const sensor_msgs::msg::CameraInfo::SharedPtr msg_ci)
{
    if (camera_info_ref_received_) return;  // 防止重复处理
    camera_info_ref_msg = *msg_ci;
    camera_info_ref_received_ = true;
    RCLCPP_INFO(get_logger(), "camera_info_ref topic %s received. stopping timer", cameraInfoRefTopic.c_str());

    if (timer_camera_info_ref_) 
    {
        timer_camera_info_ref_->cancel();
        timer_camera_info_ref_.reset();
        camera_info_ref_sub.reset();
    }
}

void Cam2CamNode::onCameraInfoSource(const sensor_msgs::msg::CameraInfo::SharedPtr msg_ci)
{
    if (camera_info_source_received_) return; // 防止重复处理
    camera_info_source_msg = *msg_ci;
    camera_info_source_received_ = true;
    RCLCPP_INFO(get_logger(), "camera_info_source topic %s received. stopping timer", cameraInfoSourceTopic.c_str());

    if (timer_camera_info_source_)
    {
        timer_camera_info_source_->cancel();
        timer_camera_info_source_.reset();
        camera_info_source_sub.reset();
    }
}

bool Cam2CamNode::onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img, const sensor_msgs::msg::CameraInfo& msg_cam_info, tf2::Transform& tf_marker2img)
{
    std::array<double, 4> intrinsics = {msg_cam_info.k.data()[0], msg_cam_info.k.data()[4], msg_cam_info.k.data()[2], msg_cam_info.k.data()[5]};

    RCLCPP_DEBUG(get_logger(), "k => %f, %f, %f, %f", intrinsics[0], intrinsics[1], intrinsics[2], intrinsics[3]);
    
    //  RCLCPP_INFO(get_logger(), 
    //            "Image message details:\n"
    //            "  frame_id: %s\n"
    //            "  encoding: %s\n"
    //            "  width: %d\n"
    //            "  height: %d\n"
    //            "  step: %d (bytes per row)\n"
    //            "  data size: %zu bytes\n"
    //            "  is_bigendian: %d\n"
    //            "  timestamp: %d.%09d",
    //            msg_img->header.frame_id.c_str(),
    //            msg_img->encoding.c_str(),
    //            msg_img->width,
    //            msg_img->height,
    //            msg_img->step,
    //            msg_img->data.size(),
    //            msg_img->is_bigendian,
    //            msg_img->header.stamp.sec,
    //            msg_img->header.stamp.nanosec);

    // convert to 8bit monochrome image
    // RCLCPP_INFO(get_logger(), "frame_id: %s, width: %d, height: %d", msg_img->header.frame_id.c_str(), msg_img->width, msg_img->height);
    // 添加数据有效性检查
    if (!msg_img || msg_img->width <= 0 || msg_img->height <= 0) {
        RCLCPP_ERROR(get_logger(), "Invalid image message received");
        return false;
    }
    cv::Mat img_bgr8, img_uint8 ;
    // 使用try-catch结构提高代码健壮性
    try {
         img_bgr8 = cv_bridge::toCvShare(msg_img, "bgr8")->image;
         cv::cvtColor(img_bgr8, img_uint8, cv::COLOR_BGR2GRAY);
    } catch (const cv::Exception& e) {
        RCLCPP_ERROR(get_logger(), "OpenCV exception: %s", e.what());
        return false;
    }

    image_u8_t im{img_uint8.cols, img_uint8.rows, img_uint8.cols, img_uint8.data};

    // detect tags
    mutex.lock();
    *detections = *apriltag_detector_detect(td, &im);
    mutex.unlock();

    if(profile)
        timeprofile_display(td->tp);

    std::vector<geometry_msgs::msg::TransformStamped> tfs;

    std::stringstream ss_detection_ids;
    for(int i = 0; i < zarray_size(detections); i++) 
    {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);

        RCLCPP_DEBUG(get_logger(),
                     "detection %3d: id (%2dx%2d)-%-4d, hamming %d, margin %8.3f\n",
                     i, det->family->nbits, det->family->h, det->id,
                     det->hamming, det->decision_margin);

        // ignore untracked tags
        if(!tag_frames.empty() && !tag_frames.count(det->id)) { continue; }

        // reject detections with more corrected bits than allowed
        if(det->hamming > max_hamming) { continue; }

        ss_detection_ids << det->id << " ";
        if (det->id != calibration_id)
        {
            if (i == (zarray_size(detections) - 1))
            {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "topic frame_id: %s, alibration_id: %d, detection_ids: %s, ignore ...", 
                    msg_img->header.frame_id.c_str(), calibration_id, ss_detection_ids.str().c_str());
            }            
            continue;
        }
        else
        {
            geometry_msgs::msg::Transform tf_msg;
            const double size = tag_sizes.count(det->id) ? tag_sizes.at(det->id) : tag_edge_size;
            if(estimate_pose != nullptr) 
            {
                tf_msg = estimate_pose(det, intrinsics, size, shared_from_this());
                tf2::fromMsg(tf_msg, tf_marker2img);
                return true;
            }
        }
    }
    return false;
}

rcl_interfaces::msg::SetParametersResult
Cam2CamNode::onParameter(const std::vector<rclcpp::Parameter>& parameters)
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

bool Cam2CamNode::getTransform(
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
				refFrame, childFrame, tf2::TimePointZero, tf2::durationFromSec(
					0.5));
		} catch (const tf2::TransformException & e) {
			RCLCPP_ERROR_STREAM(
				this->get_logger(),
				"Error in lookupTransform of " << childFrame << " in " << refFrame << " : " << e.what());
			return false;
		}
	}
	return true;
}
