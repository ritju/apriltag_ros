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

class AprilTagSingleNode : public rclcpp::Node {
public:
    AprilTagSingleNode(const rclcpp::NodeOptions& options);

    ~AprilTagSingleNode() override;

private:

    //  parameters specified by launch file
    std::string imageTopic;
    std::string cameraInfoTopic;
    int calibration_id;
    double boader_height;

    // output tf 
    tf2::Transform tf_marker2camera;
    tf2::Transform tf_camera_link_to_color_optical;
    double translation_x, translation_y, translation_z;
    double rotation_roll, rotation_pitch, rotation_yaw;
    int data_number = 0;
    double translation_x_mean = 0.0, translation_y_mean = 0.0, translation_z_mean = 0.0;
    double rotation_roll_mean = 0.0, rotation_pitch_mean = 0.0, rotation_yaw_mean = 0.0;

    double calculate_mean(const double& mean_history, int& number, double new_value);

    tf2_ros::TransformBroadcaster tf_broadcaster;

    // camera_info
    std::atomic<bool> camera_info_received_{false};
    rclcpp::TimerBase::SharedPtr timer_camera_info_;
    void timer_camera_info_callback();
    CameraInfoT camera_info_msg;

    // subs
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
    void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg_ci);
    void onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img);

    void initializeCameraLinkToColorOptical();

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

    rcl_interfaces::msg::SetParametersResult onParameter(const std::vector<rclcpp::Parameter>& parameters);

    bool getTransform(
        const std::string & refFrame, const std::string & childFrame,
        geometry_msgs::msg::TransformStamped & transform);
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    zarray_t* detections;
};

RCLCPP_COMPONENTS_REGISTER_NODE(AprilTagSingleNode)

AprilTagSingleNode::AprilTagSingleNode(const rclcpp::NodeOptions& options)
  : Node("apriltag", options),
    tf_broadcaster(this),
    // parameter
    cb_parameter(add_on_set_parameters_callback(std::bind(&AprilTagSingleNode::onParameter, this, std::placeholders::_1))),
    td(apriltag_detector_create())
{
    RCLCPP_INFO(get_logger(), "AprilTagSingleNode started.");

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
    imageTopic = declare_parameter("image_topic", "/rgb_camera_front/image_raw", descr("topic of image", true));
    cameraInfoTopic = declare_parameter("camera_info_topic", "/rgb_camera_front/camera_info", descr("topic of camera info", true));
    calibration_id = declare_parameter("calibration_id", 0, descr("id of calibration marker", true));
    boader_height = declare_parameter("boader_height", 0.01, descr("border_height", true));

    RCLCPP_INFO(get_logger(), "image_topic           : %s", imageTopic.c_str());
    RCLCPP_INFO(get_logger(), "camera_info_topic     : %s", cameraInfoTopic.c_str());
    RCLCPP_INFO(get_logger(), "calibration_id        : %d", calibration_id);
    RCLCPP_INFO(get_logger(), "apriltag marker size  : %.2f m", tag_edge_size);
    RCLCPP_INFO(get_logger(), "board_height          : %.2f m", boader_height);

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
    timer_camera_info_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&AprilTagSingleNode::timer_camera_info_callback, this));
    image_sub = this->create_subscription<ImageT>(imageTopic, rclcpp::QoS(1).best_effort(), std::bind(&AprilTagSingleNode::onCamera, this, std::placeholders::_1));

    initializeCameraLinkToColorOptical(); 
}

AprilTagSingleNode::~AprilTagSingleNode()
{
    apriltag_detector_destroy(td);
    tf_destructor(tf);
}

void AprilTagSingleNode::printTransform(const std::string& name, const tf2::Transform& tf)
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

geometry_msgs::msg::TransformStamped AprilTagSingleNode::generateTFStamped(const std::string &parent, const std::string &child, tf2::Transform tf, builtin_interfaces::msg::Time stamp)
{
    geometry_msgs::msg::TransformStamped tf_stamped_msg;
    tf_stamped_msg.header.frame_id = parent;
    tf_stamped_msg.header.stamp = stamp;
    tf_stamped_msg.child_frame_id = child;
    tf2::toMsg(tf, tf_stamped_msg.transform);
    return tf_stamped_msg;    
}

void AprilTagSingleNode::initializeCameraLinkToColorOptical()
{
    tf_camera_link_to_color_optical.setIdentity();
    tf_camera_link_to_color_optical.setOrigin(tf2::Vector3(-0.014, 0.000, -0.002));
    tf2::Quaternion q;
    q.setRPY(M_PI / 2.0, -M_PI / 2.0, 0.0);
    tf_camera_link_to_color_optical.setRotation(q);
    
}

double AprilTagSingleNode::calculate_mean(const double& mean_history, int& number, double new_value)
{
    double mean = 0.0;
    double sum = mean_history * number + new_value;
    mean = sum / (number + 1);
    return mean;
}

void AprilTagSingleNode::timer_camera_info_callback()
{
    if (!camera_info_received_ && !camera_info_sub)
    {
        RCLCPP_INFO(get_logger(), "Attempting to subscribe to %s", cameraInfoTopic.c_str());
        camera_info_sub = this->create_subscription<CameraInfoT>(cameraInfoTopic, rclcpp::QoS(1).best_effort(), std::bind(&AprilTagSingleNode::onCameraInfo, this, std::placeholders::_1));
    }
}

void AprilTagSingleNode::onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg_ci)
{
    if (camera_info_received_) return;  // 防止重复处理
    camera_info_msg = *msg_ci;
    camera_info_received_ = true;
    RCLCPP_INFO(get_logger(), "camera_info topic %s received. stopping timer", cameraInfoTopic.c_str());

    if (timer_camera_info_) 
    {
        timer_camera_info_->cancel();
        timer_camera_info_.reset();
        camera_info_sub.reset();
    }
}

void AprilTagSingleNode::onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img)
{
    if (!camera_info_received_)
    {
        return;
    }
    std::array<double, 4> intrinsics = {camera_info_msg.k.data()[0], camera_info_msg.k.data()[4], camera_info_msg.k.data()[2], camera_info_msg.k.data()[5]};

    RCLCPP_DEBUG(get_logger(), "k => %f, %f, %f, %f", intrinsics[0], intrinsics[1], intrinsics[2], intrinsics[3]);
    
    // 添加数据有效性检查
    if (!msg_img || msg_img->width <= 0 || msg_img->height <= 0) {
        RCLCPP_ERROR(get_logger(), "Invalid image message received");
        return;
    }
    cv::Mat img_bgr8, img_uint8 ;
    // 使用try-catch结构提高代码健壮性
    try {
         img_bgr8 = cv_bridge::toCvShare(msg_img, "bgr8")->image;
         cv::cvtColor(img_bgr8, img_uint8, cv::COLOR_BGR2GRAY);
    } catch (const cv::Exception& e) {
        RCLCPP_ERROR(get_logger(), "OpenCV exception: %s", e.what());
        return;
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
                tf2::fromMsg(tf_msg, tf_marker2camera);
                auto tf_output = tf_marker2camera.inverse() * tf_camera_link_to_color_optical;
                auto origin = tf_output.getOrigin();
                auto rotation = tf_output.getRotation();
                translation_x = origin.getX();
                translation_y = origin.getY();
                translation_z = origin.getZ();
                tf2::Matrix3x3 mat(rotation);
                mat.getRPY(rotation_roll, rotation_pitch, rotation_yaw);

                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "-----------------------------------------------");
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "origin (x,y,z) : (%f %f %f)", translation_x, translation_y, translation_z);
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "rotation rpy   : (%f %f %f)", rotation_roll, rotation_pitch, rotation_yaw);
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "camera height  : %f", translation_z + boader_height);
                return;
            }
        }
    }
}

rcl_interfaces::msg::SetParametersResult
AprilTagSingleNode::onParameter(const std::vector<rclcpp::Parameter>& parameters)
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

bool AprilTagSingleNode::getTransform(
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




