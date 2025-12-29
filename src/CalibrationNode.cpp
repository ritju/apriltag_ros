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

class AprilTagNode : public rclcpp::Node {
public:
    AprilTagNode(const rclcpp::NodeOptions& options);

    ~AprilTagNode() override;

private:
    const OnSetParametersCallbackHandle::SharedPtr cb_parameter;

    apriltag_family_t* tf;
    apriltag_detector_t* const td;

    // parameter
    std::mutex mutex;
    double tag_edge_size;
    std::atomic<int> max_hamming;
    std::atomic<bool> profile;
    std::unordered_map<int, std::string> tag_frames;
    std::unordered_map<int, double> tag_sizes;
    // tf_base_link_to_marker
    double marker_translation_x, marker_translation_y, marker_translation_z;
    double marker_roll{0.0}, marker_pitch{0.0}, marker_yaw{0.0};
    // tf_base_link_to_camera;
    double camera_translation_x, camera_translation_y, camera_translation_z;
    double camera_roll, camera_pitch, camera_yaw;

    std::function<void(apriltag_family_t*)> tf_destructor;

    const image_transport::CameraSubscriber sub_cam;
    const rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr pub_detections;
    tf2_ros::TransformBroadcaster tf_broadcaster;

    pose_estimation_f estimate_pose = nullptr;

    void onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_ci);

    rcl_interfaces::msg::SetParametersResult onParameter(const std::vector<rclcpp::Parameter>& parameters);


    tf2::Transform tf_real_to_dummy;
    tf2::Transform tf_base_link_to_marker_dummy;
    tf2::Transform tf_base_link_to_camera;
    bool getTransform(
        const std::string & refFrame, const std::string & childFrame,
        geometry_msgs::msg::TransformStamped & transform);
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::Publisher<capella_ros_service_interfaces::msg::ChargeMarkerVisible>::SharedPtr detect_status;
    // rclcpp::TimerBase::SharedPtr id_selected_timer_;
    rclcpp::TimerBase::SharedPtr marker_timer;
    aruco_msgs::msg::MarkerAndMacVector msgs;
    aruco_msgs::msg::MarkerAndMac msg;
    std::string charger_id_;
    bool id_selected = false;
    // float id_selected_lifecycle_;
    std::vector<std::string> marker_id_and_bluetooth_mac_vector;
    rclcpp::Time charger_id_time_start;
    zarray_t detections;
    int marker_id;
    
    bool marker_visible_last = false;
    bool marker_visible_pub = false;

    void marker_visible_callback();
};

RCLCPP_COMPONENTS_REGISTER_NODE(AprilTagNode)


AprilTagNode::AprilTagNode(const rclcpp::NodeOptions& options)
  : Node("apriltag", options),
    // parameter
    cb_parameter(add_on_set_parameters_callback(std::bind(&AprilTagNode::onParameter, this, std::placeholders::_1))),
    td(apriltag_detector_create()),
    // topics
    sub_cam(image_transport::create_camera_subscription(
        this,
        this->get_node_topics_interface()->resolve_topic_name("image_rect"),
        std::bind(&AprilTagNode::onCamera, this, std::placeholders::_1, std::placeholders::_2),
        declare_parameter("image_transport", "raw", descr({}, true)),
        rmw_qos_profile_sensor_data)),
    pub_detections(create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>("detections", rclcpp::QoS(1))),
    tf_broadcaster(this)
{
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

    declare_parameter("marker_id_and_bluetooth_mac_vec", std::vector<std::string>(), descr("the vector of marker id and bluetooth mac"));

    // tf_base_link_to_marker
    marker_translation_x = declare_parameter("marker_translation_x", 2.0, descr("default marker_translation_x", true));
    marker_translation_y = declare_parameter("marker_translation_y", 0.0, descr("default marker_translation_y", true));
    marker_translation_z = declare_parameter("marker_translation_z", 1.0, descr("default marker_translation_z", true));
    marker_roll = declare_parameter("marker_roll", 0.0, descr("default marker_roll", true));
    marker_pitch = declare_parameter("marker_pitch", 0.0, descr("default marker_pitch", true));
    marker_yaw = declare_parameter("marker_yaw", 0.0, descr("default marker_yaw", true));

    this->get_parameter_or<std::vector<std::string>>("marker_id_and_bluetooth_mac_vec", marker_id_and_bluetooth_mac_vector, {"0/94:C9:60:43:BE:07"});
    // RCLCPP_INFO(get_logger(), "marker_id_and_bluetooth_mac_vector.size(): %ld", marker_id_and_bluetooth_mac_vector.size());

	int id_mac_length = marker_id_and_bluetooth_mac_vector.size();
	for (int ids_index = 0; ids_index < id_mac_length; ids_index++)
	{
		std::string id_and_mac = marker_id_and_bluetooth_mac_vector[ids_index];
		int id_and_mac_length = id_and_mac.length();
		int pos = id_and_mac.find('/');
		int marker_id_;
		std::string bluetooth_mac;
		marker_id_ = atoi(id_and_mac.substr(0, pos).c_str());
		bluetooth_mac = id_and_mac.substr(pos + 1, id_and_mac_length - pos -1);
		RCLCPP_INFO(this->get_logger(), "marker_id_: %d", marker_id_);
		RCLCPP_INFO(this->get_logger(), "bluetooth_mac: %s", bluetooth_mac.c_str());
		msg.marker_id = marker_id_;
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

    // tf for marker to dummy marker
    tf_real_to_dummy.setIdentity();
    tf2::Quaternion q_marker_real_to_dummy;
    q_marker_real_to_dummy.setRPY(-M_PI / 2.0, M_PI / 2.0, 0.0);
    tf_real_to_dummy.setRotation(q_marker_real_to_dummy);

    // tf for base_link to marker_dummy
    tf_base_link_to_marker_dummy.setIdentity();
    tf2::Quaternion q_base_link_to_marker_dummy;
    q_base_link_to_marker_dummy.setRPY(marker_roll, marker_pitch, marker_yaw);
    tf_base_link_to_marker_dummy.setOrigin(tf2::Vector3(marker_translation_x, marker_translation_y, marker_translation_z));
    tf_base_link_to_marker_dummy.setRotation(q_base_link_to_marker_dummy);

    detect_status = this->create_publisher<capella_ros_service_interfaces::msg::ChargeMarkerVisible>("marker_visible", rclcpp::QoS(1).reliable().transient_local());
    
    marker_timer = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&AprilTagNode::marker_visible_callback, this));
}

AprilTagNode::~AprilTagNode()
{
    apriltag_detector_destroy(td);
    tf_destructor(tf);
}

void AprilTagNode::marker_visible_callback()
{
	// RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000, "/marker_visible callback");
    capella_ros_service_interfaces::msg::ChargeMarkerVisible marker_detect_status;

    int detections_size = zarray_size(&detections);
    // RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000, "apriltag detected, size: %d", detections_size);
    if (detections_size == 0)
    {
        marker_detect_status.marker_visible = false;
    }
    else if (detections_size > 0)
    {
        marker_detect_status.marker_visible = true;        
    }

    if (!marker_visible_pub)
    {
        detect_status->publish(marker_detect_status);
        marker_visible_pub = true;
        marker_visible_last = marker_detect_status.marker_visible;
    }
    else
    {
        if (marker_visible_last != marker_detect_status.marker_visible)
        {
            detect_status->publish(marker_detect_status);
            marker_visible_last = marker_detect_status.marker_visible;
        }
    }
}

void AprilTagNode::onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img,
                            const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_ci)
{
    // camera intrinsics for rectified images
    const std::array<double, 4> intrinsics = {msg_ci->p.data()[0], msg_ci->p.data()[5], msg_ci->p.data()[2], msg_ci->p.data()[6]};

    // convert to 8bit monochrome image
    const cv::Mat img_uint8 = cv_bridge::toCvShare(msg_img, "mono8")->image;

    image_u8_t im{img_uint8.cols, img_uint8.rows, img_uint8.cols, img_uint8.data};

    // detect tags
    mutex.lock();
    double start_time = this->now().seconds();
    detections = *apriltag_detector_detect(td, &im);
    double end_time = this->now().seconds();
    RCLCPP_DEBUG(get_logger(), "compute detections cost time: %d ms", (int)round((end_time - start_time) * 1000));
    mutex.unlock();

    if(profile)
        timeprofile_display(td->tp);

    apriltag_msgs::msg::AprilTagDetectionArray msg_detections;
    msg_detections.header = msg_img->header;

    std::vector<geometry_msgs::msg::TransformStamped> tfs;
    // RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000, "detections size: %d", zarray_size(&detections) );

    for(int i = 0; i < zarray_size(&detections); i++) {
        apriltag_detection_t* det;
        zarray_get(&detections, i, &det);

        RCLCPP_DEBUG(get_logger(),
                     "detection %3d: id (%2dx%2d)-%-4d, hamming %d, margin %8.3f\n",
                     i, det->family->nbits, det->family->h, det->id,
                     det->hamming, det->decision_margin);

        // ignore untracked tags
        if(!tag_frames.empty() && !tag_frames.count(det->id)) { continue; }

        // reject detections with more corrected bits than allowed
        if(det->hamming > max_hamming) { continue; }

        // detection
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

        // tf from real to dummy
        geometry_msgs::msg::TransformStamped stampedTransform_real_to_dummy;
        std::stringstream ss_parent, ss_child;
        ss_parent << "april" << det->family->name << ":" << det->id;
        ss_child << "april" << det->family->name << ":" << det->id << "_dummy";
        stampedTransform_real_to_dummy.header.frame_id = ss_parent.str();
        stampedTransform_real_to_dummy.header.stamp = msg_img->header.stamp;
        stampedTransform_real_to_dummy.child_frame_id = ss_child.str();
        tf2::toMsg(tf_real_to_dummy, stampedTransform_real_to_dummy.transform);

        // tf from base_link to marker_dummy
        geometry_msgs::msg::TransformStamped stampedTransform_base_link_to_marker_dummy;
        ss_child << "april" << det->family->name << ":" << det->id << "_dummy";
        stampedTransform_base_link_to_marker_dummy.header.frame_id = "base_link";
        stampedTransform_base_link_to_marker_dummy.header.stamp = msg_img->header.stamp;
        stampedTransform_base_link_to_marker_dummy.child_frame_id = ss_child.str();
        tf2::toMsg(tf_base_link_to_marker_dummy, stampedTransform_base_link_to_marker_dummy.transform);

        // 3D orientation and position
        geometry_msgs::msg::TransformStamped tf;
        tf.header = msg_img->header;
        // set child frame name by generic tag name or configured tag name
        tf.child_frame_id = tag_frames.count(det->id) ? tag_frames.at(det->id) : "april" + std::string(det->family->name) + ":" + std::to_string(det->id);
        const double size = tag_sizes.count(det->id) ? tag_sizes.at(det->id) : tag_edge_size;
        if(estimate_pose != nullptr) {
            tf.transform = estimate_pose(det, intrinsics, size, shared_from_this());
        }

        tfs.push_back(tf); // camera=>marker
        tfs.push_back(stampedTransform_real_to_dummy);  // marker=>marker_dummy
        tfs.push_back(stampedTransform_base_link_to_marker_dummy); // base_link=>marker_dumy

        tf2::Transform tf_camera_to_marker;
        tf2::fromMsg(tf.transform, tf_camera_to_marker);
        tf_base_link_to_camera = tf_base_link_to_marker_dummy * tf_real_to_dummy.inverse() * tf_camera_to_marker.inverse();
        
        auto translation = tf_base_link_to_camera.getOrigin();
        camera_translation_x = translation.getX();
        camera_translation_y = translation.getY();
        camera_translation_z = translation.getZ();
        auto rotation = tf_base_link_to_camera.getRotation();
        tf2::Matrix3x3 mat(rotation);
        mat.getRPY(camera_roll, camera_pitch, camera_yaw);
        RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000, "translation: (%.2f, %.2f, %.2f)", camera_translation_x, camera_translation_y, camera_translation_z);
        RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000, "rotation   : (%.2f, %.2f, %.2f)", camera_roll, camera_pitch, camera_yaw);
    }

    pub_detections->publish(msg_detections);
    tf_broadcaster.sendTransform(tfs);

    // apriltag_detections_destroy(&detections);
}

rcl_interfaces::msg::SetParametersResult
AprilTagNode::onParameter(const std::vector<rclcpp::Parameter>& parameters)
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

bool AprilTagNode::getTransform(
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
