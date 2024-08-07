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
#include <angles/angles.h>


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

    // parameter
    std::mutex mutex;
    double tag_edge_size;
    std::atomic<int> max_hamming;
    std::atomic<bool> profile;
    std::unordered_map<int, std::string> tag_frames;
    std::unordered_map<int, double> tag_sizes;

    std::function<void(apriltag_family_t*)> tf_destructor;

    const image_transport::CameraSubscriber sub_cam;
    const rclcpp::Publisher<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr pub_detections;
    tf2_ros::TransformBroadcaster tf_broadcaster;

    pose_estimation_f estimate_pose = nullptr;

    void onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img, const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_ci);

    rcl_interfaces::msg::SetParametersResult onParameter(const std::vector<rclcpp::Parameter>& parameters);


    tf2::Transform tf_real_to_dummy;
    tf2::Transform tf_base_link_to_dummy_base_link;
    bool getTransform(
        const std::string & refFrame, const std::string & childFrame,
        geometry_msgs::msg::TransformStamped & transform);
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::Publisher<aruco_msgs::msg::PoseWithId>::SharedPtr pose_with_id_pub;
    rclcpp::Publisher<aruco_msgs::msg::MarkerAndMacVector>::SharedPtr id_and_mac_pub;
    rclcpp::Publisher<capella_ros_service_interfaces::msg::ChargeMarkerVisible>::SharedPtr detect_status;
    rclcpp::TimerBase::SharedPtr id_mac_timer_;
    // rclcpp::TimerBase::SharedPtr id_selected_timer_;
    rclcpp::TimerBase::SharedPtr marker_timer;
    aruco_msgs::msg::MarkerAndMacVector msgs;
    aruco_msgs::msg::MarkerAndMac msg;
    std::string charger_id_;
    bool id_selected = false;
    // float id_selected_lifecycle_;
    std::vector<std::string> marker_id_and_bluetooth_mac_vector;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr charger_id_sub;
    zarray_t detections;
    int marker_id;
    int marker_id_correction;
    float marker_frame_translation;
    capella_ros_service_interfaces::msg::ChargeMarkerVisible marker_detect_status;
    std::string apriltag_family_name;
    

    void id_mac_callback();
    void charger_id_callback(std_msgs::msg::String msg);
    void marker_visible_callback();
    bool in_idRanges(std::vector<int> ids);

    tf2::Transform tf_marker1_to_charger;
    tf2::Transform tf_camera_to_marker1, tf_camera_to_marker2, tf_marker1_to_marker2_fixed;
    tf2::Stamped<tf2::Transform> stamped_tf_camera_to_marker1; 
    tf2::Stamped<tf2::Transform> stamped_tf_camera_to_marker2; 
    std::vector<std::pair<int, geometry_msgs::msg::TransformStamped>> id_and_tf_vec;
    float similarity_threshold;
    float radius_threshold;
    tf2::Stamped<tf2::Transform> camera_pose_last1;
    tf2::Stamped<tf2::Transform> camera_pose_last2;
    tf2::Stamped<tf2::Transform> camera_pose_current1;
    tf2::Stamped<tf2::Transform> camera_pose_current2;
    bool pose_inited = false;

    tf2::Transform tf_baselink_to_camera;

    // statistics
    int frame_all = 0;
    int frame_detected = 0;
    int frame_not_detected = 0;
    int frame_error = 0;

};

RCLCPP_COMPONENTS_REGISTER_NODE(AprilTagDoubleNode)


AprilTagDoubleNode::AprilTagDoubleNode(const rclcpp::NodeOptions& options)
  : Node("apriltag", options),
    // parameter
    cb_parameter(add_on_set_parameters_callback(std::bind(&AprilTagDoubleNode::onParameter, this, std::placeholders::_1))),
    td(apriltag_detector_create()),
    // topics
    sub_cam(image_transport::create_camera_subscription(
        this,
        this->get_node_topics_interface()->resolve_topic_name("image_rect"),
        std::bind(&AprilTagDoubleNode::onCamera, this, std::placeholders::_1, std::placeholders::_2),
        declare_parameter("image_transport", "raw", descr({}, true)),
        rmw_qos_profile_sensor_data)),
    pub_detections(create_publisher<apriltag_msgs::msg::AprilTagDetectionArray>("detections", rclcpp::QoS(1))),
    tf_broadcaster(this)
{
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // read-only parameters
    apriltag_family_name = declare_parameter("family", "36h11", descr("tag family", true));
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
    declare_parameter("marker_frame_translation", -0.04, descr("The translation distance on Y axis from charge frame to marker1 frame."));
    declare_parameter("similarity_threshold", 0.96, descr("similarity_threshold to check tf's validity"));
    declare_parameter("radius_threshold", 0.013, descr("radius_threshold to check tf's validity"));

    this->get_parameter_or<std::vector<std::string>>("marker_id_and_bluetooth_mac_vec", marker_id_and_bluetooth_mac_vector, {"0:1/94:C9:60:43:BE:01"});
    // RCLCPP_INFO(get_logger(), "marker_id_and_bluetooth_mac_vector.size(): %ld", marker_id_and_bluetooth_mac_vector.size());
    this->get_parameter_or<float>("marker_frame_translation", marker_frame_translation, -0.04);
    this->get_parameter_or<float>("similarity_threshold", similarity_threshold, 0.96);
    this->get_parameter_or<float>("radius_threshold", radius_threshold, 0.014);
    RCLCPP_INFO(get_logger(), "similarity_threshold: %f", similarity_threshold);
    RCLCPP_INFO(get_logger(), "radius_threshold: %f", radius_threshold);

	int id_mac_length = marker_id_and_bluetooth_mac_vector.size();
    RCLCPP_INFO(get_logger(), "marker_id_and_bluetooth_mac_vector size: %d", id_mac_length);
	for (int ids_index = 0; ids_index < id_mac_length; ids_index++)
	{
		std::string id_and_mac = marker_id_and_bluetooth_mac_vector[ids_index];
		int id_and_mac_length = id_and_mac.length();
		int pos = id_and_mac.find('/');
		int marker_id_, marker_id_correction_;
		std::string bluetooth_mac;

        // compute two markers ids and bluetooth mac;
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
        // use tag specific size
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

    // tf for marker to dummy marker
    tf_real_to_dummy.setIdentity();
    tf2::Quaternion q_marker_real_to_dummy;
    q_marker_real_to_dummy.setRPY(-M_PI / 2.0, M_PI / 2.0, 0.0);
    tf_real_to_dummy.setRotation(q_marker_real_to_dummy);

    // tf for base_link to dummy base_link
    tf_base_link_to_dummy_base_link.setIdentity();
    tf2::Quaternion q_base_link_to_dummy_base_link;
    q_base_link_to_dummy_base_link.setRPY(0.0, 0.0, M_PI);
    tf_base_link_to_dummy_base_link.setRotation(q_base_link_to_dummy_base_link);

    // generate tf from marker1 to charger.
    tf_marker1_to_charger.setIdentity();
    RCLCPP_INFO(get_logger(), "marker_frame_translation: %f", marker_frame_translation);
    tf_marker1_to_charger.setOrigin(tf2::Vector3(0., marker_frame_translation, 0.));
    tf2::Quaternion q_marker_to_charger;
    q_marker_to_charger.setRPY(0., 0., 0.);
    tf_marker1_to_charger.setRotation(q_marker_to_charger);

    // generate tf from marker1 to marker2
    tf_marker1_to_marker2_fixed.setIdentity();
    tf_marker1_to_marker2_fixed.setOrigin(tf2::Vector3(0., marker_frame_translation * 2, 0.));
    tf2::Quaternion q_marker1_to_marker2;
    q_marker1_to_marker2.setRPY(0., 0., 0.);
    tf_marker1_to_marker2_fixed.setRotation(q_marker1_to_marker2);


    pose_with_id_pub = this->create_publisher<aruco_msgs::msg::PoseWithId>("/pose_with_id", 100);
    detect_status = this->create_publisher<capella_ros_service_interfaces::msg::ChargeMarkerVisible>("marker_visible", 10);
    id_and_mac_pub = this->create_publisher<aruco_msgs::msg::MarkerAndMacVector>("/id_mac", 30);
    
    marker_timer = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&AprilTagDoubleNode::marker_visible_callback, this));
	id_mac_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&AprilTagDoubleNode::id_mac_callback, this));
    charger_id_sub =  this->create_subscription<std_msgs::msg::String>("/charger/id",rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local().reliable(),std::bind(&AprilTagDoubleNode::charger_id_callback, this, std::placeholders::_1));
}

AprilTagDoubleNode::~AprilTagDoubleNode()
{
    apriltag_detector_destroy(td);
    tf_destructor(tf);
}

void AprilTagDoubleNode::id_mac_callback()
{
	// RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 10000, "id_mac_callback");
    id_and_mac_pub->publish(msgs);
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
                // id_selected = true;
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

void AprilTagDoubleNode::marker_visible_callback()
{
	// RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000, "/marker_visible callback");
    detect_status->publish(marker_detect_status);        
}

void AprilTagDoubleNode::onCamera(const sensor_msgs::msg::Image::ConstSharedPtr& msg_img,
                            const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_ci)
{
    try
    {
    // init for tfs
    tf_camera_to_marker1.setIdentity();
    tf_camera_to_marker2.setIdentity();
    id_and_tf_vec = std::vector<std::pair<int, geometry_msgs::msg::TransformStamped>>();

    tf_baselink_to_camera.setIdentity();
    geometry_msgs::msg::TransformStamped stamped_tf_baselink_to_camera_msg;
    tf2::Stamped<tf2::Transform> stamped_tf_baselink_to_camera;
    if(getTransform(std::string("base_link"), std::string("camera3_color_optical_frame"), stamped_tf_baselink_to_camera_msg))
    {
        tf2::fromMsg(stamped_tf_baselink_to_camera_msg, stamped_tf_baselink_to_camera);
        tf_baselink_to_camera = static_cast<tf2::Transform>(stamped_tf_baselink_to_camera);
    }

    
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

        // 3D orientation and position
        geometry_msgs::msg::TransformStamped tf;
        tf.header = msg_img->header;
        // set child frame name by generic tag name or configured tag name
        tf.child_frame_id = tag_frames.count(det->id) ? tag_frames.at(det->id) : "april" + std::string(det->family->name) + ":" + std::to_string(det->id);
        const double size = tag_sizes.count(det->id) ? tag_sizes.at(det->id) : tag_edge_size;
        // RCLCPP_INFO(get_logger(), "size: %f", size);
        // RCLCPP_INFO(get_logger(), "tag_edge_size: %f", tag_edge_size);
        if(estimate_pose != nullptr) {
            tf.transform = estimate_pose(det, intrinsics, size);
        }
        
        std::pair<int, geometry_msgs::msg::TransformStamped> id_tf_pair;
        id_tf_pair.first = det->id;
        id_tf_pair.second = tf;
        id_and_tf_vec.push_back(id_tf_pair);
        // tfs.push_back(tf);
        tfs.push_back(stampedTransform_real_to_dummy);

        tf2::Stamped<tf2::Transform> dummyToBaselink;
        dummyToBaselink.setIdentity();

        // delete old topic , only pub valid topic from charger to baselink_dummy
        // aruco_msgs::msg::PoseWithId pose_with_id_msg;
        // pose_with_id_msg.pose.header.stamp = msg_img->header.stamp;
        // pose_with_id_msg.pose.header.frame_id = ss_child.str();
        // pose_with_id_msg.marker_id = det->id;

        // geometry_msgs::msg::TransformStamped transform_stamped;
        // if (getTransform(ss_child.str(), "base_link", transform_stamped))
        // {
        //     tf2::fromMsg(transform_stamped, dummyToBaselink);
        //     tf2::toMsg(static_cast<tf2::Transform>(dummyToBaselink) * tf_base_link_to_dummy_base_link, pose_with_id_msg.pose.pose);
        //     pose_with_id_pub->publish(pose_with_id_msg);       
        // }
    }

    
    // fix bug for reason:  marker_visible's state is out of sync 
    int detections_size = zarray_size(&detections);
    // RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 1000, "apriltag detected, size: %d", detections_size);
    frame_all++;
    if (detections_size < 2)
    {
        frame_not_detected++;
        // RCLCPP_INFO(get_logger(), "detections's size == 0");
        marker_detect_status.marker_visible = false;
        marker_detect_status.marker_id = -1;
        marker_detect_status.marker_id_correction = -1;
    }
    else
    {
        auto marker_id_vector = std::vector<int>();
        // RCLCPP_INFO(get_logger(), "detections's size > 0, size: %d", detections_size);
        for (int i = 0; i < detections_size; i++)
        {            
            apriltag_detection_t* det;
            zarray_get(&detections, i, &det);
            // RCLCPP_INFO(get_logger(), "det->haming: %d, max-haming: %d", det->hamming, max_hamming.load());
            if (det->hamming <= max_hamming.load())
            {
                marker_id_vector.push_back(det->id);
            }
        }
        // RCLCPP_INFO(get_logger(), "marker_id_vector size: %lu", marker_id_vector.size());
        // std::stringstream ss;
        // for (size_t i = 0; i < marker_id_vector.size(); i++)
        // {
        //     ss << marker_id_vector[i] << " ";
        // }
        // RCLCPP_INFO(get_logger(), "marker_id_vector: %s", ss.str().c_str());
        if (id_selected)
        {
            // RCLCPP_INFO(this->get_logger(), "id_selected");
            // RCLCPP_INFO(this->get_logger(), "marker_id: %d", marker_id);
            // RCLCPP_INFO(this->get_logger(), "marker_id_correction: %d", marker_id_correction);
            
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
            // RCLCPP_INFO(this->get_logger(), "id not selected");
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
    
    if (marker_detect_status.marker_visible)
    {
        frame_detected++;
        // add tf from charger_frame to marker1 frame
        geometry_msgs::msg::TransformStamped stampedTransform_marker1_to_charger;
        std::stringstream ss_marker1_frame;
        ss_marker1_frame << "apriltag" << apriltag_family_name << ":" << marker_id << "_dummy";
        stampedTransform_marker1_to_charger.header.frame_id = ss_marker1_frame.str() ;
        stampedTransform_marker1_to_charger.header.stamp = msg_img->header.stamp;
        stampedTransform_marker1_to_charger.child_frame_id = std::string("charger");
        tf2::toMsg(tf_marker1_to_charger, stampedTransform_marker1_to_charger.transform);
        tfs.push_back(stampedTransform_marker1_to_charger);

        // pub valid marker1 and marker2(optional) 
        size_t tf_size = id_and_tf_vec.size();
        int index_marker1 = 0, index_marker2 = 0;
        for (size_t i = 0; i < tf_size; i++)
        {
            if (id_and_tf_vec[i].first == marker_id)
            {
                index_marker1 = i;
                // RCLCPP_INFO(get_logger(), "inex_marker1: %d", index_marker1);               
                tf2::fromMsg(id_and_tf_vec[i].second, stamped_tf_camera_to_marker1);
                tf_camera_to_marker1 = static_cast<tf2::Transform>(stamped_tf_camera_to_marker1);
            }
            if (id_and_tf_vec[i].first == marker_id_correction)
            {
                index_marker2 = i;
                // RCLCPP_INFO(get_logger(), "inex_marker2: %d", index_marker2);  
                tf2::fromMsg(id_and_tf_vec[i].second, stamped_tf_camera_to_marker2);
                tf_camera_to_marker2 = static_cast<tf2::Transform>(stamped_tf_camera_to_marker2);
            }
        }
        tf2::Transform tf_marker1_to_marker2_current;
        tf_marker1_to_marker2_current = tf_real_to_dummy.inverse() * tf_camera_to_marker1.inverse() * tf_camera_to_marker2 * tf_real_to_dummy ;
        
        // for debug error
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

        // get tf from marker1_to_marker2_fixed to marker1_to_marker2_current
        auto tf_fixed_to_current = tf_marker1_to_marker2_fixed.inverse() * tf_marker1_to_marker2_current;
        float error_x, error_y, error_z;
        error_x = tf_fixed_to_current.getOrigin()[0];
        error_y = tf_fixed_to_current.getOrigin()[0];
        error_z = tf_fixed_to_current.getOrigin()[0];
        float error_radius;
        error_radius = std::hypot(std::hypot(error_x, error_y), error_z);
        

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
        float similarity = w_f * w_c + x_f * x_c + y_f * y_c + z_f * z_c;

        if (similarity > similarity_threshold && error_radius < radius_threshold)
        {
            tfs.push_back(id_and_tf_vec[index_marker1].second);
            tfs.push_back(id_and_tf_vec[index_marker2].second);

            auto end_time = this->get_clock()->now().seconds();
            auto start_time = rclcpp::Time(msg_img->header.stamp).seconds();
            auto delta_time = end_time - start_time;
            RCLCPP_DEBUG(get_logger(), "cost time: %f second.", delta_time);

            // pub topic /pose_with_id
            aruco_msgs::msg::PoseWithId pose_with_id_msg;
            pose_with_id_msg.pose.header.stamp = msg_img->header.stamp;
            pose_with_id_msg.pose.header.frame_id = std::string("charger");
            pose_with_id_msg.marker_id = marker_id;
            pose_with_id_msg.similarity = similarity;
            pose_with_id_msg.radius = error_radius;
            auto tf_charger_to_baselink_dummy = tf_marker1_to_charger.inverse() * tf_real_to_dummy.inverse()
                * tf_camera_to_marker1.inverse() * tf_baselink_to_camera.inverse() * tf_base_link_to_dummy_base_link ;
            tf2::toMsg(tf_charger_to_baselink_dummy, pose_with_id_msg.pose.pose);
            pose_with_id_pub->publish(pose_with_id_msg);
        }
        else
        {
            frame_error++;
            if (similarity <= similarity_threshold)
            {
                RCLCPP_INFO(get_logger(), "error tf detected, similarity  : %f, threshold: %f", similarity, similarity_threshold);
            }
            if (error_radius >= radius_threshold)
            {
                RCLCPP_INFO(get_logger(), "error tf detected, error_raidus: %f, threshold: %f", error_radius, radius_threshold);
            }
        }

        
        // just for test data
        // if (similarity > similarity_threshold)
        // {
        //     // tfs.push_back(id_and_tf_vec[index_marker1].second);
        //     // tfs.push_back(id_and_tf_vec[index_marker2].second);

        //     // debug data
        //     camera_pose_current1 = stamped_tf_camera_to_marker1;
        //     camera_pose_current2 = stamped_tf_camera_to_marker2;
        //     if (!pose_inited)
        //     {
        //         pose_inited = true;
        //         camera_pose_last1 = camera_pose_current1;
        //         camera_pose_last2 = camera_pose_current2;
        //     }
        //     else
        //     {
        //         float x_last1, y_last1, theta_last1, x_now1, y_now1, theta_now1;
        //         float x_last2, y_last2, theta_last2, x_now2, y_now2, theta_now2;
        //         float distance1, distance2, theta_dis1, theta_dis2;

        //         // marker1_dummy to baselink_dummy
        //         auto tf_marker_to_camera_last1 = tf_real_to_dummy.inverse() * static_cast<tf2::Transform>(camera_pose_last1).inverse()
        //              * tf_baselink_to_camera.inverse() * tf_base_link_to_dummy_base_link;
        //         x_last1 = static_cast<tf2::Transform>(tf_marker_to_camera_last1).getOrigin()[0];
        //         y_last1 = static_cast<tf2::Transform>(tf_marker_to_camera_last1).getOrigin()[1];
        //         theta_last1 = tf2::getYaw(tf_marker_to_camera_last1.getRotation());
                
        //         auto tf_marker_to_camera_current1 = tf_real_to_dummy.inverse() * static_cast<tf2::Transform>(camera_pose_current1).inverse()
        //             * tf_baselink_to_camera.inverse() * tf_base_link_to_dummy_base_link;
        //         x_now1 = static_cast<tf2::Transform>(tf_marker_to_camera_current1).getOrigin()[0];
        //         y_now1 = static_cast<tf2::Transform>(tf_marker_to_camera_current1).getOrigin()[1];
        //         theta_now1 = tf2::getYaw(tf_marker_to_camera_current1.getRotation());
        //         distance1 = std::hypot(x_last1 - x_now1, y_last1 - y_now1);
        //         theta_dis1 = std::abs(angles::shortest_angular_distance(theta_last1, theta_now1));
        //         if (theta_dis1 > 0.2 || distance1 > 0.2)
        //         {
        //             RCLCPP_INFO(get_logger(), "----------------------------------");
        //             RCLCPP_INFO(get_logger(), "x_last1     : %f, y_last1: %f, theta_last1: %f", x_last1, y_last1, theta_last1);
        //             RCLCPP_INFO(get_logger(), "x_now1      : %f, y_now1 : %f, theta_now1 : %f", x_now1, y_now1, theta_now1);
        //             RCLCPP_INFO(get_logger(), "distanc1    : %f", distance1);
        //             RCLCPP_INFO(get_logger(), "theta_dis1  : %f", theta_dis1);
        //             RCLCPP_INFO(get_logger(), "similarity  : %f", similarity);
        //             RCLCPP_INFO(get_logger(), "radius error: %f", error_radius);
        //             RCLCPP_INFO(get_logger(), "m1_m2 tf_c, x: %f, y: %f, theta_diff: %f", 
        //                     tf_marker1_to_marker2_current.getOrigin()[0], tf_marker1_to_marker2_current.getOrigin()[1],
        //                     tf2::getYaw(tf_marker1_to_marker2_current.getRotation()));
        //         }
        //         else
        //         {
        //             RCLCPP_INFO(get_logger(), "----------------------------------");

        //             RCLCPP_INFO(get_logger(), "m1_m2 tf_c, x: %f, y: %f, theta_diff: %f", 
        //                     tf_marker1_to_marker2_current.getOrigin()[0], tf_marker1_to_marker2_current.getOrigin()[1],
        //                     tf2::getYaw(tf_marker1_to_marker2_current.getRotation()));
        //             // RCLCPP_INFO(get_logger(), "radius error: %f", error_radius);
        //             // RCLCPP_INFO(get_logger(), "similarity  : %f", similarity);
        //             // RCLCPP_INFO(get_logger(), "marker1_to_marker2 tf_current, theta_same: %f", tf2::getYaw(tf_marker1_to_marker2_current.getRotation()));
        //             if (error_radius > 0.04)
        //             {
        //                 RCLCPP_INFO(get_logger(), "==================================");
        //                 RCLCPP_INFO(get_logger(), "radius error: %f", error_radius);
        //                 float x_c, y_c, z_c, theta_c;
        //                 x_c = tf_marker1_to_marker2_current.getOrigin()[0];
        //                 y_c = tf_marker1_to_marker2_current.getOrigin()[1];
        //                 z_c = tf_marker1_to_marker2_current.getOrigin()[2];
        //                 theta_c = tf2::getYaw(tf_marker1_to_marker2_current.getRotation());
        //                 RCLCPP_INFO(get_logger(), "x_c: %f, y_c: %f, z_c: %f, theta_c: %f", x_c, y_c, z_c, theta_c);
        //             }
        //             else
        //             {
        //                 // RCLCPP_INFO(get_logger(), "**********************************");
        //                 // RCLCPP_INFO(get_logger(), "radius error: %f", error_radius);
        //             }
        //         }

        //         // marker2_dummy to baselink_dummy
        //         auto tf_marker_to_camera_last2 = tf_real_to_dummy.inverse() * static_cast<tf2::Transform>(camera_pose_last2).inverse()
        //             * tf_baselink_to_camera.inverse() * tf_base_link_to_dummy_base_link;
        //         x_last2 = static_cast<tf2::Transform>(tf_marker_to_camera_last2).getOrigin()[0];
        //         y_last2 = static_cast<tf2::Transform>(tf_marker_to_camera_last2).getOrigin()[1];
        //         theta_last2 = tf2::getYaw(tf_marker_to_camera_last2.getRotation());
                
        //         auto tf_marker_to_camera_current2 = tf_real_to_dummy.inverse() * static_cast<tf2::Transform>(camera_pose_current2).inverse()
        //             * tf_baselink_to_camera.inverse() * tf_base_link_to_dummy_base_link;
        //         x_now2 = static_cast<tf2::Transform>(tf_marker_to_camera_current2).getOrigin()[0];
        //         y_now2 = static_cast<tf2::Transform>(tf_marker_to_camera_current2).getOrigin()[1];
        //         theta_now2 = tf2::getYaw(tf_marker_to_camera_current2.getRotation());
        //         distance2 = std::hypot(x_last2 - x_now2, y_last2 - y_now2);
        //         theta_dis2 = std::abs(angles::shortest_angular_distance(theta_last2, theta_now2));
        //         if (theta_dis2 > 0.3 || distance2 > 0.3)
        //         {
        //             // RCLCPP_INFO(get_logger(), "-----------------------------------");
        //             // RCLCPP_INFO(get_logger(), "x_last2     : %f, y_last2: %f, theta_last2: %f", x_last2, y_last2, theta_last2);
        //             // RCLCPP_INFO(get_logger(), "x_now2      : %f, y_now2 : %f, theta_now2 : %f", x_now2, y_now2, theta_now2);
        //             // RCLCPP_INFO(get_logger(), "distanc2    : %f", distance2);
        //             // RCLCPP_INFO(get_logger(), "theta_dis2  : %f", theta_dis2);
        //             // RCLCPP_INFO(get_logger(), "similarity  : %f", similarity);
        //             // RCLCPP_INFO(get_logger(), "radius error: %f", error_radius);
        //         }

        //         camera_pose_last1 = camera_pose_current1;
        //         camera_pose_last2 = camera_pose_current2;

        //     }
        // }
        
    }

    // add tf from base_link to base_link_dummy
    geometry_msgs::msg::TransformStamped tf_baselink_to_baselink_dummy_msg;
    tf_baselink_to_baselink_dummy_msg.header.frame_id = std::string("base_link");
    tf_baselink_to_baselink_dummy_msg.header.stamp = msg_img->header.stamp;
    tf_baselink_to_baselink_dummy_msg.child_frame_id = std::string("base_link_dummy");
    tf2::toMsg(tf_base_link_to_dummy_base_link, tf_baselink_to_baselink_dummy_msg.transform);
    tfs.push_back(tf_baselink_to_baselink_dummy_msg);

    pub_detections->publish(msg_detections);
    tf_broadcaster.sendTransform(tfs);

    // apriltag_detections_destroy(&detections);
    }
    catch(const char* msg)
    {
        RCLCPP_INFO(get_logger(), "error: %s", msg);
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
