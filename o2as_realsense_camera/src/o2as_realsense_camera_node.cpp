#include <ros/ros.h>
#include <librealsense2/rs.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <std_srvs/Empty.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "o2as_realsense_camera/GetFrame.h"
#include "o2as_realsense_camera/DumpFrame.h"

namespace o2as {

class RealSenseCameraNode
{
public:
    RealSenseCameraNode() : nh_("~"), image_transport_(nh_)
	{
		// Initialize camera parameters
		depth_scale_ = 0.0;
		memset(inv_param_, 0, 9 * sizeof(double));
		configure();

		// Pubilsh static transform
		publishStaticTransforms();

		// Activate
		active_ = false;
		activate();
	}

	~RealSenseCameraNode()
	{
		deactivate();
	}

protected:
	using Point = pcl::PointXYZ;
	using PointCloud = pcl::PointCloud<Point>;

	/// Load parameters and connect to the camera
	int configure() 
	{
		// load ROS parameters
		ros::param::get("~serial_number", serial_number_);
		ros::param::get("~color_width", color_width_);
		ros::param::get("~color_height", color_height_);
		ros::param::get("~depth_width", depth_width_);
		ros::param::get("~depth_height", depth_height_);
		ros::param::get("~camera_frame", camera_frame_);
		ros::param::get("~publish_images", publish_images_);
		ros::param::get("~camera_data_path", camera_data_path_);
		ros::param::get("~color_image_filename", color_image_filename_);
		ros::param::get("~depth_image_filename", depth_image_filename_);
		ros::param::get("~point_cloud_filename", point_cloud_filename_);

		// Enable camera
		if (!deviceExists(serial_number_)) {
			ROS_ERROR("realsense device with serial number %s not exists", serial_number_.c_str());
		}

		config_.enable_device(serial_number_);
		config_.enable_stream(rs2_stream::RS2_STREAM_COLOR, color_width_, color_height_, rs2_format::RS2_FORMAT_BGR8);
		config_.enable_stream(rs2_stream::RS2_STREAM_DEPTH, depth_width_, depth_height_, rs2_format::RS2_FORMAT_Z16 );
		return 0;
	}

	bool deviceExists(const std::string& serial)
	{
		rs2::context ctx;    
		for (auto&& dev : ctx.query_devices()) 
		{
			std::string s = std::string(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
			if (s == serial) {
				return true;
			}
		}
		return false;
	}

	//###################################################### 
	// camera info
	//###################################################### 

	rs2_intrinsics getColorCameraIntrinsics()
	{
		rs2::pipeline_profile pipeline_profile = pipe_.get_active_profile();
		rs2::stream_profile color_profile = pipeline_profile.get_stream(rs2_stream::RS2_STREAM_COLOR);
		rs2::video_stream_profile video_stream_profile_color = color_profile.as<rs2::video_stream_profile>();
		rs2_intrinsics intrinsics_color = video_stream_profile_color.get_intrinsics();
		return intrinsics_color;
	}

	void invMatrix3x3(float src[9], double dst[9])
	{
		double det = (double)src[0] * src[4] * src[8] + src[3] * src[7] * src[2] + src[6] * src[1] * src[5] - src[0] * src[7] * src[5] - src[6] * src[4] * src[2] - src[3] * src[1] * src[8];
		if (fabs(det) < 1e-6) {
			memset(dst, 0, 9 * sizeof(double));
		}
		else {
			double inv_det = 1.0 / det;
			dst[0] = ((double)src[4] * src[8] - src[5] * src[7]) * inv_det;
			dst[1] = ((double)src[2] * src[7] - src[1] * src[8]) * inv_det;
			dst[2] = ((double)src[1] * src[5] - src[2] * src[4]) * inv_det;
			dst[3] = ((double)src[5] * src[6] - src[3] * src[8]) * inv_det;
			dst[4] = ((double)src[0] * src[8] - src[2] * src[6]) * inv_det;
			dst[5] = ((double)src[2] * src[3] - src[0] * src[5]) * inv_det;
			dst[6] = ((double)src[3] * src[7] - src[4] * src[6]) * inv_det;
			dst[7] = ((double)src[1] * src[6] - src[0] * src[7]) * inv_det;
			dst[8] = ((double)src[0] * src[4] - src[1] * src[3]) * inv_det;
		}
	}

	bool prepareConversion()
	{
		rs2::pipeline_profile pipeline_profile = pipe_.get_active_profile();

		// depth_scale
		rs2::depth_sensor depth_sensor = pipeline_profile.get_device().first<rs2::depth_sensor>();
		depth_scale_ = (double)depth_sensor.get_depth_scale();

		// inv_matrix
		rs2_intrinsics intrinsics_color = getColorCameraIntrinsics();
		float cam_param[9] = {intrinsics_color.fx, 0.0, intrinsics_color.ppx, 0.0, intrinsics_color.fy, intrinsics_color.ppy, 0.0, 0.0, 1.0};
		invMatrix3x3(cam_param, inv_param_);
	}

	//###################################################### 
	// Activate / Deactivate
	//###################################################### 

	/// Activate ROS service servers and publishers
	int activate()
	{
		// start pipeline
		rs2::pipeline_profile pipeline_profile = pipe_.start(config_);
		prepareConversion();

		// activate service servers
		servers_.get_frame = nh_.advertiseService("get_frame" , &RealSenseCameraNode::getFrameCallback , this);
		servers_.dump_frame = nh_.advertiseService("dump_frame" , &RealSenseCameraNode::dumpFrameCallback , this);

		// activate publishers
		publishers_.color_image = image_transport_.advertise("color", 1, true);
		publishers_.depth_image = image_transport_.advertise("depth", 1, true);
		publishers_.point_cloud = nh_.advertise<PointCloud>("cloud", 1, true);

		// start image publishing timer
		if (publish_images_) {
			publish_images_timer_ = nh_.createTimer(ros::Rate(30), &RealSenseCameraNode::publishImageCallback, this);
		}

		active_ = true;
		return 0;
	}

	void deactivate() 
	{
		if (active_ == false) {
			return;
		}
		active_ = false;

		// Deactivate service servers
		servers_.get_frame.shutdown();
		servers_.dump_frame.shutdown();
		// Deactivate publishers
		publishers_.color_image.shutdown();
		publishers_.depth_image.shutdown();
		publishers_.point_cloud.shutdown();
		// Deactivate timers
		publish_images_timer_.stop();
		// Release the camera
		pipe_.stop();
	}

	//###################################################### 
	// Capture / Dump / Publish
	//###################################################### 

	std_msgs::Header getHeader()
	{
		std_msgs::Header header;
		header.frame_id = camera_frame_;
		header.stamp    = ros::Time::now();
		return header;		
	}

	bool getAlignedFrame(rs2::frame& color_frame, rs2::frame& depth_frame)
	{
		try 
		{
			rs2::frameset frameset = pipe_.wait_for_frames();
			rs2::align align(rs2_stream::RS2_STREAM_COLOR);
			rs2::frameset aligned_frameset = align.process(frameset);
			if (!aligned_frameset.size()) {
				return false;
			}
			color_frame = aligned_frameset.get_color_frame();
			depth_frame = aligned_frameset.get_depth_frame();
		}
		catch (const char *error_message) {
			ROS_ERROR("%s", error_message);
			return false;
		}
		return true;
	}

	inline void rsFrameToCvMat(rs2::frame& frame, int format, cv::Mat& cv_image)
	{
		int width  = frame.as<rs2::video_frame>().get_width();
		int height = frame.as<rs2::video_frame>().get_height();
		cv_image = cv::Mat(height, width, format, const_cast<void*>(frame.get_data()));
	}

	inline void getCvColorImage(rs2::frame& frame, cv::Mat& cv_image)
	{
		rsFrameToCvMat(frame, CV_8UC3, cv_image);
	}

	inline void getCvDepthImage(rs2::frame& frame, cv::Mat& cv_image)
	{
		rsFrameToCvMat(frame, CV_16SC1, cv_image);
	}

	/// conversion from depth image to point cloud using realsense2 sdk
	PointCloud::Ptr getPointCloud(rs2::frame& depth_frame)
	{
		rs2::pointcloud pc;
		rs2::points points = pc.calculate(depth_frame);
		PointCloud::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		auto sp = points.get_profile().as<rs2::video_stream_profile>();
		cloud->width = sp.width();
		cloud->height = sp.height();
		cloud->is_dense = false;
		cloud->points.resize(points.size());
		auto ptr = points.get_vertices();
		for (auto& p : cloud->points)
		{
			p.x = ptr->x;
			p.y = ptr->y;
			p.z = ptr->z;
			ptr++;
		}
		return cloud;
	}

	/// Conversion from depth image to point cloud using camera parameter
	PointCloud::Ptr getPointCloud2(rs2::frame& depth_frame)
	{
		// realsense frame to cv image
		int width  = depth_frame.as<rs2::video_frame>().get_width();
		int height = depth_frame.as<rs2::video_frame>().get_height();
		cv::Mat cv_depth_image = cv::Mat(height, width, CV_16SC1, const_cast<void*>(depth_frame.get_data()));

		// cv image to point cloud
		PointCloud::Ptr cloud(new PointCloud);
		cloud->width    = width;
		cloud->height   = height;
		cloud->is_dense = true;
		cloud->points.resize(width * height);
		unsigned short *ptr_depth;

		int pos = 0;
		double proj_x, proj_y, depth;
		const float max_depth = 1e+6f + 1.0f;
		for (int y = 0; y < height; y++) {
			ptr_depth = cv_depth_image.ptr<unsigned short>(y);
			for (int x = 0; x < width; x++) {
				proj_x = inv_param_[0] * (double)x + inv_param_[1] * (double)y + inv_param_[2];
				proj_y = inv_param_[3] * (double)x + inv_param_[4] * (double)y + inv_param_[5];
				depth  = depth_scale_  * (double)ptr_depth[x];
				cloud->points[pos].x = (float)(proj_x * depth); 
				cloud->points[pos].y = (float)(proj_y * depth);
				cloud->points[pos].z = (float)depth;
				pos++;
			}
		}
		return cloud;
	}

	bool getFrame(bool dump, bool publish, sensor_msgs::Image& color_image_msg, sensor_msgs::Image& depth_image_msg, sensor_msgs::PointCloud2& point_cloud_msg)
	{
		// get frame
		rs2::frame color_frame, depth_frame;
		getAlignedFrame(color_frame, depth_frame);

		// prepare message
		std_msgs::Header header = getHeader(); //res.point_cloud.header
		cv_bridge::CvImage cv_color(header, sensor_msgs::image_encodings::BGR8, cv::Mat());
		cv_bridge::CvImage cv_depth(header, sensor_msgs::image_encodings::MONO16, cv::Mat());

		// get cv image
		getCvColorImage(color_frame, cv_color.image);
		getCvDepthImage(depth_frame, cv_depth.image);
		//PointCloud::Ptr point_cloud = getPointCloud(depth_frame);
		PointCloud::Ptr point_cloud = getPointCloud2(depth_frame);
		
		// to ros message
		color_image_msg = *cv_color.toImageMsg();
		depth_image_msg = *cv_depth.toImageMsg();
		pcl::toROSMsg(*point_cloud, point_cloud_msg);

		// store image and point cloud
		if (dump) {
			dumpFrame(cv_color.image, cv_depth.image, point_cloud);
		}

		// publish point cloud if requested
		if (publish) {
			publishImage(color_image_msg, depth_image_msg, point_cloud_msg);
		}
		return true;
	}

	bool getFrameCallback(o2as_realsense_camera::GetFrame::Request & req, o2as_realsense_camera::GetFrame::Response & res) 
	{
		return getFrame(req.dump, req.publish, res.color_image, res.depth_image, res.point_cloud);
	}

	void dumpFrame(cv::Mat const & color_image, cv::Mat const & depth_image, PointCloud::Ptr point_cloud) {
		// create path if it does not exist
		boost::filesystem::path path(camera_data_path_);
		if (!boost::filesystem::is_directory(path)) {
			boost::filesystem::create_directory(camera_data_path_);
		}

		camera_data_path_;
		std::string color_image_filename = camera_data_path_ + "/" + color_image_filename_;
		std::string depth_image_filename = camera_data_path_ + "/" + depth_image_filename_;
		std::string point_cloud_filename = camera_data_path_ + "/" + point_cloud_filename_;
		cv::imwrite((char*)color_image_filename.c_str(), color_image);
		cv::imwrite((char*)depth_image_filename.c_str(), depth_image);
		pcl::io::savePCDFile((char*)point_cloud_filename.c_str(), *point_cloud);
	}

	bool dumpFrameCallback(o2as_realsense_camera::DumpFrame::Request & req, o2as_realsense_camera::DumpFrame::Response & res) 
	{
		camera_data_path_     = req.camera_data_path;
		color_image_filename_ = req.color_image_filename;
		depth_image_filename_ = req.depth_image_filename;
		point_cloud_filename_ = req.point_cloud_filename;

		sensor_msgs::Image color_image_msg;
		sensor_msgs::Image depth_image_msg; 
		sensor_msgs::PointCloud2 point_cloud_msg;
		getFrame(true, false, color_image_msg, depth_image_msg, point_cloud_msg);
		return true;
	}

    inline void publishImage(sensor_msgs::Image& color_image_msg, sensor_msgs::Image& depth_image_msg, sensor_msgs::PointCloud2& point_cloud_msg) 
	{
		publishers_.color_image.publish(color_image_msg);
		publishers_.depth_image.publish(depth_image_msg);
		// publishers_.point_cloud.publish(point_cloud_msg);
	}

    void publishImageCallback(ros::TimerEvent const &) 
	{
		sensor_msgs::Image color_image_msg;
		sensor_msgs::Image depth_image_msg; 
		sensor_msgs::PointCloud2 point_cloud_msg;
		getFrame(false, true, color_image_msg, depth_image_msg, point_cloud_msg);
	}

	//###################################################### 
	// Transform
	//###################################################### 

    struct float3
    {
        float x, y, z;
    };
    struct quaternion
    {
        double x, y, z, w;
    };

	void publish_static_tf(const ros::Time& t, const float3& trans, const quaternion& q, const std::string& from, const std::string& to)
	{
		geometry_msgs::TransformStamped msg;
		msg.header.stamp = t;
		msg.header.frame_id = from;
		msg.child_frame_id = to;
		msg.transform.translation.x = trans.z;
		msg.transform.translation.y = -trans.x;
		msg.transform.translation.z = -trans.y;
		msg.transform.rotation.x = q.x;
		msg.transform.rotation.y = q.y;
		msg.transform.rotation.z = q.z;
		msg.transform.rotation.w = q.w;
		static_tf_broadcaster_.sendTransform(msg);
	}

	void publishStaticTransforms()
	{
		ros::Time transform_ts = ros::Time::now();
		float3 zero_trans{0, 0, 0};
		publish_static_tf(transform_ts, zero_trans, quaternion{0, 0, 0, 1}, "world", "camera_link");
	}

private:
	/// The node handle
    ros::NodeHandle nh_;

	/// Serial id of the RealSense camera.
	std::string serial_number_;
	/// Size of images to be captured.
	int color_width_;
	int color_height_;
	int depth_width_;
	int depth_height_;
	/// Camera configuration
	rs2::config config_;
	/// Camera pipeline object
    rs2::pipeline pipe_;
	/// State flag of the node
	bool active_;
	/// Camera parameters
    double depth_scale_;
    double inv_param_[9];

	/// If true, publishes images with a frequency of 30Hz.
	bool publish_images_;
	/// Timer to trigger image publishing.
	ros::Timer publish_images_timer_;
	/// Object for handling transportation of images.
	image_transport::ImageTransport image_transport_;
	/// The frame in which the image and point clouds are send.
	std::string camera_frame_;
	/// Publish static transform
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
	/// Location where the images and point clouds are stored.
	std::string camera_data_path_;
	std::string color_image_filename_;
	std::string depth_image_filename_;
	std::string point_cloud_filename_;

	struct Publishers {
		/// Publisher for publishing color images.
		image_transport::Publisher color_image;
		/// Publisher for publishing depth images.
		image_transport::Publisher depth_image;
		/// Publisher for publishing raw point clouds.
		ros::Publisher point_cloud;
	} publishers_;

	struct Servers {
		/// Service server for supplying point clouds and images.
		ros::ServiceServer get_frame;
		/// Service server for dumping image and cloud to disk.
		ros::ServiceServer dump_frame;
	} servers_;
};

} // namespace o2as

int main(int argc, char ** argv) {
    ros::init(argc, argv, "realsense_camera");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
	o2as::RealSenseCameraNode node;
	ros::spin();
}
