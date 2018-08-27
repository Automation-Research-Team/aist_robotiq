#include "o2as_realsense_camera/o2as_realsense_camera.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/Image.h"

#include "o2as_realsense_camera/util_cam.h"

namespace o2as {

RealSenseCamera::RealSenseCamera(std::string serial_number, int color_width, int color_height, int depth_width, int depth_height)
{
    this->display_ = false;
    this->publish_ = false;

	this->connected_ 	= false;
	this->depth_scale_ 	= 0.0;
	memset(this->inv_param_, 0, 9 * sizeof(double));

	// params
	this->serial_number_ = serial_number;
	this->color_width_   = color_width;
    this->color_height_  = color_height;
    this->depth_width_   = depth_width;
    this->depth_height_  = depth_height;

    // dynamic reconfigure
    dynamic_reconfigure::Server<o2as_realsense_camera::CameraSettingConfig>::CallbackType f;
    f = boost::bind(&RealSenseCamera::dynamic_reconfigure_callback, this, _1, _2);
    this->dynamic_reconfigure_server.setCallback(f);

	// services
	ROS_INFO("advertise connect service");
	this->connect_service_ = this->nh_.advertiseService(CONNECT_SERVICE, &RealSenseCamera::executeConnect, this);
	ROS_INFO("advertise save_frame_for_cad_matching service");
	this->save_frame_for_cad_matching_service_ = this->nh_.advertiseService(SAVE_FRAME_FOR_CAD_MATCHING_SERVICE, &RealSenseCamera::executeSaveFrameForCadMatching, this);
	ROS_INFO("advertise save_camera_frame service");
	this->save_camera_info_service_ = this->nh_.advertiseService(SAVE_CAMERA_INFO_SERVICE, &RealSenseCamera::executeSaveCameraInfo, this);

	// publisher
	this->color_image_pub_ = this->nh_.advertise<sensor_msgs::Image>(COLOR_IMAGE_TOPIC, 1);
	this->depth_image_pub_ = this->nh_.advertise<sensor_msgs::Image>(DEPTH_IMAGE_TOPIC, 1);

	ROS_INFO("realsense camera node ready");
}

RealSenseCamera::~RealSenseCamera()
{
	disconnect();
	disableDisplay();
}

//###################################################### 
// dynamic reconfigure
//###################################################### 

void RealSenseCamera::dynamic_reconfigure_callback(o2as_realsense_camera::CameraSettingConfig &config, uint32_t level)
{
    ROS_INFO("reconfigure = %d, %d", config.display, config.publish);
    if (this->display_ != config.display) {
		this->display_ = config.display;
		if (this->display_ == true) {
			this->enableDisplay();
		} else {
			this->disableDisplay();
		}
	}
    if (this->publish_ != config.publish) {
		this->publish_ = config.publish;
	}
}

void RealSenseCamera::enableDisplay()
{
	cv::namedWindow("color");
	cv::namedWindow("depth");
	cv::startWindowThread();
}

void RealSenseCamera::disableDisplay()
{
	cv::destroyWindow("color");
	cv::destroyWindow("depth");
}

//###################################################### 
// connect / disconnect
//###################################################### 

bool RealSenseCamera::disconnect()
{
	if (this->connected_) {
		this->pipe_.stop();
		this->connected_ = false;
	}
}

bool RealSenseCamera::connect()
{
	if (this->connected_) {
		disconnect();
	}

	// start pipeline
	rs2::config config;
	config.enable_device(this->serial_number_);
	config.enable_stream(rs2_stream::RS2_STREAM_COLOR, this->color_width_, this->color_height_, rs2_format::RS2_FORMAT_BGR8);
	config.enable_stream(rs2_stream::RS2_STREAM_DEPTH, this->depth_width_, this->depth_height_, rs2_format::RS2_FORMAT_Z16 );
	rs2::pipeline_profile pipeline_profile = this->pipe_.start(config);
	this->connected_ = true;

	prepareDataConversion();
	return true;
}

bool RealSenseCamera::executeConnect(
	o2as_realsense_camera::Connect::Request &req,
	o2as_realsense_camera::Connect::Response &res) 
{
	return connect();
}

//###################################################### 
// camera info
//###################################################### 

rs2_intrinsics RealSenseCamera::getColorCameraIntrinsics()
{
	rs2::pipeline_profile pipeline_profile = this->pipe_.get_active_profile();
	rs2::stream_profile color_profile = pipeline_profile.get_stream(rs2_stream::RS2_STREAM_COLOR);
	rs2::video_stream_profile video_stream_profile_color = color_profile.as<rs2::video_stream_profile>();
	rs2_intrinsics intrinsics_color = video_stream_profile_color.get_intrinsics();
	return intrinsics_color;
}

bool RealSenseCamera::prepareDataConversion()
{
	rs2::pipeline_profile pipeline_profile = this->pipe_.get_active_profile();

	// depth_scale
	rs2::depth_sensor depth_sensor = pipeline_profile.get_device().first<rs2::depth_sensor>();
	this->depth_scale_ = (double)depth_sensor.get_depth_scale();

	// inv_matrix
	rs2_intrinsics intrinsics_color = getColorCameraIntrinsics();
	float cam_param[9] = {intrinsics_color.fx, 0.0, intrinsics_color.ppx, 0.0, intrinsics_color.fy, intrinsics_color.ppy, 0.0, 0.0, 1.0};
	InvMatrix3x3(cam_param, this->inv_param_);
}

bool RealSenseCamera::saveCameraInfo(std::string setting_filename)
{
	rs2_intrinsics intrinsics = getColorCameraIntrinsics();
	ROS_INFO("focal_length_x = %f",    intrinsics.fx);
	ROS_INFO("focal_length_y = %f",    intrinsics.fy);
	ROS_INFO("principal_point_x = %f", intrinsics.ppx);
	ROS_INFO("principal_point_y = %f", intrinsics.ppy);

	CameraSetting setting;
	setting.width             = intrinsics.width;
	setting.height            = intrinsics.height;
	setting.focal_length_x    = intrinsics.fx;
	setting.focal_length_y    = intrinsics.fy;
	setting.principal_point_x = intrinsics.ppx;
	setting.principal_point_y = intrinsics.ppy;
	setting.dist_param_k1     = intrinsics.coeffs[0];
	setting.dist_param_k2     = intrinsics.coeffs[1];
	setting.dist_param_p1     = intrinsics.coeffs[2];
	setting.dist_param_p2     = intrinsics.coeffs[3];
	setting.dist_param_k3     = intrinsics.coeffs[4];
	return SaveCameraSetting(setting_filename, setting);
}

bool RealSenseCamera::executeSaveCameraInfo(
	o2as_realsense_camera::SaveCameraInfo::Request &req,
	o2as_realsense_camera::SaveCameraInfo::Response &res) 
{
	res.success = saveCameraInfo(req.filename);
	return true;
}

//###################################################### 
// capture
//###################################################### 

bool RealSenseCamera::getAlignedFrameSet(rs2::frame& depth_frame, rs2::frame& color_frame)
{
	try 
	{
		// get frame
		rs2::frameset   frameset = this->pipe_.wait_for_frames();
		rs2::align      align(rs2_stream::RS2_STREAM_COLOR);
		rs2::frameset   aligned_frameset = align.process(frameset);
		if(!aligned_frameset.size()) {
			return false;
		}
		depth_frame = aligned_frameset.get_depth_frame();
		color_frame = aligned_frameset.get_color_frame();
	}
	catch(const char *error_message) {
		ROS_ERROR("%s", error_message);
		return false;
	}
	return true;
}

bool RealSenseCamera::saveDepthFrameInCadFormat(std::string pcloud_filename, rs2::frame& depth_frame)
{
	float *point_cloud = NULL;
	try 
	{
		ROS_DEBUG("save depth image. filename = %s", pcloud_filename.c_str());
		int width  = depth_frame.as<rs2::video_frame>().get_width();
		int height = depth_frame.as<rs2::video_frame>().get_height();
		point_cloud = (float*)malloc(width * height * 3 * sizeof(float));
		GetPointCloud2(depth_frame, this->depth_scale_, this->inv_param_, point_cloud);
		SavePointCloudToBinary(point_cloud, width, height, (char*)pcloud_filename.c_str());
	}
	catch(const char *error_message) {
		ROS_ERROR("%s", error_message);
	}
	if (point_cloud != NULL) free(point_cloud);
	return true;
}

bool RealSenseCamera::saveColorFrameInCadFormat(std::string image_filename, const rs2::frame& color_frame)
{
	try 
	{
		ROS_DEBUG("save color image. filename = %s", image_filename.c_str());
		int width  = color_frame.as<rs2::video_frame>().get_width();
		int height = color_frame.as<rs2::video_frame>().get_height();
		cv::Mat cv_color_image = cv::Mat(height, width, CV_8UC3, const_cast<void*>(color_frame.get_data()));
		imwrite((char*)image_filename.c_str(), cv_color_image);
	}
	catch(const char *error_message) {
		ROS_ERROR("%s", error_message);
		return false;
	}
	return true;
}

bool RealSenseCamera::saveFrameForCadMatching(std::string pcloud_filename, std::string image_filename)
{
	try 
	{
		rs2::frame depth_frame, color_frame;
		getAlignedFrameSet(depth_frame, color_frame);

		saveDepthFrameInCadFormat(pcloud_filename, depth_frame);
		saveColorFrameInCadFormat(image_filename, color_frame);

		if (this->publish_ || this->display_) {
			// to cv image
			int width  = color_frame.as<rs2::video_frame>().get_width();
			int height = color_frame.as<rs2::video_frame>().get_height();
			cv::Mat cv_depth_image = cv::Mat(height, width, CV_16UC1, const_cast<void*>(depth_frame.get_data()));
			cv::Mat cv_color_image = cv::Mat(height, width, CV_8UC3, const_cast<void*>(color_frame.get_data()));

			if (this->publish_) {
				// publish captured image
				sensor_msgs::ImagePtr depth_image_msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", cv_depth_image).toImageMsg();
				sensor_msgs::ImagePtr color_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_color_image).toImageMsg();
				this->depth_image_pub_.publish(depth_image_msg);
				this->color_image_pub_.publish(color_image_msg);
			}
			if (this->display_) {
				// display captured image
				cv::imshow("depth", cv_depth_image);
				cv::imshow("color", cv_color_image);
			}
		}
	}
	catch(const char *error_message) {
		ROS_ERROR("%s", error_message);
	}
}

bool RealSenseCamera::executeSaveFrameForCadMatching(
	o2as_realsense_camera::SaveFrameForCadMatching::Request &req,
	o2as_realsense_camera::SaveFrameForCadMatching::Response &res) 
{
	saveFrameForCadMatching(req.pcloud_filename, req.image_filename);
	return true;
}

bool RealSenseCamera::polling()
{
	// capture depth and color image
	rs2::frameset frames;
	if (this->pipe_.poll_for_frames(&frames))
	{
		rs2::frame depth_frame = frames.first(RS2_STREAM_DEPTH);
		depth_frame.get_data();
	}
	return true;
}

} // namespace o2as
