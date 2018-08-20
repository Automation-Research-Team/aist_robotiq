#include "o2as_realsense_camera/o2as_realsense_camera.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "o2as_realsense_camera/util_cam.h"

namespace o2as {

RealSenseCamera::RealSenseCamera(std::string serial_number)
{
	this->serial_number_ = serial_number;
	
	cv::namedWindow("color");
	cv::namedWindow("depth");
	cv::startWindowThread();

	// service
	ROS_INFO("advertise connect service");
	this->service_connect_ = this->nh_.advertiseService(CONNECT_SERVICE, &RealSenseCamera::Connect, this);
	ROS_INFO("advertise save_frame_for_cad_matching service");
	this->service_save_frame_for_cad_matching_ = this->nh_.advertiseService(SAVE_FRAME_FOR_CAD_MATCHING_SERVICE, &RealSenseCamera::SaveFrameForCadMatching, this);
	ROS_INFO("realsense camera node ready");
}

RealSenseCamera::~RealSenseCamera()
{
	this->pipe_.stop();
	cv::destroyWindow("color");
	cv::destroyWindow("depth");
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
bool RealSenseCamera::Connect(
	o2as_realsense_camera::Connect::Request &req,
	o2as_realsense_camera::Connect::Response &res) 
{
	// enable device
	rs2::config config;
	config.enable_device(this->serial_number_);
	config.enable_stream(rs2_stream::RS2_STREAM_COLOR, this->color_width_, this->color_height_, rs2_format::RS2_FORMAT_BGR8);
	config.enable_stream(rs2_stream::RS2_STREAM_DEPTH, this->depth_width_, this->depth_height_, rs2_format::RS2_FORMAT_Z16 );
	rs2::pipeline_profile pipeline_profile = this->pipe_.start(config);
	// get depth scale
	rs2::depth_sensor depth_sensor = pipeline_profile.get_device().first<rs2::depth_sensor>();
	this->depth_scale_ = (double)depth_sensor.get_depth_scale();
	// get intrinsics
	rs2::stream_profile color_profile = pipeline_profile.get_stream(rs2_stream::RS2_STREAM_COLOR);
	rs2::stream_profile depth_profile = pipeline_profile.get_stream(rs2_stream::RS2_STREAM_DEPTH);
	rs2::video_stream_profile video_stream_profile_color = color_profile.as<rs2::video_stream_profile>();
	rs2::video_stream_profile video_stream_profile_depth = depth_profile.as<rs2::video_stream_profile>();
	rs2_intrinsics intrinsics_color = video_stream_profile_color.get_intrinsics();
	rs2_intrinsics intrinsics_depth = video_stream_profile_depth.get_intrinsics();
	ROS_INFO("focal_length_x = %f",    intrinsics_color.fx);
	ROS_INFO("focal_length_y = %f",    intrinsics_color.fy);
	ROS_INFO("principal_point_x = %f", intrinsics_color.ppx);
	ROS_INFO("principal_point_y = %f", intrinsics_color.ppy);
	float cam_param[9] = {intrinsics_color.fx, 0.0, intrinsics_color.ppx, 0.0, intrinsics_color.fy, intrinsics_color.ppy, 0.0, 0.0, 1.0};
	InvMatrix3x3(cam_param, this->inv_param_);
	return true;
}

bool RealSenseCamera::SaveFrameForCadMatching(
	o2as_realsense_camera::SaveFrameForCadMatching::Request &req,
	o2as_realsense_camera::SaveFrameForCadMatching::Response &res) 
{
	float *point_cloud = NULL;
	try 
	{
		// get frame
		rs2::frameset   frameset = this->pipe_.wait_for_frames();
		rs2::align      align(rs2_stream::RS2_STREAM_COLOR);
		rs2::frameset   aligned_frameset = align.process(frameset);
		if(!aligned_frameset.size()) {
			return false;
		}
		rs2::frame depth_frame = aligned_frameset.get_depth_frame();
		rs2::frame color_frame = aligned_frameset.get_color_frame();
		// to cv image
		int width  = color_frame.as<rs2::video_frame>().get_width();
		int height = color_frame.as<rs2::video_frame>().get_height();
		cv::Mat cv_depth_image = cv::Mat(height, width, CV_16SC1, const_cast<void*>(depth_frame.get_data()));
		cv::Mat cv_color_image = cv::Mat(height, width, CV_8UC3, const_cast<void*>(color_frame.get_data()));
		// convert image format
		ROS_DEBUG("get point cloud");
		point_cloud = (float*)malloc(width * height * 3 * sizeof(float));
		GetPointCloud2(depth_frame, this->depth_scale_, this->inv_param_, point_cloud);
		// save image data to files
		ROS_DEBUG("save color image. filename = %s", req.image_filename.c_str());
		imwrite((char*)req.image_filename.c_str(), cv_color_image);
		ROS_DEBUG("save depth image. filename = %s", req.pcloud_filename.c_str());
		SavePointCloudToBinary(point_cloud, width, height, (char*)req.pcloud_filename.c_str());
		cv::imshow("depth", cv_depth_image);
		cv::imshow("color", cv_color_image);
	}
	catch(const char *error_message) {
		ROS_ERROR("%s", error_message);
	}
	if (point_cloud != NULL) free(point_cloud);
	return true;
}

} // namespace o2as
