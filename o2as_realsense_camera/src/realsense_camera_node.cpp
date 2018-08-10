#include <string>
#include <librealsense2/rs.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>

#include "o2as_realsense_camera/Connect.h"
#include "o2as_realsense_camera/GetFrame.h"
#include "o2as_realsense_camera/SaveFrameForCadMatching.h"
#include "util_cam.h"

class RealsenseCamera
{
public:
    RealsenseCamera(std::string serial_number)
    {
        _serial_number = serial_number;

        cv::namedWindow("color");
        cv::namedWindow("depth");

        // service
        ROS_INFO("advertise connect service");
        this->_service_connect = nh.advertiseService("connect", &RealsenseCamera::Connect, this);
        ROS_INFO("advertise get_frame service");
        this->_service_get_frame = nh.advertiseService("get_frame", &RealsenseCamera::GetFrame, this);
        ROS_INFO("advertise save_frame_for_cad_matching service");
        this->_service_save_frame_for_cad_matching = nh.advertiseService("save_frame_for_cad_matching", &RealsenseCamera::SaveFrameForCadMatching, this);

        ROS_INFO("realsense camera node ready");
    }

    ~RealsenseCamera()
    {
        _pipe.stop();
        cv::destroyWindow("color");
        cv::destroyWindow("depth");
    }

    bool polling()
    {
		// capture depth and color image
        rs2::frameset frames;
        if (_pipe.poll_for_frames(&frames))
        {
            rs2::frame depth_frame = frames.first(RS2_STREAM_DEPTH);
            depth_frame.get_data();
        }
        return true;
    }

    bool Connect(
        o2as_realsense_camera::Connect::Request &req,
        o2as_realsense_camera::Connect::Response &res) 
    {
        // enable device
        rs2::config config;
        config.enable_device(_serial_number);
        config.enable_stream(rs2_stream::RS2_STREAM_COLOR, color_width, color_height, rs2_format::RS2_FORMAT_BGR8);
        config.enable_stream(rs2_stream::RS2_STREAM_DEPTH, depth_width, depth_height, rs2_format::RS2_FORMAT_Z16 );
        rs2::pipeline_profile pipeline_profile = _pipe.start(config);

        // get depth scale
        rs2::depth_sensor depth_sensor = pipeline_profile.get_device().first<rs2::depth_sensor>();
		_depth_scale = (double)depth_sensor.get_depth_scale();

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
    	//float _cam_param[9] = {465.5449829, 0.0, 315.8484497, 0.0, 465.5450439, 184.2312622, 0.0, 0.0, 1.0};
    	//float _cam_param[9] = {461.605774, 0.0, 318.471497, 0.0, 461.605804, 180.336258, 0.0, 0.0, 1.0}; //616205004841
        //float _cam_param[9] = {intrinsics_depth.fx, 0.0, intrinsics_depth.ppx, 0.0, intrinsics_depth.fy, intrinsics_depth.ppy, 0.0, 0.0, 1.0};
        float _cam_param[9] = {intrinsics_color.fx, 0.0, intrinsics_color.ppx, 0.0, intrinsics_color.fy, intrinsics_color.ppy, 0.0, 0.0, 1.0};
        InvMatrix3x3(_cam_param, _inv_param);

        return true;
    }
    
    bool GetFrame(
        o2as_realsense_camera::GetFrame::Request &req,
        o2as_realsense_camera::GetFrame::Response &res) 
    {
        // get frame
		rs2::frameset   frameset = _pipe.wait_for_frames();
		rs2::align      align(rs2_stream::RS2_STREAM_COLOR);
		rs2::frameset   aligned_frameset = align.process(frameset);
		if(!aligned_frameset.size()) {
			return false;
		}

        // depth image
		rs2::frame depth_frame = aligned_frameset.get_depth_frame();
   	    int depth_width  = depth_frame.as<rs2::video_frame>().get_width();
        int depth_height = depth_frame.as<rs2::video_frame>().get_height();
       	cv::Mat cv_depth_image = cv::Mat(depth_height, depth_width, CV_16SC1, const_cast<void*>(depth_frame.get_data()));

        // color image
		rs2::frame color_frame = aligned_frameset.get_color_frame();
   	    int color_width  = color_frame.as<rs2::video_frame>().get_width();
        int color_height = color_frame.as<rs2::video_frame>().get_height();
        cv::Mat cv_color_image = cv::Mat(color_height, color_width, CV_8UC3, const_cast<void*>(color_frame.get_data()));
        //cv::cvtColor(cv_color_image, cv_tmp_image, cv::COLOR_BGR2RGB);/

        cv::imshow("depth", cv_depth_image);
        cv::imshow("color", cv_color_image);
        return true;
    }

    bool SaveFrameForCadMatching(
        o2as_realsense_camera::SaveFrameForCadMatching::Request &req,
        o2as_realsense_camera::SaveFrameForCadMatching::Response &res) 
    {
        float *point_cloud = NULL;

        try 
        {
            // get frame
            rs2::frameset   frameset = _pipe.wait_for_frames();
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
            ROS_INFO("get point cloud");
            point_cloud = (float*)malloc(width * height * 3 * sizeof(float));
            GetPointCloud2(depth_frame, _depth_scale, _inv_param, point_cloud);

            // save image data to files
            ROS_INFO("save color image. filename = %s", req.image_filename.c_str());
            imwrite((char*)req.image_filename.c_str(), cv_color_image);
            ROS_INFO("save depth image. filename = %s", req.pcloud_filename.c_str());
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

private:
    std::string _serial_number;
    rs2::pipeline _pipe;
    int color_width = 640;
    int color_height = 360;
    int depth_width = 640;
    int depth_height = 480;
    double _depth_scale = 1.0;
    double _inv_param[9];

    ros::NodeHandle nh;
    ros::ServiceServer _service_connect;
    ros::ServiceServer _service_get_frame;
    ros::ServiceServer _service_save_frame_for_cad_matching;
};


int main(int argc, char **argv) 
{
    ros::init(argc, argv, "realsense_camera_node");
    ros::NodeHandle nh;

    std::string serial_number;
    ros::param::get("~serial_number", serial_number);
    RealsenseCamera device(serial_number);

    // cv::startWindowThread();
    try {
        int cnt = 0;
		while (ros::ok()) {
            //server.polling();
            cv::waitKey(1);
            ros::spinOnce();
        }
    }
	catch(const char *error_message) {
		ROS_ERROR("%s", error_message);
	}
}
