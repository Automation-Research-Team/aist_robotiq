#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <o2as_realsense_camera/RealSenseCameraConfig.h>

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
        // Setup dynamic reconfigure
        dynamic_reconfigure::Server<o2as_realsense_camera::RealSenseCameraConfig>::CallbackType f;
        f = boost::bind(&RealSenseCameraNode::dynamicReconfigureCallback, this, _1, _2);
        dynamic_reconfigure_server_.setCallback(f);
        
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

    enum config_entry {
        cfg_send_color = 0,
        cfg_send_depth,
        cfg_send_cloud,
        cfg_count
    };

    void setParam(o2as_realsense_camera::RealSenseCameraConfig &config, config_entry param)
    {
        switch (param) {
        case cfg_send_color:
            ROS_DEBUG_STREAM("send_color: " << config.send_color);
            send_color_ = config.send_color;
            break;
        case cfg_send_depth:
            ROS_DEBUG_STREAM("send_depth: " << config.send_depth);
            send_depth_ = config.send_depth;
            break;
        case cfg_send_cloud:
            ROS_DEBUG_STREAM("send_cloud: " << config.send_cloud);
            send_cloud_ = config.send_cloud;
            break;
        }
    }

    void dynamicReconfigureCallback(o2as_realsense_camera::RealSenseCameraConfig &config, uint32_t level)
    {
        if (set_default_dynamic_reconfig_values == level)
        {
            for (int i = 1; i < cfg_count; ++i)
            {
                ROS_DEBUG_STREAM("config = " << i);
                setParam(config ,(config_entry)i);
            }
        }
        else
        {
            setParam(config, (config_entry)level);
        }
    }

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
        ros::param::get("~send_color", send_color_);
        ros::param::get("~send_depth", send_depth_);
        ros::param::get("~send_cloud", send_cloud_);
        ros::param::get("~trigger_mode", trigger_mode_);

        // Enable camera
        if (!deviceExists(serial_number_)) {
            ROS_ERROR("realsense device with serial number %s not exists", serial_number_.c_str());
        }

        rs_config_.enable_device(serial_number_);
        rs_config_.enable_stream(rs2_stream::RS2_STREAM_COLOR, color_width_, color_height_, rs2_format::RS2_FORMAT_BGR8);
        rs_config_.enable_stream(rs2_stream::RS2_STREAM_DEPTH, depth_width_, depth_height_, rs2_format::RS2_FORMAT_Z16 );
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
        rs2::pipeline_profile pipeline_profile = rs_pipe_.get_active_profile();
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
        rs2::pipeline_profile pipeline_profile = rs_pipe_.get_active_profile();

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
        rs2::pipeline_profile pipeline_profile = rs_pipe_.start(rs_config_);
        prepareConversion();

        // activate service servers
        servers_.get_frame = nh_.advertiseService("get_frame", &RealSenseCameraNode::getFrameCallback, this);
        servers_.dump_frame = nh_.advertiseService("dump_frame", &RealSenseCameraNode::dumpFrameCallback, this);

        // activate publishers
        publishers_.color_image = image_transport_.advertise("color", 1, true);
        publishers_.depth_image = image_transport_.advertise("depth", 1, true);
        publishers_.point_cloud = nh_.advertise<PointCloud>("cloud", 1, true);

        // start image publishing timer
        publish_frame_timer_ = nh_.createTimer(ros::Rate(30), &RealSenseCameraNode::publishFrameCallback, this);
        set_trigger_mode(trigger_mode_);

        active_ = true;
        return 0;
    }

    void set_trigger_mode(bool mode)
    {
        if (trigger_mode_ == false) {
            publish_frame_timer_.start();
        } else {
            publish_frame_timer_.stop();
        }
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
        publish_frame_timer_.stop();
        // Release the camera
        rs_pipe_.stop();
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
            rs2::frameset frameset = rs_pipe_.wait_for_frames();
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

    bool getFrame(bool publish = false, 
        sensor_msgs::Image* color_image_msg = NULL, 
        sensor_msgs::Image* depth_image_msg = NULL, 
        sensor_msgs::PointCloud2* point_cloud_msg = NULL)
    {
        ROS_DEBUG_STREAM("getFrame"
            << "publish: "      << publish       << ", "
            << "trigger_mode: " << trigger_mode_ << ", "
            << "send_color: "   << send_color_   << ", "
            << "send_depth: "   << send_depth_   << ", "
            << "send_cloud: "   << send_cloud_
            );

        // get frame
        rs2::frame color_frame, depth_frame;
        getAlignedFrame(color_frame, depth_frame);

        // prepare message
        std_msgs::Header header = getHeader();
        cv_bridge::CvImage cv_color(header, sensor_msgs::image_encodings::BGR8, cv::Mat());
        cv_bridge::CvImage cv_depth(header, sensor_msgs::image_encodings::MONO16, cv::Mat());

        // get cv image
        getCvColorImage(color_frame, cv_color.image);
        getCvDepthImage(depth_frame, cv_depth.image);
        PointCloud::Ptr point_cloud = getPointCloud(depth_frame);
        
        // to ros message
        if (color_image_msg!=NULL && send_color_==true) {
            *color_image_msg = *cv_color.toImageMsg();
        }
        if (color_image_msg!=NULL && send_depth_==true) {
            *depth_image_msg = *cv_depth.toImageMsg();
        }
        if (color_image_msg!=NULL && send_cloud_==true) {
            pcl::toROSMsg(*point_cloud, *point_cloud_msg);
        }

        // publish point cloud if requested
        if (publish) {
            publishFrame(color_image_msg, depth_image_msg, point_cloud_msg);
        }
        return true;
    }

    bool getFrameCallback(
        o2as_realsense_camera::GetFrame::Request & req, 
        o2as_realsense_camera::GetFrame::Response & res) 
    {
        return getFrame(req.publish, &res.color_image, &res.depth_image, &res.point_cloud);
    }

    inline void publishFrame(
        sensor_msgs::Image* color_image_msg = NULL, 
        sensor_msgs::Image* depth_image_msg = NULL, 
        sensor_msgs::PointCloud2* point_cloud_msg = NULL) 
    {
        if (color_image_msg) {
            publishers_.color_image.publish(*color_image_msg);
        }
        if (depth_image_msg) {
            publishers_.depth_image.publish(*depth_image_msg);
        }
        if (point_cloud_msg) {
            publishers_.point_cloud.publish(*point_cloud_msg);
        }
    }

    void publishFrameCallback(ros::TimerEvent const &) 
    {
        sensor_msgs::Image color_image_msg;
        sensor_msgs::Image depth_image_msg; 
        sensor_msgs::PointCloud2 point_cloud_msg;
        getFrame(true, &color_image_msg, &depth_image_msg, &point_cloud_msg);
    }

    /// Conversion from depth image to point cloud using camera parameter
    float* getPointCloud2(cv::Mat& cv_depth_image)
    {
        int width  = cv_depth_image.cols;
        int height = cv_depth_image.rows;
        float *cloud = (float*)malloc(width * height * 3 * sizeof(float));
        if (cloud == NULL) {
            ROS_ERROR_STREAM("get point cloud : memory allocation error");
            return NULL;
        }

        int pos = 0;
        double proj_x, proj_y, depth;
        const float max_depth = 1e+6f + 1.0f;
        unsigned short *ptr_depth;
        double scale = depth_scale_ * 1000; // in mm unit
        for (int y = 0; y < height; y++) {
            ptr_depth = cv_depth_image.ptr<unsigned short>(y);
            for (int x = 0; x < width; x++) {
                proj_x = inv_param_[0] * (double)x + inv_param_[1] * (double)y + inv_param_[2];
                proj_y = inv_param_[3] * (double)x + inv_param_[4] * (double)y + inv_param_[5];
                depth  = scale  * (double)ptr_depth[x];
                cloud[pos  ] = (float)(proj_x * depth); 
                cloud[pos+1] = (float)(proj_y * depth);
                cloud[pos+2] = (float)depth;
                pos += 3;
            }
        }
        return cloud;
    }

    bool save_cloud_binary(const char* filename, int width, int height, float *cloud)
    {
        FILE *fout = fopen(filename, "wb");
        if (fout == NULL) {
            ROS_ERROR_STREAM("cannot open point cloud file - " << filename);
            return false;
        }
        fwrite(cloud, sizeof(float), width * height * 3, fout);
        fclose(fout);
        return true;
    }

    bool dumpFrameCallback(
        o2as_realsense_camera::DumpFrame::Request & req, 
        o2as_realsense_camera::DumpFrame::Response & res) 
    {
        // get frame
        rs2::frame color_frame, depth_frame;
        getAlignedFrame(color_frame, depth_frame);

        // save color image
        if (!req.color_image_filename.empty()) {
            cv::Mat cv_color_image;
            getCvColorImage(color_frame, cv_color_image);
            cv::imwrite((char*)req.color_image_filename.c_str(), cv_color_image);
        }

        // save depth image and point cloud
        if (!req.depth_image_filename.empty() || !req.point_cloud_filename.empty()) {
            cv::Mat cv_depth_image;
            getCvDepthImage(depth_frame, cv_depth_image);
            if (!req.depth_image_filename.empty()) {
                cv::imwrite((char*)req.depth_image_filename.c_str(), cv_depth_image);
            }
            if (!req.point_cloud_filename.empty()) {
                float* cloud = getPointCloud2(cv_depth_image);
                save_cloud_binary((char*)req.point_cloud_filename.c_str(), cv_depth_image.cols, cv_depth_image.rows, cloud);
                free(cloud);
            }
        }

        return true;
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

    void publish_static_tf(const ros::Time& t, 
        const float3& trans, const quaternion& q, 
        const std::string& from, const std::string& to)
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
    /// Dynamic reconfigure
    dynamic_reconfigure::Server<o2as_realsense_camera::RealSenseCameraConfig> dynamic_reconfigure_server_;
    const uint32_t set_default_dynamic_reconfig_values = 0xffffffff;

    /// Serial id of the RealSense camera.
    std::string serial_number_;
    /// Size of images to be captured.
    int color_width_;
    int color_height_;
    int depth_width_;
    int depth_height_;
    /// Camera configuration
    rs2::config rs_config_;
    /// Camera pipeline object
    rs2::pipeline rs_pipe_;
    /// State flag of the node
    bool active_;
    /// Camera parameters
    double depth_scale_;
    double inv_param_[9];
    /// If true, publishes images with a frequency of 30Hz.
    bool trigger_mode_;
    bool send_color_;
    bool send_depth_;
    bool send_cloud_;
    /// Timer to trigger frame publishing.
    ros::Timer publish_frame_timer_;
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

    // struct Config {
    // } config_;

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
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    o2as::RealSenseCameraNode node;
    ros::spin();
}
