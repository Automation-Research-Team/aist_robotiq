#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/PointCloud2.h>
#include <o2as_pc2depth/pc2depth.h>


bool callback(o2as_pc2depth::pc2depth::Request& req,
    o2as_pc2depth::pc2depth::Response& res) {

    // Receive topic coming from depth sensor
    sensor_msgs::PointCloud2 msg =
        *ros::topic::waitForMessage<sensor_msgs::PointCloud2>(req.topic_name);

    // Create response (sensor_msgs::Image)
    ros::Time timeNow = ros::Time::now();
    std::string frame = "camera";

    sensor_msgs::Image depth_map;
    depth_map.header.stamp = timeNow;
    depth_map.header.frame_id = frame;
    depth_map.encoding = "32FC1";

    int offset_z = msg.fields[2].offset;
    sensor_msgs::fillImage(
        depth_map, sensor_msgs::image_encodings::TYPE_32FC1,
        msg.height, msg.width, msg.width * sizeof(float),
        &msg.data[offset_z]
    );
    res.image = depth_map;

    return true;
}


int main(int argc, char **argv) {
    std::cout << "Starting pc2depth ROS service server ...";
    ros::init(argc, argv, "pc2depth_server");
    ros::NodeHandle nh("~");

    nh.advertiseService("pc2depth", callback);

    std::cout << "Ready!" << std::endl;
    ros::spin();
    return 0;
}
