#include "o2as_vision.h"

using namespace o2as;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "monitor");
	ros::NodeHandle nh;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
    ros::console::notifyLoggerLevelsChanged();

	std::string image_dir;
	ros::param::get("~image_dir", image_dir);
	std::vector<std::string> camera_list;
	ros::param::get("~camera_list", camera_list);

	Vision vision;
    for (auto&& camera : camera_list)
    {
		ROS_INFO("register camera %s", camera.c_str());
		vision.RegisterGroup(camera);
		vision.GetGroup(camera).SetImageDir(image_dir);
	}
	vision.PrepareAll();

	double t = 1.0;
	ros::param::get("~sleep_time", t);
	ros::Duration sleep_time(t);
	while (ros::ok())
	{
		vision.UpdateScene();
		sleep_time.sleep();
	}
	return 0;
}
