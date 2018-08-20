#pragma once

#include <ros/ros.h>
#include <string>

#include <o2as_cad_matching_msgs/LoadModelData.h>
#include <o2as_cad_matching_msgs/Prepare.h>
#include <o2as_cad_matching_msgs/Search.h>
#include <o2as_cad_matching_msgs/SearchResult.h>

namespace o2as {

const std::string LOAD_MODEL_DATA_SERVICE = "load_model_data";
const std::string PREPARE_SERVICE = "prepare";
const std::string SEARCH_SERVICE = "search";

class CadMatchingInterface
{
public:
	CadMatchingInterface(std::string group_name);
	~CadMatchingInterface();

	bool LoadModelData(std::string model_filename = "");
	bool Prepare();
	bool Search(std::string pcloud_filename, std::string color_filename, 
		o2as_cad_matching_msgs::SearchResult& result);

protected:
    std::string group_name_;
    ros::NodeHandle nh_;
    ros::ServiceClient loadmodeldata_service_;
    ros::ServiceClient prepare_service_;
    ros::ServiceClient search_service_;
};

} // namespace o2as
