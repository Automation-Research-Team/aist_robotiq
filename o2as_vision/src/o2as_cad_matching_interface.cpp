
#include "o2as_cad_matching_interface.h"
//#include "o2as_cad_matching/o2as_cad_matching.h"

namespace o2as {

CadMatchingInterface::CadMatchingInterface(std::string group_name)
{
	ROS_DEBUG("CadMatchingInterface::CadMatchingInterface() begin");

    this->group_name_ = group_name;
	ros::service::waitForService(group_name+"/"+LOAD_MODEL_DATA_SERVICE, ros::Duration(-1));
   	this->loadmodeldata_service_ = this->nh_.serviceClient<o2as_cad_matching_msgs::LoadModelData>(group_name+"/"+LOAD_MODEL_DATA_SERVICE);
	ros::service::waitForService(group_name+"/"+PREPARE_SERVICE, ros::Duration(-1));
   	this->prepare_service_ = this->nh_.serviceClient<o2as_cad_matching_msgs::Prepare>(group_name+"/"+PREPARE_SERVICE);
	ros::service::waitForService(group_name+"/"+SEARCH_SERVICE, ros::Duration(-1));
   	this->search_service_ = this->nh_.serviceClient<o2as_cad_matching_msgs::Search>(group_name+"/"+SEARCH_SERVICE);

	ROS_DEBUG("CadMatchingInterface::CadMatchingInterface() end");
}

CadMatchingInterface::~CadMatchingInterface()
{
	ROS_INFO("CadMatchingInterface::~CadMatchingInterface()");
}

bool CadMatchingInterface::LoadModelData(std::string model_filename)
{
	ROS_DEBUG("CadMatchingInterface::LoadModelData() begin");

	o2as_cad_matching_msgs::LoadModelData srv;
	if (!this->loadmodeldata_service_.call(srv)) {
		ROS_ERROR("CadMatchingInterface::LoadModelData() failed");
		return false;
	}

	ROS_DEBUG("CadMatchingInterface::LoadModelData() success");
	return true;
}

bool CadMatchingInterface::Prepare()
{
	ROS_DEBUG("CadMatchingInterface::Prepare() begin");

	o2as_cad_matching_msgs::Prepare srv;
	if (this->prepare_service_.call(srv)) {
		ROS_INFO("Prepare success");
	} else {
		ROS_ERROR("Prepare fail");
		return false;
	}

	ROS_DEBUG("CadMatchingInterface::Prepare() end");
	return true;
}

bool CadMatchingInterface::Search(std::string pcloud_filename, std::string image_filename, 
	o2as_cad_matching_msgs::SearchResult& result)
{
	ROS_DEBUG("CadMatchingInterface::Search() begin");

	o2as_cad_matching_msgs::Search srv;
	srv.request.pcloud_filename = pcloud_filename;
	srv.request.image_filename = image_filename;
	if (this->search_service_.call(srv)) {
		ROS_INFO("Search success");
		result = srv.response.search_result;
	} else {
		ROS_ERROR("Search fail");
		return false;
	}

	ROS_DEBUG("CadMatchingInterface::Search() end");
	return true;
}

} // namespace o2as
