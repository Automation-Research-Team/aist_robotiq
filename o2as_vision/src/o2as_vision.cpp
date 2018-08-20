#include "o2as_vision.h"
#include <ros/ros.h>

namespace o2as {

Vision::Vision()
{
	ROS_INFO("Vision::Vision()");
}

Vision::~Vision()
{
	ROS_INFO("Vision::~Vision()");
    for (auto&& group : this->groups_)
    {
		delete group.second;
	}
	this->groups_.clear();
}

bool Vision::RegisterGroup(std::string group_name)
{
	ROS_INFO("Vision::RegisterGroup()");
	this->groups_.emplace(group_name, new VisionGroupInterface(group_name));
}

bool Vision::Prepare(std::string group_name)
{
	ROS_INFO("Vision::Prepare()");
	this->groups_[group_name]->Prepare();
}

bool Vision::PrepareAll()
{
	ROS_INFO("Vision::PrepareAll()");
    for (auto&& group : this->groups_)
    {
		group.second->Prepare();
	}
}

bool Vision::UpdateScene()
{
	ROS_INFO("Vision::UpdateScene()");
    for (auto&& group : this->groups_)
    {
		group.second->FindObject();
	}
}

} // namespace o2as
