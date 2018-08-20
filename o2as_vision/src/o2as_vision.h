#pragma once

#include "o2as_vision_group_interface.h"

namespace o2as {

class Vision
{
public:
	Vision();
	~Vision();

	bool RegisterGroup(std::string group_name);
	bool Prepare(std::string group_name);
	bool PrepareAll();
	bool UpdateScene();

	VisionGroupInterface& GetGroup(std::string group_name) { return *this->groups_[group_name]; }

private:
	std::map<std::string, VisionGroupInterface*> groups_;
};

} // namespace o2as
