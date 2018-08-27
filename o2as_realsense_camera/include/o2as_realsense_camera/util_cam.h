#include <librealsense2/rs.hpp>

namespace o2as {

typedef struct tagCameraSetting
{
	int		width;
	int		height;
	float	focal_length_x;
	float	focal_length_y;
	float	principal_point_x;
	float	principal_point_y;
	float	dist_param_k1;
	float	dist_param_k2;
	float	dist_param_p1;
	float	dist_param_p2;
	float	dist_param_k3;
} CameraSetting;

bool DeviceExists(std::string serial_number);
void InvMatrix3x3(float src[9], double dst[9]);
void GetPointCloud2(rs2::frame &depth_frame, double depth_scale, double inv_param[9], float *point_cloud);
int SavePointCloudToBinary(float *src_pc, int width, int height, char *fileName);
bool SaveCameraSetting(std::string filename, const CameraSetting& setting);

} // namespace o2as
