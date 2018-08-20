#include <librealsense2/rs.hpp>

namespace o2as {

bool DeviceExists(std::string serial_number);
void  InvMatrix3x3(float src[9], double dst[9]);
void  GetPointCloud2(rs2::frame &depth_frame, double depth_scale, double inv_param[9], float *point_cloud);
int SavePointCloudToBinary(float *src_pc, int width, int height, char *fileName);

} // namespace o2as
