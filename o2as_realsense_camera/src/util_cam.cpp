#include "o2as_realsense_camera/util_cam.h"

#include <cmath>
#include <cstring>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;

namespace o2as {

bool DeviceExists(std::string serial_number)
{
    rs2::context ctx;    
    for (auto&& dev : ctx.query_devices()) 
    {
        std::string s = std::string(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
        if (s == serial_number) {
            return true;
        }
    }
    return false;
}

void InvMatrix3x3(float src[9], double dst[9])
{
	double	det, inv_det;

	det = (double)src[0] * src[4] * src[8] + src[3] * src[7] * src[2] + src[6] * src[1] * src[5] - src[0] * src[7] * src[5] - src[6] * src[4] * src[2] - src[3] * src[1] * src[8];

	if (fabs(det) < 1e-6) {
		memset(dst, 0, 9 * sizeof(double));
	}
	else {
		inv_det = 1.0 / det;
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

void GetPointCloud2(rs2::frame &depth_frame, double depth_scale, double inv_param[9], float *point_cloud)
{
	int     		x, y, pos;
	double  		proj_x, proj_y, depth;
	unsigned short  *ptr_depth;
	int     		width  = depth_frame.as<rs2::video_frame>().get_width();
	int     		height = depth_frame.as<rs2::video_frame>().get_height();
	cv::Mat 		cv_depth_image = cv::Mat(height, width, CV_16SC1, const_cast<void*>(depth_frame.get_data()));
	const float		max_depth = 1e+6f + 1.0f;

#pragma omp parallel for firstprivate(x, ptr_depth, proj_x, proj_y, depth, pos)
	for (y = 0; y < height; y++) {
		ptr_depth = cv_depth_image.ptr<unsigned short>(y);
		pos = y * width * 3;
		for (x = 0; x < width; x++) {
			if (ptr_depth[x] > 0) {
				proj_x = inv_param[0] * (double)x + inv_param[1] * (double)y + inv_param[2];
				proj_y = inv_param[3] * (double)x + inv_param[4] * (double)y + inv_param[5];
				depth  = 1000.0 * depth_scale  * (double)ptr_depth[x];
				point_cloud[pos  ] = (float)(proj_x * depth);
				point_cloud[pos+1] = (float)(proj_y * depth);
				point_cloud[pos+2] = (float)depth;
			}
			else {
				point_cloud[pos  ] = 0.0f;
				point_cloud[pos+1] = 0.0f;
				point_cloud[pos+2] = max_depth;
			}
			pos += 3;
		}
	}
}

int SavePointCloudToBinary(float *src_pc, int width, int height, char *fileName)
{
	FILE *fout = NULL;
	fout = fopen(fileName, "wb");
	if (fout == NULL) {
		printf("cannot open point cloud file - %s\n", fileName);
		return -1;
	}

	fwrite(src_pc, sizeof(float), width * height * 3, fout);
	fclose(fout);
	return 0;
}

template <class T>
void WriteKeyValue(ofstream& f, std::string key, T value, std::string indent = "  ", std::string space = " ") 
{
	f << indent;
	f << std::setw(20) << std::left << key << ":";
	f << space << std::setw(10) << std::left << to_string(value);
	f << std::endl;
	// f << indent << key << ":" << space << to_string(value) << std::endl;
}

bool SaveCameraSetting(std::string filename, const CameraSetting& setting)
{
	ofstream f;
	f.open((char*)filename.c_str());
	string indent = "  ";
	string space = " ";
	f << "camera_setting: \n";
	WriteKeyValue<int>(f, "width", setting.width);
	WriteKeyValue<int>(f, "height", setting.height);
	WriteKeyValue<float>(f, "focal_length_x", setting.focal_length_x);
	WriteKeyValue<float>(f, "focal_length_y", setting.focal_length_y);
	WriteKeyValue<float>(f, "principal_point_x", setting.principal_point_x);
	WriteKeyValue<float>(f, "principal_point_y", setting.principal_point_y);
	WriteKeyValue<float>(f, "dist_param_k1", setting.dist_param_k1);
	WriteKeyValue<float>(f, "dist_param_k2", setting.dist_param_k2);
	WriteKeyValue<float>(f, "dist_param_p1", setting.dist_param_p1);
	WriteKeyValue<float>(f, "dist_param_p2", setting.dist_param_p2);
	WriteKeyValue<float>(f, "dist_param_k3", setting.dist_param_k3);
	f.close();
	return 0;
}

} // namespace o2as
