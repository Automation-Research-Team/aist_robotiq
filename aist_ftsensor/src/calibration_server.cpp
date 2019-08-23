#include "calibration.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "aist_ftsensor_calibration");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
				    ros::console::levels::Debug);

    try
    {
        aist_ftsensor::calibration node("~");
        node.run();
    }
    catch (const std::exception& err)
    {
        std::cerr << err.what() << std::endl;
        return 1;
    }
    catch (...)
    {
        std::cerr << "Unknown error." << std::endl;
        return 1;
    }

    return 0;
}

