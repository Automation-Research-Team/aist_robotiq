/*!
  \file		calibrator_main.cpp
  \brief	Entry point of calibrator node
*/
#include "Calibrator.h"
#include "ros/ros.h"

int
main(int argc, char **argv)
{
    ros::init(argc, argv, "calibrator");
    aist_handeye_calibration::Calibrator().spin();

    return 0;
}
