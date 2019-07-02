/*!
  \file		calibrator_main.cpp
  \brief	Entry point for calibrator node
*/
#include "Calibrator.h"
#include "ros/ros.h"

int
main(int argc, char **argv)
{
    ros::init(argc, argv, "calibrator");
    o2as_easy_handeye::Calibrator().spin();

    return 0;
}
