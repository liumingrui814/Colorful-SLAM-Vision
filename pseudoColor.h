#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <boost/format.hpp>
#include <chrono>
#include <mutex>
//dependency for pcl (just for visualize)
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>

namespace PseudoColor{

    //-- 指定一张浮点图像（CV_32FC1）,指定nan值, 指定伪彩色类型，按伪彩色渲染一张浮点图像
    cv::Mat visualizeRangeImage(cv::Mat rangeMat, float nan_value, cv::ColormapTypes colorTable);

    //-- 生成一个伪彩色的colorBar, 大小为 256x1
    cv::Mat generateColorMap(cv::ColormapTypes mapType);

}