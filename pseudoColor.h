#ifndef PSEUDO_COLOR_H
#define PSEUDO_COLOR_H
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

}//namespace PseudoColor

namespace OpticFlowColor{

    /**
     * @brief 可视化一个边长(radius*2+1)正方形的光流颜色映射图，将光流(x,y)映射为颜色m
     * @param radius 可视化的最大光流的长，即map大小为 (radius*2+1)x(radius*2+1),
     *               此时默认两个像素的长度一定小于radius（如EDF算出来最大200，那radius可取200）
     * @details (x,y)对应的颜色矩阵的位置即(center+x, center+y), 其中center坐标为[radius,radius]
     *          矩阵满足：不同朝向是不同颜色，HSV的H通道顺着360度，距离越大则饱和度越高，亮度始终最高
    */
    cv::Mat generatePixel2DViewMap(int radius);

    /**
     * @brief 根据最大的光流模长生成一个二维向量的颜色
     * @param pixel 二维向量，即光流的[dx,dy]
     * @param maxRadius 最大模，最大模的颜色为满饱和度满亮度
    */
    cv::Vec3b pixel2valueHSV(cv::Vec2f pixel, float maxRadius);

    /**
     * @brief 使用HSV伪彩色渲染一个二维光流图
     * @param matPixel2D CV_32FC2的图像，每个[x,y]位置的Vec2f代表一个光流向量
     *        [0]是图像坐标系x方向的偏移量
     *        [1]是图像坐标系y方向的偏移量
    */
    cv::Mat visualizePixel2DMap(cv::Mat matPixel2D);

}//namespace OpticFlowColor

namespace OrientationColor{

    //-- 使用HSV色表生成一个1x180的色表，用于对应0-360度的旋转向量
    cv::Mat generateOrientationMap();

}//-- namespace OrientationColor

#endif