#include "pseudoColor.h"

//-- 可视化一个距离图，默认rangeMat中的有效元素都是大于0的
cv::Mat PseudoColor::visualizeRangeImage(cv::Mat rangeMat, float nan_value, cv::ColormapTypes colorTable)
{
    //-- 有效区域与无效区域的遮罩
    cv::Mat mask(rangeMat.rows, rangeMat.cols, CV_8UC1, cv::Scalar::all(255));
    float min = FLT_MAX, max = -1;
    for (int i = 0;i < rangeMat.rows;i++){
        for(int j = 0; j < rangeMat.cols;j++){
            float value = rangeMat.at<float>(i,j);
            //-- 无效遮罩
            if(value==nan_value){
                mask.at<uint8_t>(i,j) = 0;
            }else{ //-- 有效区域, 寻找最近与最远的点用于渲染
                if(value > max) max = value;
                if(value < min) min = value;
            }
        }
    }
    //-- 根据最大值与最小值来赋值一个用于伪彩色渲染的灰度图
    //-- 有效区域用最大最小值计算比例，无效区域默认255
    cv::Mat image_gray(rangeMat.rows, rangeMat.cols, CV_8UC1, cv::Scalar::all(0));
    for(int i = 0; i < rangeMat.rows; ++i){
        for(int j = 0; j < rangeMat.cols; ++j){
            if(rangeMat.at<float>(i,j) != nan_value){
                double ratio = (rangeMat.at<float>(i,j)-min)/(max-min);
                image_gray.at<uint8_t>(i,j) = 255 *ratio;
            }else{
                image_gray.at<uint8_t>(i,j) = 255;
            }
        }
    }
    cv::applyColorMap(image_gray, image_gray, colorTable);
    //-- 将无效区域设置为黑色
    for(int i = 0; i < image_gray.rows; ++i){
        for(int j = 0; j < image_gray.cols; ++j){
            if(mask.at<uint8_t>(i,j) == 0){
                image_gray.at<cv::Vec3b>(i,j) = cv::Vec3b(0,0,0);
            }
        }
    }
    return image_gray;
}


cv::Mat PseudoColor::generateColorMap(cv::ColormapTypes mapType){
    cv::Mat valueTabel(256,1,CV_8UC1);
    cv::Mat ColorTabel;
    //-- 顺次赋值0-255, 一共256个值, 赋给valueTabel形成灰度图
    for(int i = 0; i<256; i++){
        valueTabel.at<uint8_t>(i,0)=i;
    }
    //-- 用灰度图生成伪彩色的渲染图
    cv::applyColorMap(valueTabel,ColorTabel,mapType);
    return ColorTabel;
}