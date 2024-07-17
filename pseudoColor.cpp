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


//-- 生成一个伪彩色的colorBar, 大小为 256x1
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


//-- 可视化一个边长(radius*2+1)正方形的光流颜色映射图
cv::Mat OpticFlowColor::generatePixel2DViewMap(int radius)
{
    int img_width = radius * 2 + 1;
    cv::Mat opticMap(img_width, img_width, CV_8UC3, cv::Scalar::all(0));
    cv::cvtColor(opticMap, opticMap, cv::COLOR_BGR2HSV);
    double max_norm = sqrt(2) * radius;
    //-- i是行索引(y方向索引)
    for(int i = 0; i < img_width; ++i)
    {
        //-- j是列索引(x方向索引)
        for(int j = 0; j < img_width; ++j)
        {
            //-- 获得[i,j]位置对应的光流向量[x,y]
            int x = j - radius;
            int y = i - radius;
            //-- 获得光流向量[x,y]的atan2角度, 取值范围为[0.2pi]
            double theta = atan2(y,x);
            if(theta < 0) theta += 2*M_PI;
            //-- 获得光流向量[x,y]的模长
            double norm = sqrt(x*x + y*y);
            //-- 计算角度对应的HSV的H值
            int h_val = cvRound(90.0*theta/M_PI);
            //-- 计算模长对应的HSV的SV值
            int s_val = cvRound(double(norm) / double(max_norm) * 255);
            opticMap.at<cv::Vec3b>(i,j) = cv::Vec3b(h_val, s_val, 255);
        }
    }
    cv::cvtColor(opticMap, opticMap, cv::COLOR_HSV2BGR);
    return opticMap;
}


//-- 根据最大的光流模长生成一个二维向量的颜色
cv::Vec3b OpticFlowColor::pixel2valueHSV(cv::Vec2f pixel, float maxRadius)
{
    float x = pixel[0];
    float y = pixel[1];
    //-- 获得光流向量[x,y]的atan2角度, 取值范围为[0.2pi]
    double theta = atan2(y, x);
    if(theta < 0) theta += 2*M_PI;
    //-- 获得光流向量[x,y]的模长
    double norm = sqrt(x*x + y*y);
    //-- 计算角度对应的HSV的H值
    int h_val = cvRound(90.0*theta/M_PI);
    //-- 计算模长对应的HSV的SV值
    int s_val = cvRound(double(norm) / double(maxRadius) * 255);
    cv::Vec3b hsv_val = cv::Vec3b(h_val, s_val, 255);
    return hsv_val;
}


//-- 使用HSV色表渲染一个二维光流图
cv::Mat OpticFlowColor::visualizePixel2DMap(cv::Mat matPixel2D)
{
    cv::Mat vizMat(matPixel2D.rows, matPixel2D.cols, CV_8UC3, cv::Scalar::all(0));
    //-- 首先寻找最大的norm
    float maxRadius = -1;
    for(int i = 0; i < matPixel2D.rows; ++i){
        for(int j = 0; j < matPixel2D.cols; ++j){
            cv::Vec2f val = matPixel2D.at<cv::Vec2f>(i,j);
            float radius = sqrt(val[0] * val[0] + val[1] * val[1]);
            if(radius > maxRadius) maxRadius = radius;
        }
    }
    //-- 根据最大的radius渲染颜色
    for(int i = 0; i < matPixel2D.rows; ++i){
        for(int j = 0; j < matPixel2D.cols; ++j){
            cv::Vec2f val = matPixel2D.at<cv::Vec2f>(i,j);
            cv::Vec3b hsv_val = pixel2valueHSV(val, maxRadius);
            vizMat.at<cv::Vec3b>(i,j) = hsv_val;
        }
    }
    cv::cvtColor(vizMat, vizMat, cv::COLOR_HSV2BGR);
    return vizMat;
}


//-- 使用HSV色表生成一个1x180的色表，用于对应0-360度的旋转
cv::Mat OrientationColor::generateOrientationMap()
{
    cv::Mat hsvTabel(1,180,CV_8UC3,cv::Scalar::all(0));
    for(int i = 0; i < 180; i++){
        hsvTabel.at<cv::Vec3b>(0,i) = cv::Vec3b(i,255,255);
    }
    cv::cvtColor(hsvTabel, hsvTabel, cv::COLOR_HSV2BGR);
    return hsvTabel;
}