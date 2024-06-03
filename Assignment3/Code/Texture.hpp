//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture
{
private:
    cv::Mat image_data;

public:
    Texture(const std::string &name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        int u_img = std::floor(u * width);
        int v_img = std::floor((1 - v) * height);

        float h_t = u * width - u_img;
        float v_t = (1 - v) * height - v_img;

        auto color00 = image_data.at<cv::Vec3b>(v_img, u_img);
        auto color10 = image_data.at<cv::Vec3b>(v_img, u_img + 1);
        auto color01 = image_data.at<cv::Vec3b>(v_img + 1, u_img);
        auto color11 = image_data.at<cv::Vec3b>(v_img + 1, u_img + 1);

        auto colorh1 = (1 - h_t) * color00 + h_t * color10;
        auto colorh2 = (1 - h_t) * color01 + h_t * color11;

        auto colorfinal = (1 - v_t) * colorh1 + v_t * colorh2;

        return Eigen::Vector3f(colorfinal[0], colorfinal[1], colorfinal[2]);
    }
};
#endif //RASTERIZER_TEXTURE_H
