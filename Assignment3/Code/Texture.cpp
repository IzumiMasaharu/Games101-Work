//
// Created by LEI XU on 4/27/19.
//

#include "Texture.hpp"

Texture::Texture(const std::string& name)
{
    image_data = cv::imread(name);
    cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
    width = image_data.cols;
    height = image_data.rows;
}

Eigen::Vector3f Texture::getColor(float u, float v)
{
    auto u_img = u * width;
    auto v_img = (1 - v) * height;
    
    auto color = image_data.at<cv::Vec3b>(v_img, u_img);
    return Eigen::Vector3f(color[0], color[1], color[2]);
}