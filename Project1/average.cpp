#include<iostream>
#include<vector>
#include<opencv2/core/core.hpp>

void average(std::vector<cv::Vec2d> *buffer, cv::Vec2d &avg) {

     double a = 0.0;
     double b = 0.0;
     for (cv::Vec2d var : (*buffer))
     {
          a = a + var[0];
          b = b + var[1];
     }
     a = a / buffer->size();
     b = b / buffer->size();
     avg[0] = a;
     avg[1] = b;
}