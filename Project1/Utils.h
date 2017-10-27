#ifndef UTIL_H
#define UTIL_H

#include<vector>
#include<opencv2/core/core.hpp>

class Utils {

public:
    static void average(std::vector<cv::Vec2d> *buffer, cv::Vec2d &avg);
    static void LinearRegression(std::vector<int> x, std::vector<int> y, cv::Vec2d &poly_coeffs);
    static void vStack(cv::Vec2d *current_lane_line_co, cv::Vec2d *avg_buffer, std::vector<cv::Vec2d> &matrix);
    static void dotProduct(cv::Vec2d *mat, std::vector<cv::Vec2d> *coeffs, cv::Vec2d &lane_coeffs);
    static std::vector <int> range(int N1, int N2);
    static bool find(std::vector<int> container, int x);
};

#endif