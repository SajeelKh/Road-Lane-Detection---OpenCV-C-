#include "Utils.h"
#include <vector>
#include <iostream>
#include <algorithm>
#include <numeric>
#include "opencv2\core\core.hpp"

void Utils::average(std::vector<cv::Vec2d> *buffer, cv::Vec2d &avg) {

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

void Utils::LinearRegression(std::vector<int> x, std::vector<int> y, cv::Vec2d &poly_coeffs) {

     //int myints[] = { 43, 21, 25, 42, 57, 59 };
     //std::vector<int> x(myints, myints + sizeof(myints) / sizeof(int));

     //int myints2[] = { 99, 65, 79, 75, 87, 81 };
     //std::vector<int> y(myints2, myints2 + sizeof(myints2) / sizeof(int));

     //vector <int> x;
     //vector <int> y;

     /*for (size_t i = 0; i < x.size(); i++)
     {
          std::cout << "[" << x[i] << ", " << y[i] << "]";
     }*/
     int n = x.size();
     double sum_x = 0;
     double sum_x2 = 0;
     double sum_y = 0;
     double sum_xy = 0;
     double a = 0;
     double b = 0;

     for (size_t i = 0; i < x.size(); i++)
     {
          sum_x = sum_x + x[i];
          sum_x2 += pow(x[i], 2);
          sum_xy += x[i] * y[i];
          sum_y = sum_y + y[i];
     }

     a = ((sum_y * sum_x2) - (sum_x * sum_xy)) / ((n * sum_x2) - pow(sum_x, 2));
     b = ((n * sum_xy) - (sum_x * sum_y)) / ((n * sum_x2) - pow(sum_x, 2));

     //std::cout << "a: " << a << std::endl << "b: " << b << std::endl;
     poly_coeffs[0] = b;
     poly_coeffs[1] = a;
}

void Utils::vStack(cv::Vec2d *current_lane_line_co, cv::Vec2d *avg_buffer, std::vector<cv::Vec2d> &matrix) {
     
     matrix.push_back(*current_lane_line_co);
     matrix.push_back(*avg_buffer);
}

void Utils::dotProduct(cv::Vec2d *mat, std::vector<cv::Vec2d> *coeffs, cv::Vec2d &lane_coeffs) {
     double a;
     double b;
     /*std::cout << "mat[0]: " << (*mat)[0] * (*coeffs)[0][0] << std::endl;
     std::cout << "(*coeffs)[0][0]: " << (*mat)[1] * (*coeffs)[1][0] << std::endl;
     std::cout << "(*mat)[1]: " << (*mat)[1] << std::endl;
     std::cout << "(*coeffs)[1][0]: " << (*coeffs)[1][0] << std::endl;
     std::cout << "(*coeffs)[0][1]: " << (*coeffs)[0][1] << std::endl;
     std::cout << "(*coeffs)[1][1]: " << (*coeffs)[1][1] << std::endl;*/
     a = ((*mat)[0] * (*coeffs)[0][0]) + ((*mat)[1] * (*coeffs)[1][0]);
     b = ((*mat)[0] * (*coeffs)[0][1]) + ((*mat)[1] * (*coeffs)[1][1]);
     lane_coeffs[0] = a;
     lane_coeffs[1] = b;
}

std::vector<int> Utils::range(int N1, int N2) {
     std::vector<int> numbers(N2 - N1 +1);
     std::iota(numbers.begin(), numbers.end(), N1);
     return numbers;
}

bool Utils::find(std::vector<int> container, int value) {
     //std::cout << container[0] << " " << container[container.size()] << std::endl;
     auto it = std::find(container.begin(), container.end(), value);
     if (it != container.end()) 
          return true;
     else
          return false;
}