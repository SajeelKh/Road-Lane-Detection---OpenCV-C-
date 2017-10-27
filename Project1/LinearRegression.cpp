#include <iomanip>
#include <vector>
#include<opencv2/core/core.hpp>

using namespace std;

void LinearRegression(vector<int> x, vector<int> y, cv::Vec2d &poly_coeffs) {

     //int myints[] = { 43, 21, 25, 42, 57, 59 };
     //std::vector<int> x(myints, myints + sizeof(myints) / sizeof(int));

     //int myints2[] = { 99, 65, 79, 75, 87, 81 };
     //std::vector<int> y(myints2, myints2 + sizeof(myints2) / sizeof(int));

     //vector <int> x;
     //vector <int> y;

     /*for (size_t i = 0; i < x.size(); i++)
     {
     cout << x[i] << ", " << y[i] << endl;
     }*/
     int n = x.size();
     int sum_x = 0;
     int sum_x2 = 0;
     int sum_y = 0;
     int sum_xy = 0;
     double a = 0;
     double b = 0;

     for (size_t i = 0; i < x.size(); i++)
     {
          sum_x = sum_x + x[i];
     }

     for (size_t i = 0; i < x.size(); i++)
     {
          sum_x2 += pow(x[i], 2);
     }

     for (size_t i = 0; i < x.size(); i++)
     {
          sum_xy += x[i] * y[i];
     }

     for (size_t i = 0; i < y.size(); i++)
     {
          sum_y = sum_y + y[i];
     }

     a = ((sum_y * sum_x2) - (sum_x * sum_xy)) / (n * sum_x2 - pow(sum_x, 2));
     b = ((n * sum_xy) - (sum_x * sum_y)) / (n * sum_x2 - pow(sum_x, 2));

     ///cout << "a: " << a << endl << "b: " << b << endl;
     poly_coeffs[0] = a;
     poly_coeffs[1] = b;
}