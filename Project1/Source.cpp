#include"Utils.h"
#include<vector>
#include<opencv2/core/core.hpp>
#include<iostream>

using namespace std;


int main1() {
    /* int myints[] = { 43, 21, 25, 42, 57, 59 };
     std::vector<int> x(myints, myints + sizeof(myints) / sizeof(int));

     int myints2[] = { 99, 65, 79, 75, 87, 81 };
     std::vector<int> y(myints2, myints2 + sizeof(myints2) / sizeof(int));*/

     int n;

     vector <int> x { 43, 21, 25, 42, 57, 59 };
     vector <int> y { 99, 65, 79, 75, 87, 81 };
     cv::Vec2d result;

     Utils::LinearRegression(x,y,result);
     cout << result;
     cin >> n;
     return 0;

}