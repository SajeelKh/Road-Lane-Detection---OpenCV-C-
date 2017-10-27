#ifndef LINE_H
#define LINE_H

#include<iomanip>
#include<vector>

using namespace std;

class Line
{
public:
     int x1, y1, x2, y2;
     double a, b;
     
public:
     string lane_line;

     Line(int x1, int y1, int x2, int y2);
     ~Line();
     vector<int> get_coords();
     int get_x_coord(double y);
     int get_y_coord(double x);
     double compute_slope();
     double compute_intercept();
     bool candidate();
     string assign_to_lane_line();
     double dist_to_lane_line();

};


#endif

