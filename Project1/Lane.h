#ifndef LANE_H
#define LANE_H

#include <iostream>
#include<iomanip>
#include<vector>
#include <unordered_map>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include"Line.h"
#include"Utils.h"

extern int img_x;
extern int img_y;

using namespace std;
using namespace cv;

class Lane {

public:
     string name;
     int x1, x2, y1, y2 = 0;
     bool stable;
     bool initialized = false;
     double angle;
     //vector<Line> segment;
     //vector<int> points;
     Vec2d current_lane_line_coeffs;
     vector<Vec2d> buffer;
     Vec2d coeffs;

     int vanishing_point[2];
     static float CRITICAL_SLOPE_CHANGE;
     static float MOSTLY_HORIZONTAL_SLOPE;
     static float MAX_SLOPE_DIFFERENCE;
     static int MAX_DISTANCE_FROM_LINE;
     static int BUFFER_FRAMES;
     static double lane_offset;

     vector<Vec2f> DECISION_MAT{ { 0.1f, 0.9f },{ 1.0f, 0.0f } };
     unordered_map<string, vector<int>> FIRST_FRAME_RANGES {{"left_line", Utils::range(0,img_x/2)}, {"right_line", Utils::range(img_x/2, img_x)}};
     //Mat DECISION_MAT = (Mat_<int>(3, 3) << 1, 0, 1, 0, -2, 0, 3, 0, 3);

     static Lane left_line;
     static Lane right_line;

     Lane();
     Lane(vector<Line> segments, string name);
     void init_lane_line(vector<Line>);
     static bool lines_exists();
     void fit_lane_line(vector<Line> *segments, Vec2d &cllc);
     static void update_vanishing_point(Lane &left, Lane &right);
     double get_a();
     double get_b();
     void update_lane_line(vector<Line> *segments);
     void update_current_lane_line_coeffs(vector<Line> *segments);
     int get_x_coord(int y);
     void update_lane_line_coords();
     bool isInitialized();
     static Lane& select_lane_line(string str);
     static void draw_lanes(Mat img, Mat hough_img);
     void calculate_angle();
     void calculate_center_offset();
     void find_lane_offset();

};

#endif