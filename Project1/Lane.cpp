extern int img_x;
extern int img_y;

#define PI 3.14159265

#include<iomanip>
#include<vector>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include"Lane.h"
#include"Utils.h"

using namespace std;
using namespace cv;

float Lane::CRITICAL_SLOPE_CHANGE = 0.1f;
float Lane::MOSTLY_HORIZONTAL_SLOPE = 0.4f;
float Lane::MAX_SLOPE_DIFFERENCE = 0.8f;
int Lane::MAX_DISTANCE_FROM_LINE = 20;
int Lane::BUFFER_FRAMES = 10;
double Lane::lane_offset = 0;

Lane Lane::left_line = Lane();
Lane Lane::right_line = Lane();

Lane::Lane() {
     
}

Lane::Lane(vector<Line> segments, string name) {
     //std::cout << "LANE CONSTRUCTOR" << endl;
     this->name = name;
     int buffer_frames = Lane::BUFFER_FRAMES;
     fit_lane_line(&segments, this->current_lane_line_coeffs);
     //cout << "Current_Coefficients: " << current_lane_line_coeffs[0] << ", " << current_lane_line_coeffs[1] << endl;
     this->buffer.assign(buffer_frames, current_lane_line_coeffs);
     /*for each (Vec2d var in this->buffer)
     {
          cout << "BUFFER" << endl;
          cout << var << endl;
     }*/
     this->coeffs = this->buffer[0];
     //cout << "Coeffs: " << this->coeffs << endl;
     this->stable = true;
     this->x1, this->x2, this->y1, this->y2 = 0;
     this->initialized = true;
}

void init_lane_line(vector<Line>) {

}

bool Lane::isInitialized() {
     return initialized;
}

Lane& Lane::select_lane_line(string str) {
     if (str == "left_line")
          return Lane::left_line; 
     else
          return Lane::right_line;
}

void Lane::fit_lane_line(vector<Line> *segments, Vec2d &cllc) {

     //cout << "FIT_LANE_LINE" << endl;
     vector<int> x;
     vector<int> y;

     //cout << x.empty() << Lane::left_line.isInitialized() << Lane::right_line.isInitialized();
     if (!Lane::left_line.isInitialized() || !Lane::right_line.isInitialized()) {
          //cout << "______________________________________________YES___________________" << endl;
          String lane_line = (*segments)[0].lane_line;
          
          //cout << (*segments)[0].lane_line << ": " << endl;
          for (size_t i = 0; i < segments->size(); i++)
          {
               Line l = (*segments)[i];
               //cout << l.x1 << ", " << l.y1 << ", " << l.x2 << ", " << l.y2 << endl;
               //cout << l.lane_line << "  x1:   " << l.x1 << ",        " << Utils::find(Lane::FIRST_FRAME_RANGES[lane_line], l.x1) << endl;
               
               if (Utils::find(Lane::FIRST_FRAME_RANGES[lane_line], l.x1)) {
                    //cout << "RANGE: " << l.x1 << ", " << Utils::find(Lane::FIRST_FRAME_RANGES[lane_line], l.x1) << endl;
                    x.push_back(l.x1);
                    x.push_back(l.x2);
                    y.push_back(l.y1);
                    y.push_back(l.y2);
               }
               
          }
          /*cout << "X:" << endl;
          for (size_t i = 0; i < x.size(); i++)
          {
               
               cout << x[i] << " ";
          }*/
     }
     else {
          //cout << "______________________________________________NO___________________" << endl;

          for (size_t i = 0; i < segments->size(); i++)
          {
               Line l = (*segments)[i];
               //cout << l.x1 << ", " << l.y1 << ", " << l.x2 << ", " << l.y2 << endl;
               x.push_back(l.x1);
               x.push_back(l.x2);
               y.push_back(l.y1);
               y.push_back(l.y2);
          }

     }

     if (!x.empty()) {
          Utils::LinearRegression(x, y, cllc);
          //cout << "Coefficients: " << cllc[0] << ", " << cllc[1] << endl;
     }
}

void Lane::update_vanishing_point(Lane &left, Lane &right) {
     //cout << "UPDATE VANISHING POINT: " << endl;
     Vec2d equation;
     double x;
     double y;
     //cout << left.coeffs[0] << " - " << right.coeffs[0] << endl;
     equation[0] = left.coeffs[0] - right.coeffs[0];
     //cout << left.coeffs[1] << " - " << right.coeffs[1] << endl;
     equation[1] = left.coeffs[1] - right.coeffs[1];
     //cout << "EQUATION: " << equation[0] << ", " << equation[1] << endl;
     x = -equation[1] / equation[0];
     //cout << "x: " << x << endl;
     y = (left.coeffs[0] * x) + left.coeffs[1];
     //cout << "y: " << y << endl;
     x = int(x);
     y = int(y);
     left.vanishing_point[0] = x;
     left.vanishing_point[1] = y;
     right.vanishing_point[0] = x;
     right.vanishing_point[1] = y;
}

void Lane::update_lane_line(vector<Line> *segments) {
     //cout << "SEGMENT 1: " << segments << endl;
     Vec2d average_buffer;
     Utils::average(&this->buffer, average_buffer);
     //average_buffer[0] += 100;
     //average_buffer[1] += 400;
     //cout << "average_buffer: " << average_buffer << endl;
     this->coeffs = average_buffer;
     //cout << "coeffs: " << coeffs << endl;
     update_current_lane_line_coeffs(segments);
     //cout << "Stable: " << this->stable << endl;
     //stable = false;
     Vec2d weights = Lane::DECISION_MAT[this->stable];
     //cout << "Weights: " << weights << endl;
     vector<Vec2d> matrix;
     Vec2d current_buffer_coeffs;
     Utils::vStack(&this->current_lane_line_coeffs, &average_buffer, matrix);
     /*for (Vec2d var : matrix) {
          cout << "Matrix: " << var << endl;
     }*/
     //cout << endl;
     Utils::dotProduct(&weights, &matrix, current_buffer_coeffs);
     //cout << "CURRENT BUFFER COEFFICIENTS: " << current_buffer_coeffs << endl;
     std::vector<Vec2d>::iterator it;
     it = this->buffer.begin();
     this->buffer.insert(it, current_buffer_coeffs);
     this->buffer.pop_back();
     //cout << "BUFFER AGAIN:" << endl;
     //cout << "-----------------------------" << endl;
    /* for each (Vec2d var in this->buffer)
     {
          cout << var << endl;
     }*/
     //cout << "-----------------------------" << endl;
     calculate_angle();
     update_lane_line_coords();
     find_lane_offset();
}

void Lane::update_current_lane_line_coeffs(vector<Line> *segments) {
     Vec2d average_buffer;
     Vec2d lane_line_coeffs;
     Utils::average(&this->buffer, average_buffer);
     //cout << "SEGMENT 2: " << segments << endl;
     fit_lane_line(segments, lane_line_coeffs);
     //cout << "Lane Line Coefficients" << lane_line_coeffs << endl;
     if (lane_line_coeffs[0] == 0 && lane_line_coeffs[1] == 0) {
          lane_line_coeffs = average_buffer;
     }
     double buffer_slope = average_buffer[0];
     //cout << "Buffer SLOPE: " << buffer_slope << endl;
     double current_slope = lane_line_coeffs[0];
     double buffer_intercept = average_buffer[1];
     double current_intercept = lane_line_coeffs[1];
     this->current_lane_line_coeffs = lane_line_coeffs;
     if ((abs(current_slope - buffer_slope) > Lane::CRITICAL_SLOPE_CHANGE) || (abs(current_intercept - buffer_intercept) > 100))
          this->stable = false;
     else
          this->stable = true;
}
      
void Lane::update_lane_line_coords() {
     //cout << "UPDATE LANE LINE COORDS: " << endl;
     int visual_offset = 50;
     this->y1 = img_y;
     this->x1 = get_x_coord(this->y1);
     //cout << "x1: " << x1 << endl;
     this->y2 = this->vanishing_point[1] + visual_offset;
     //cout << "y1: " << y1 << endl;
     this->x2 = get_x_coord(this->y2);
     //cout << "x2: " << x2 << endl;
     //cout << "y2: " << y2 << endl;
}

void Lane::calculate_angle() {
     double slope = this->get_a();
     int slope_ratio = slope * 100;
     //cout << slope_ratio << endl;
     cout << "ANGLE: " << this->name << ": " << (atan(slope) * (180/PI)) << endl;
     this->angle = atan(slope) * (180 / PI);
}

//double calculate_center_offset() {
//     double ym_per_pix = 30 / 720;   //meters per pixel in y dimension
//     double xm_per_pix = 3.7 / 700;  //meters per pixel in x dimension
//
//     double xMax = img.shape[1] * xm_per_pix;
//     double yMax = img.shape[0] * ym_per_pix;
//     double vehicleCenter = xMax / 2;
//
//
//}

int Lane::get_x_coord(int y) {
     //cout << "GET X COORD: " << this->coeffs[1] << ", " << this->coeffs[0] << ", " << y << endl;
     return int((double(y) - this->coeffs[1]) / this->coeffs[0]);
}

double Lane::get_a() {
     return this->coeffs[0];
}

double Lane::get_b() {
     return this->coeffs[1];
}

void Lane::find_lane_offset() {
     int img_center_x = img_x / 2;
     //cout << "OOOOOOOOOOOOOOOOOOOOFFSET:" << endl;
     //cout << img_center_x << endl;
     int lane_width = abs(Lane::left_line.x1 - Lane::right_line.x1);
     //cout << lane_width << endl;
     int lane_center_x = floor((Lane::left_line.x1 + Lane::right_line.x1) / 2);
     //cout << lane_center_x << endl;
     int pixel_offset = img_center_x - lane_center_x;
     //cout << pixel_offset << endl;
     double lane_width_m = 3.7;  // How wide we expect the lane to be in meters
     Lane::lane_offset = lane_width_m * (double(pixel_offset) / double(lane_width));
     Lane::lane_offset = round(Lane::lane_offset * 100) / 100;
     //cout << Lane::lane_offset << endl;

}

void Lane::draw_lanes(Mat img, Mat hough_img) {
     //cout << "DRAW LANES" << endl;
     int lineType = 8;
     Mat lane_img = Mat::zeros(img_y, img_x, CV_8UC3);
     Mat lanes = Mat::zeros(img_y, img_x, CV_8UC3);

     Point roi_points[1][4];
     roi_points[0][0] = Point(Lane::right_line.x1, Lane::right_line.y1);              //Bottom Right
     roi_points[0][1] = Point(Lane::left_line.x1, Lane::left_line.y1);              //Bottom Left
     roi_points[0][2] = Point(Lane::left_line.x2, Lane::left_line.y2);              //Top Left
     roi_points[0][3] = Point(Lane::right_line.x2, Lane::right_line.y2);              //Top Right

     const Point* ppt[1] = { roi_points[0] };
     int no_Points[] = { 4 };

     fillPoly(lanes,
          ppt,
          no_Points,
          1,
          Scalar(0, 255, 255),
          lineType);
     //cout << "img_y/2: " << img_y / 2 << endl;
     cv::line(lanes, Point(Lane::left_line.x1, Lane::left_line.y1), Point(Lane::left_line.x2, Lane::left_line.y2), Scalar(0, 0, 255), 3, CV_AA);
     cv::line(lanes, Point(Lane::right_line.x1, Lane::right_line.y1), Point(Lane::right_line.x2, Lane::right_line.y2), Scalar(0, 0, 255), 3, CV_AA);
     //cv::line(lanes, Point(img_x/2, 0), Point(img_x/2, img_y), Scalar(0, 0, 255), 1, CV_AA);

     string Lane_Offset = to_string(abs(Lane::lane_offset));
     char buffer[7];
     std::size_t length = Lane_Offset.copy(buffer, 4, 0);
     buffer[length] = '\0';

     string offset = string("Lane Offset: ") + buffer + string("m ");
     if (Lane::lane_offset < 0) {
          offset = offset + string("Left");
     }

     else if (Lane::lane_offset > 0) {
          offset = offset + string("Right");
     }

     else if (Lane::lane_offset == 0) {
          offset = string("Lane Offset: ") + string("In centre");
     }
     
     //putText(lanes, offset , Point(img_x-120, 25), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 2);
     putText(lanes, offset, Point(img_x - 600, 35), FONT_HERSHEY_SIMPLEX, 1.5, Scalar(0, 0, 255), 3);

     cv::addWeighted(lanes, 0.5, img, 0.7, 20, lane_img);
     hough_img.copyTo(lane_img(cv::Rect(0, 0, hough_img.cols, hough_img.rows)));
     cv::namedWindow("Lanes",CV_WINDOW_KEEPRATIO);
     cv::imshow("Lanes", lane_img);
}