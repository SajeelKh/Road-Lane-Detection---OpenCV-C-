#include"Lane.h"
#include"Line.h"
#include<cmath>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

extern int img_y;
extern int img_x;

namespace Tools {

     int lineType = 8;

     void create_ROI_mask(cv::Mat &mask_img) {

          Point roi_points[1][5];
          roi_points[0][0] = Point(img_x, img_y);           //Bottom Right
          roi_points[0][1] = Point(0, img_y);               //Bottom Left
          ///roi_points[0][2] = Point(0, img_y - 100);       
          
          //roi_points[0][2] = Point(280, 230);             //Top Left
          //roi_points[0][3] = Point(380, 230);             //Top Right
          roi_points[0][2] = Point(550, 450);               //Top Left
          roi_points[0][3] = Point(740, 450);               //Top Right

          const Point* ppt[1] = { roi_points[0] };
          int no_Points[] = { 4 };

          fillPoly(mask_img,
               ppt,
               no_Points,
               1,
               Scalar(255, 255, 255),
               lineType);
          //imshow("",mask_img);
     }

     void hough_line_transform(cv::Mat image, vector<Line> &filtered_lines, int rho, float theta, int threshold, int min_line_length, int max_line_gap) {
          //std::cout << "Hough" << endl;
          std::vector<Vec4i> lines;
          cv::HoughLinesP(image, lines, rho, theta, threshold, min_line_length, max_line_gap);
          /*for (Vec4i l : lines) {
               cout << "[ " << l[0] << ", " << l[1] << ", " << l[2] << ", " << l[3] << " ]" << endl;
               cv::line(img, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 1, CV_AA);
               imshow("roi", image);
               imshow("Hough lines", img);
          }*/
          if (!lines.empty()) {
               for (size_t i = 0; i < lines.size(); i++)
               {
                    Line line = Line(lines[i][0], lines[i][1], lines[i][2], lines[i][3]);
                    if (line.candidate())
                         filtered_lines.push_back(line);
               }
          }
          /*for (Line l : filtered_lines) {
               if (l.lane_line == "left_line") {
                    cout << "[ " << l.x1 << ", " << l.y1 << ", " << l.x2 << ", " << l.y2 << " ]" << endl;
               }
          }*/
     }

     void update_lane(vector<Line> segments) {
          //cout << segments.size() << endl;
          //std::cout << "update lane" << endl;
          vector<Line> left;
          vector<Line> right;
          if (!segments.empty()) {
               for (size_t i = 0; i < segments.size(); i++)
               {
                    //std::cout << "BRRRROOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO" << endl;
                    if (segments[i].lane_line == "left_line")
                         left.push_back(segments[i]);
                    else if (segments[i].lane_line == "right_line")
                         right.push_back(segments[i]);
               }

               if (!Lane::left_line.isInitialized() && !Lane::right_line.isInitialized()) {

                    Lane::left_line = Lane(left, "left");
                    //cout << Lane::left_line.coeffs << endl;
                    Lane::right_line = Lane(right, "right");
               }

               Lane::update_vanishing_point(Lane::left_line, Lane::right_line);
               Lane::left_line.update_lane_line(&left);
               Lane::right_line.update_lane_line(&right);

          }
     }

     void draw_snap_houghlines_on_roi(cv::Mat &ROIimg, cv::Mat &ROIimg2, cv::Mat &ROI_mask, const vector<Line> &lines) {
          cvtColor(ROIimg, ROIimg, CV_GRAY2BGR);
          cvtColor(ROI_mask, ROI_mask, CV_GRAY2BGR);
          //ROI_mask.convertTo(ROI_mask, CV_8UC3);
          Scalar color;
          for (size_t i = 0; i < lines.size(); i++)
          {
               Line l = lines[i];
               if (l.lane_line == "right_line") {
                    color = Scalar(0, 255, 0);
               }
               if (l.lane_line == "left_line") {
                    color = Scalar(0, 0, 255);
               }
               
               //std::cout << Point(l.x1, l.y1) << "," << Point(l.x2, l.y2) << endl;
               cv::line(ROIimg, Point(l.x1, l.y1), Point(l.x2, l.y2), color, 1, CV_AA);
          }

          //imshow("", ROI_mask);
          cv::addWeighted(ROIimg, 1, ROI_mask, 0.2, 5, ROIimg);
          //imshow("s", ROIimg);
          //imshow("a", imgMorphed);
          //Size size(200, 150);//the dst image size,e.g.100x100
          Size size(500, 300);
          //Size size(600, 400);
          resize(ROIimg, ROIimg2, size);//resize image
     }

     

     void updateROI() {

     }

}