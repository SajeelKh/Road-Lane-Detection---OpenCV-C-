#ifndef TOOLS_H
#define TOOLS_H

#include"opencv2\core\core.hpp"
#include<vector>
#include"Line.h"

namespace Tools {
     void hough_line_transform(cv::Mat image, std::vector<Line> &filtered_lines, int rho, float theta, int threshold, int min_line_length, int max_line_gap);
     void update_lane(vector<Line> segments);
     void draw_snap_houghlines_on_roi(cv::Mat &ROIimg, cv::Mat &ROIimg2, cv::Mat &ROI_mask,  const vector<Line> &lines);
     void updateROI();
     void create_ROI_mask(cv::Mat &mask_img);

}

#endif // !TOOLS_H

