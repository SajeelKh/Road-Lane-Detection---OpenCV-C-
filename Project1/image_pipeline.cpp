#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include "opencv2/videoio.hpp"

#include<iostream>
#include<conio.h>
#include"Line.h"
#include"Lane.h"
#include"Tools.h"
#include"Utils.h"

using namespace cv;
using namespace std;

int img_y;
int img_x;


int main() {
     int n;
     cv::Mat imgOriginal;        // input image
     cv::Mat imgGrayscale;       // grayscale of input image
     cv::Mat imgBlurred;         // intermediate blured image
     cv::Mat imgCanny;           // Canny edge image
     cv::Mat imgMorphed;
     cv::Mat imgHSV;
     cv::Mat mask_yellow;
     cv::Mat mask_white;
     cv::Mat mask_yw;
     cv::Mat mask_yw_img;
     cv::Mat ROIimg;
     cv::Mat ROIimg2;
     cv::Mat ROI_mask;
     vector<Mat> hsv_planes;

     int lineType = 8;
     char charCheckForEscKey = 0;
     int morph_size = 1;

     //imgOriginal = imread("data/2.png", CV_LOAD_IMAGE_COLOR);

     //VideoCapture cap(1);
     cv::VideoCapture capVideo;

     cv::Mat imgFrame;

     capVideo.open("data/Test_Video.mp4");

     //if (!cap.isOpened())
     if (!capVideo.isOpened()) {                                                     // if unable to open video file
          std::cout << "\nerror reading video file" << std::endl << std::endl;       // show error message
          _getch();                                                                  // it may be necessary to change or remove this line if not using Windows
          return(0);                                                                 // and exit program
     }

     if (capVideo.get(CV_CAP_PROP_FRAME_COUNT) < 1) {
          std::cout << "\nerror: video file must have at least one frame";
          _getch();
          return(0);
     }

     while (charCheckForEscKey != 27 && capVideo.isOpened()) {            // until the Esc key is pressed or webcam connection is lost
          bool blnFrameReadSuccessfully = /*cap.read*/capVideo.read(imgOriginal);     // get next frame*/
          
          //cap >> imgOriginal;
          ::img_x = imgOriginal.cols;
          ::img_y = imgOriginal.rows;
          
          //cout << "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||" << endl;
          if (!blnFrameReadSuccessfully || imgOriginal.empty()) {                 // if frame not read successfully
               std::cout << "error: frame not read from webcam\n";                 // print error message to std out
               break;                                                              // and jump out of while loop
          }

          double fps = capVideo.get(CV_CAP_PROP_FPS);
          //cout << "Frames per second: " << fps << endl;

          Mat mask_img = Mat::zeros(imgOriginal.rows, imgOriginal.cols, CV_8UC1);
          cv::cvtColor(imgOriginal, imgGrayscale, CV_BGR2GRAY);                   // convert to grayscale
          cv::cvtColor(imgOriginal, imgHSV, CV_BGR2HSV);

          //split(imgHSV, hsv_planes);
          //Mat h = hsv_planes[0]; // H channel
          //Mat s = hsv_planes[1]; // S channel
          //Mat v = hsv_planes[2]; // V 

          //imshow("s", hsv_planes[0]);

          Scalar lower_yellow = Scalar( 18, 51, 77 );
          Scalar upper_yellow = Scalar( 33, 255, 255 );

          Scalar lower_white = Scalar(0, 0, 204);
          Scalar upper_white = Scalar(180, 26, 255);
 

          cv::inRange(imgHSV, lower_yellow, upper_yellow, mask_yellow);

          cv::inRange(imgHSV, lower_white, upper_white, mask_white);
          
          cv::bitwise_or(mask_white, mask_yellow, mask_yw);
          
          /*namedWindow("Combined", CV_WINDOW_KEEPRATIO);
          imshow("Combined", mask_yw);*/
          /*namedWindow("yellow", WINDOW_AUTOSIZE);
          imshow("yellow", mask_yellow);
          imshow("img", imgOriginal);*/
          cv::bitwise_and(imgGrayscale, mask_yw, mask_yw_img);

          Mat kernel = getStructuringElement(MORPH_RECT, Size(2 * morph_size + 1, 2 * morph_size + 1), Point(morph_size, morph_size));
          morphologyEx(mask_yw_img, imgMorphed, MORPH_CLOSE, kernel, Point(-1, -1), 1);

         /* namedWindow("Morphed", CV_WINDOW_KEEPRATIO);
          imshow("Morphed", imgMorphed);*/

          cv::GaussianBlur(imgMorphed, imgBlurred, cv::Size(3, 3), 1.0);                    // input image, output image, smoothing window width and height in pixels, blurr intensity

          cv::Canny(imgBlurred, imgCanny, 280, 360);        // input image, output image, low threshold, high threshold                                                                               

          Tools::create_ROI_mask(mask_img);

          /*addWeighted(imgCanny, 1, mask_img, 0.2, 5, ROI_mask);
          imshow("", ROI_mask);*/

          cv::bitwise_and(imgCanny, mask_img, ROIimg);

          std::vector<Line> lines;
          Tools::hough_line_transform(ROIimg, lines, 1, CV_PI / 180, 5, 2, 8);     //image, lines, rho, theta, threshold, min_line_length, max_line_gap

          Tools::update_lane(lines);

          Tools::draw_snap_houghlines_on_roi(ROIimg, ROIimg2, mask_img, lines);

          Lane::draw_lanes(imgOriginal, ROIimg2);

          //imshow("", mask_yw_img);

          charCheckForEscKey = cv::waitKey(1);        // delay (in ms) and get key press, if any
          cin >> n;
          //std::cin.ignore(INT_MAX);
          
     }   // end while

     return(0);
}