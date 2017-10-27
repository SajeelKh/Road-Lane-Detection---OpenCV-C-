#include<cmath>
#include"Line.h"
#include"Lane.h"

Line::Line(int x1, int y1, int x2, int y2) {
     //std::cout << "Line Contructor" << endl;
     this->x1 = x1;
     this->y1 = y1;
     this->x2 = x2;
     this->y2 = y2;
     this->a = compute_slope();
     this->b = compute_intercept();
     lane_line = assign_to_lane_line();
}

Line::~Line(){}

vector<int> Line::get_coords() {
     vector<int> temp{x1, y1, x2, y2};
     return temp;
}

int Line::get_x_coord(double y){
     return ((y - this->b) / this->a);

}

int Line::get_y_coord(double x){
     return (this->a * x + this->b);
}

double Line::compute_slope() {
     return (double(this->y2) - double(this->y1)) / (double(this->x2) - double(this->x1));
}

double Line::compute_intercept() {
     return (double(this->y1) - (double(this->a) * double(this->x1)));
}

bool Line::candidate() {
     //std::cout << "Candidate" << endl;
     //std::cout << "///////////////////// " << this->a << endl;
     if (std::abs(this->a) < Lane::MOSTLY_HORIZONTAL_SLOPE) {
          //std::cout << "NEGATIVE!!!!!!!!!" << endl;
          return false;
     }
     Lane lane_line = Lane::select_lane_line(this->lane_line);
     if (lane_line.isInitialized()) {
          //cout << "HELLLLLOOOOOOOO !!!!!!!!!!!!!!!!!!!!!" << endl;
          /*cout << Lane::MAX_SLOPE_DIFFERENCE << endl;
          cout << this->a << " - " << lane_line.coeffs[0] << " = " << abs(this->a - lane_line.coeffs[0]) << endl;*/
          if (abs(this->a - lane_line.coeffs[0]) > Lane::MAX_SLOPE_DIFFERENCE) {
               //cout << "NOPE1" << endl;
               return false;
          }
          if (dist_to_lane_line() > Lane::MAX_DISTANCE_FROM_LINE) {
               //cout << "NOPE2" << endl;
               return false;
          }
          if (this->y2 < Lane::left_line.vanishing_point[1] + 20) {
               //cout << "NOPE3" << endl;
               return false;
          }

          //cout << "SUUUUUCCCCCCESSSSSSSS!!!!!!!!!!!!!" << endl;
     }
     return true;
}

string Line::assign_to_lane_line() {
     if (this->a < 0.0)
          return "left_line";
     else
          return "right_line";
}



double Line::dist_to_lane_line() {
     Lane lane_line = Lane::select_lane_line(this->lane_line);
     if (!lane_line.isInitialized())
          return 0.0;
     float avg_x = (this->x2 + this->x1) / 2;
     float avg_y = (this->y2 + this->y1) / 2;

     double distance = abs(lane_line.get_a() * avg_x - avg_y + lane_line.get_b())
                           / sqrt(pow(lane_line.get_a(), 2) + 1);
     return distance;
}