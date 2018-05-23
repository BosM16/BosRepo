#ifndef HELPER_HPP_INCLUDED
#define HELPER_HPP_INCLUDED

///TODO: we should add doxygen style comments and declarations to make documentation easier

#include "opencv2/opencv.hpp"
#include <Eigen/Dense>
#include <fstream>
#include <iterator>

// Line data structure a*x + b*y + c = 0
typedef struct Line {
  double a;
  double b;
  double c;
  Line(double a, double b, double c) : a(a), b(b), c(c) {}
  Line() : a(0), b(0), c(0) {}
} Line;

// Point data structure (x,y)
typedef struct Point {
    double x;
    double y;
    Point(double x, double y) : x(x), y(y) {}
    Point() : x(0), y(0) {}
} Point;

// Interval data structure I = [i1, i2]
typedef struct Interval{
    int i1;
    int i2;
    Interval(int i1, int i2) : i1(i1), i2(i2) {}
    Interval() : i1(0), i2(0) {}
} Interval;

// Operator overload: prints line to console
inline std::ostream& operator<<(std::ostream& os, const Line& l)
{
  return os << "Line: a = " << l.a << ", b = " << l.b << ", c =  " << l.c << '\n';
}

// since all functions are short, they stay here and are not in a cpp

// Closest point to a line
inline cv::Point2d closest_point_p2l(cv::Point2d p1, Line line){
  cv::Point2d p2;
  double v1 = sqrt(line.a*line.a + line.b*line.b);
  double v2 = line.b*p1.x - line.a*p1.y;
  p2.x = (line.b*v2 - line.a*line.c)/v1;
  p2.y = -(line.a*v2 + line.b*line.c)/v1;
  return p2;
}

// Evaluate a line with an x coordinate
inline bool eval_line_x(const double x, const Line l, cv::Point2d &pt){
  if (l.b != 0 ){
    pt.x = x;
    pt.y = -l.a/l.b*x-l.c/l.b;
    return true;
  }else
    return false;
}

// Evaluate a line with a y coordinate
inline bool eval_line_y(const double y, const Line l, cv::Point2d &pt){
  if (l.a != 0 ){
    pt.x = -l.b/l.a*y-l.c/l.a;
    pt.y = y;
    return true;
  }else
    return false;
}

// Convert a line segment determined by end points p1 and p2 to line [a,b,c]
inline Line pts2line(cv::Point2d p1, cv::Point2d p2){
  //    Line l;
  //    l.a = p1.y-p2.y;
  //    l.b = p2.x-p1.x;
  //    l.c = p2.y*p1.x-p2.x*p1.y;
  return Line(p1.y-p2.y, p2.x-p1.x, p2.y*p1.x-p2.x*p1.y);
}

// Checks whether two lines are approximately parallel
inline bool are_parallel(Line l1, Line l2){
  double dot = std::abs(l1.a*l2.a+l1.b*l2.b);
  double n1 = sqrt(l1.a*l1.a+l1.b*l1.b);
  double n2 = sqrt(l2.a*l2.a+l2.b*l2.b);
  
  //    std::cout << "dot:" << dot << std::endl;
  //    std::cout << "n1:" << n1 << std::endl;
  //    std::cout << "n2:" << n2 << std::endl;
  //    std::cout << "dot/n1n2:" << dot/(n1*n2) << std::endl;
  if(dot/(n1*n2) < 0.99)
    return false;
  else
    return true;
}

// Computes the intersection of two lines
inline cv::Point2d intersect(Line l1, Line l2){
  Eigen::Matrix2f A;
  Eigen::Vector2f b;
  std::cout << l1 << l2 << !are_parallel(l1,l2) << std::endl;
  if (!are_parallel(l1,l2)) {
    A << l1.a, l1.b,
    l2.a, l2.b;
    b << -l1.c,-l2.c;
  }
  //    std::cout << "Here is the matrix A:\n" << A << std::endl;
  //    std::cout << "Here is the vector b:\n" << b << std::endl;
  Eigen::Vector2f x = A.colPivHouseholderQr().solve(b);
  //    std::cout << "The solution is:\n" << x << std::endl;
  return cv::Point2d(x(0),x(1));
}

template<typename T>
inline void extrema(std::vector<T> *vec, double eps, std::vector<int> *minima, std::vector<int> *maxima) {
    /*
     * Computes the indices at which extrema are obtained: the vector should only contain strictly positive values
     */

    typedef enum state {
        INITIAL, DECREASE, LEVEL, INCREASE
    } State;

    State state = INITIAL;

    bool end_of_vec = false;
    int i = 2;
    int i_m, i_m1, i_m2;


    while (!end_of_vec) {
        switch (state) {
            case INITIAL:
                if (std::abs((*vec)[1] - (*vec)[0]) < (T) eps) {
                    state = LEVEL;
                    i_m = 0;
                } else if ((*vec)[1] < (*vec)[0]) {
                    state = DECREASE;
                    maxima->push_back(0);
                } else {
                    state = INCREASE;
                    minima->push_back(0);
                }
                break;
            case DECREASE:
                // Follow the monotonically decreasing function till lowest value,
                // if end of vec is reached: this is the minimum
                if (i < vec->size()) {
                    if ((*vec)[i] < (*vec)[i - 1]) {
                        // current element is smaller then previous element
                        i++;
                    } else {
                        // current element is larger then previous element
                        // previous element is the minimum
                        state = LEVEL;
                        i_m = i - 1;
                    }
                } else {
                    minima->push_back(i - 1);
                    end_of_vec = true;
                }
                break;

            case LEVEL:
                // Look in local neighbourhood whether the function stays level within given bounds.
                // The minimum or saddle point is the value in the middle of this interval
                // When it is a minimum (the function increases or vec is empty), the value and corresponding index are stored
            {
                int j;

                bool within_bound = true;
                if (i_m > 0) {
                    j = i_m - 1;
                    while (j >= 0 && within_bound) {
                        if ((std::abs((*vec)[j] - (*vec)[i_m])) < (T) eps)
                            j--;
                        else
                            within_bound = false;
                    }
                    i_m1 = j + 1;
                } else
                    i_m1 = i_m;

                within_bound = true;
                if (i_m < (vec->size() - 1)) {
                    j = i_m + 1;
                    while (j < vec->size() && within_bound) {
                        if ((std::abs((*vec)[j] - (*vec)[i_m])) < (T) eps)
                            j++;
                        else
                            within_bound = false;
                    }
                    i_m2 = j - 1;
                } else
                    i_m2 = i_m;

                if (i_m1 != 0 && i_m2 != (vec->size() - 1)) {         // x im1 im im2 x
                    if ((*vec)[i_m2 + 1] > (*vec)[i_m]) {            //       im im2 +
                        state = INCREASE;
                        if ((*vec)[i_m1 - 1] > (*vec)[i_m])         // + im1 im im2 +
                            minima->push_back((i_m1 + i_m2) / 2);
                    } else if ((*vec)[i_m2 + 1] < (*vec)[i_m]) {     //       im im2 -
                        state = DECREASE;
                        if ((*vec)[i_m1 - 1] < (*vec)[i_m])         // - im1 im im2 -
                            maxima->push_back((i_m1 + i_m2) / 2);
                    }
                    i = i_m2 + 2;

                } else if (i_m1 == 0 && i_m2 != (vec->size() - 1)) { // 0 im1 im im2 x
                    if ((*vec)[i_m2 + 1] > (*vec)[i_m]) {           // 0 im1 im im2 +
                        state = INCREASE;
                        minima->push_back(i_m1);
                    } else {                                     // 0 im1 im im2 -
                        state = DECREASE;
                        maxima->push_back(i_m1);
                    }
                    i = i_m2 + 2;

                } else if (i_m1 != 0 && i_m2 == (vec->size() - 1)) { // x im1 im im2 0
                    if ((*vec)[i_m1 - 1] > (*vec)[i_m]) {            // + im1 im im2 0
                        end_of_vec = true;
                        minima->push_back(i_m2);
                    } else {                                     // - im1 im im2 0
                        end_of_vec = true;
                        maxima->push_back(i_m2);
                    }

                } else {                                         // 0 im1 im im2 0
                    end_of_vec = true;
                }

                break;
            }
            case INCREASE:

                if (i < vec->size()) {
                    if ((*vec)[i] > (*vec)[i - 1]) {
                        // current element is smaller then previous element
                        i++;
                    } else {
                        // current element is larger then previous element
                        // previous element is the minimum
                        state = LEVEL;
                        i_m = i - 1;
                    }
                } else {
                    maxima->push_back(i - 1);
                    end_of_vec = true;
                }
                break;
            default:
                end_of_vec = true;
                break;
        }
    }
}

template<typename T>
inline std::ostream &operator<<(std::ostream &out, const std::vector<T> &v) {
    if (!v.empty()) {
        out << '[';
        if (v.size() > 1)
            std::copy(v.begin(), v.end()-1, std::ostream_iterator<T>(out, ", "));
        out << v[v.size()-1] << "]";
    }
    return out;
}

template<typename T>
inline std::ofstream &operator<<(std::ofstream &out, const std::vector<T> &v) {
    if (!v.empty()) {
        out << '[';
        if (v.size() > 1)
            std::copy(v.begin(), v.end()-1, std::ostream_iterator<T>(out, ", "));
        out << v[v.size()-1] << "]";
    }
    return out;
}

// Dumps a vector to a file

template <typename T>
inline void dump_vector(std::vector<T> *vec, const char* name){
    std::ofstream myfile;
    myfile.open (name);
    myfile << "a" << " = " << *vec << ";" << std::endl;
    myfile.close();
}

template <typename T> T average(std::vector<T> *vec){
    int N = vec->size();
    T sum = 0;
    for (int i = 0; i < N; i++)
        sum += (*vec)[i];
    return sum/N;
}

inline bool fit_line(const std::vector<Point> &pts, Line &line) {
    /*
     *
     * Estimation of line equation a*X + b*Y + c = 0
     * Comparison of variances varX and varY
     *  - when abs(varX)  > abs(varY): more horizontal line -> y = -a/b*x - c/b
     *  - when abs(varX) <= abs(varY): more vertical line -> x = -b/a*y - c/a
     *
     */
    int nb_pts = pts.size();
    if (nb_pts < 2) {
        // Fail: minimum two points are needed for line fitting
        return false;
    }
    double x(0), y(0), xy(0), x2(0), y2(0);

    for (int i = 0; i < nb_pts; i++) {
        x += pts[i].x;
        y += pts[i].y;
        xy += pts[i].x * pts[i].y;
        x2 += pts[i].x * pts[i].x;
        y2 += pts[i].y * pts[i].y;
    }
    x /= nb_pts;
    y /= nb_pts;
    x2 /= nb_pts;
    y2 /= nb_pts;
    xy /= nb_pts;

    double cov = xy - x * y;
    double varY = y2 - y * y;
    double varX = x2 - x * x;

    double a, b;

    if (std::abs(varX) > std::abs(varY)) {
        a = -cov;
        b = varX;
    } else {
        b = -cov;
        a = varY;
    }
    double c = -(a * x + b * y);
    //    double norm = sqrt(a*a + b*b + c*c);
    //    line.a = a/norm;
    //    line.b = b/norm;
    //    line.c = c/norm;
    line.a = a;
    line.b = b;
    line.c = c;

    return true;
}

inline void polar2cartesian(std::vector<float> *ranges, double angle_min, double angle_increment, std::vector<Point> *coordinates){
    double alpha = angle_min;
    for (unsigned int i = 0; i < ranges->size(); ++i) {
        double x = (*ranges)[i] * std::cos(alpha);
        double y = (*ranges)[i] * std::sin(alpha);
        coordinates->push_back(Point(x, y));
        alpha += angle_increment;
    }

}

#endif