#define TRACK_OPTFLOW
#include <opencv2/opencv.hpp>            // C++
#include <opencv2/core/version.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <thread>
#include <fstream>
#include <iostream>

#include "CStepThreadReplaceable.h"
#include "CReplaceableObject.h"
#include "yolo_wrapper.h"

#include "CTracker.h"
#include "CLineTracker.h"
#include "CJuda.h"
using namespace std;
using namespace cv;


const int alpha_slider_max = 255;
int alpha_slider = 0;
double alpha;
double beta;
Mat src1;
Mat src2;
Mat dst;
static void on_trackbar(int, void*)
{
    alpha = (double)alpha_slider / alpha_slider_max;
    beta = (1.0 - alpha);
    //addWeighted(src1, alpha, src2, beta, 0.0, dst);
    //imshow("Linear Blend", dst);
}


void CLineTracker::Update(cv::Mat& draw_frame) {
    cv::Mat grayed, edges_det, det_points;
    //cvtColor(draw_frame, grayed, CV_BGR2GRAY);

   /* blur(grayed, edges_det, Size(5, 5));
    threshold(edges_det, grayed, 150, 255, 0);

    cv::Canny(edges_det, det_points, 50, 200, 3);*/

   /* Point2f src_verticles[4];

    src_verticles[0] = Point(0, 0);
    src_verticles[1] = Point(0, 1024);
    src_verticles[2] = Point(, 0);
    src_verticles[3] = Point(1024,768);



    Point2f dst_verticles[4];
    dst_verticles[0] = Point(0, 0);
    dst_verticles[1] = Point(500, 0);
    dst_verticles[2] = Point(0, 500);
    dst_verticles[3] = Point(500, 500);
   
    auto m = cv::getPerspectiveTransform(src_verticles,
        dst_verticles);


       

    cv::warpPerspective(draw_frame, draw_frame, m, draw_frame.size());*/


//cv::Canny(edges_det, det_points, 50, 200, 3);
    //cv::Mat grayed;
    //Mat source, gray_source, edges_det, dist, cdst, det_points, cpu_dist;
    //source = draw_frame.clone();

    //cvtColor(draw_frame, grayed, CV_BGR2GRAY);

    //blur(grayed, edges_det, Size(5, 5));
    //threshold(edges_det, grayed, 150, 255, 0);

    //cv::Canny(edges_det, det_points, 50, 200, 3);

    //dist = Scalar::all(0);
    //source.copyTo(dist, det_points);
    //Mat gpu_dist = source.clone();

    //cvtColor(det_points, cdst, COLOR_GRAY2BGR);

    //std::vector<Vec4i> cpu_lines;
    //{
    //    cv::HoughLinesP(det_points, cpu_lines, 1, CV_PI / 180, 100, 100, 10);
    //}
    //std::vector<cv::Point> pts;
    //for (size_t index = 0; index < cpu_lines.size(); ++index) {
    //    double dx = cpu_lines[index][2] - cpu_lines[index][0];
    //    double dy = cpu_lines[index][3] - cpu_lines[index][1];

    //    double angle = atan2(dy, dx) * 180.0 / CV_PI;

    //    putText(cdst, std::to_string(angle), Point(cpu_lines[index][0], cpu_lines[index][1]), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, cv::Scalar(0, 0, 0), 2);
    //    //if (abs(angle) > 30 && abs(angle) < 45) {
    //        Vec4i l = cpu_lines[index];

    //        
    //        line(cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 10, LINE_AA);

    //    //}
    //        //pts.push_back(Point(cpu_lines[index][0], cpu_lines[index][1]));
    //       // pts.push_back(Point(cpu_lines[index][2], cpu_lines[index][3]));
    //   // }
    //}
    //for (int c_i = 0; c_i < pts.size(); c_i++)
      //  cv::circle(cdst, pts[c_i], 40.0, cv::Scalar(255, 255, 255));
   
   


  
}

void CLineTracker::OnMain()
{
    
}
