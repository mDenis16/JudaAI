
#include <iostream>
#include "yolo_wrapper.h"

#include <opencv2/opencv.hpp>            // C++
#include <opencv2/core/version.hpp>
#include <opencv2/videoio/videoio.hpp>
#define OPENCV_VERSION CVAUX_STR(CV_VERSION_MAJOR)"" CVAUX_STR(CV_VERSION_MINOR)"" CVAUX_STR(CV_VERSION_REVISION)
#pragma comment(lib, "opencv_world" OPENCV_VERSION ".lib")


#include <opencv2/opencv.hpp>            // C++
#include <opencv2/core/version.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <thread>

#include "CStepThreadReplaceable.h"
#include "CReplaceableObject.h"
#include "yolo_wrapper.h"
#include "CJuda.h"

int main()
{
    std::string  names_file = "data/coco.names";
    std::string  cfg_file = "yolov3.cfg";
    std::string  weights_file = "yolov3.weights";


    CJuda JudaAI;

    JudaAI.InitDetector(names_file, cfg_file, weights_file);

    JudaAI.InitStepThreads();

    int n = 0;
    std::cin >> n;
}

