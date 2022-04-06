#pragma once



#include <atomic>

struct image_t;
struct bbox_t;


struct CDetectionData {
    cv::Mat cap_frame;
    std::shared_ptr<image_t> det_image;
    std::vector<bbox_t> result_vec;
    cv::Mat draw_frame;
    bool new_detection;
    uint64_t frame_id;
    bool exit_flag;
    cv::Mat zed_cloud;
    std::queue<cv::Mat> track_optflow_queue;
    CDetectionData() : new_detection(false), exit_flag(false) {}
};



class CJuda
{
public:
    CJuda() {};

    cv::VideoCapture CaptureSource;

	void InitStepThreads();

    void InitDetector(const std::string names_file, const std::string cfg_file, const std::string weights_file);

    CStepThreadReplaceable<CDetectionData> Capture;
    CStepThreadReplaceable<CDetectionData> Drawing;
    CStepThreadReplaceable<CDetectionData> Detecting;
    CStepThreadReplaceable<CDetectionData> Preparing;
    CStepThreadReplaceable<CDetectionData> Showing;


    void OnCapture();

    void OnPreProcessing();

    void OnDetecting();
    void OnDrawing();

    void OnShowing();
    
    std::shared_ptr<Detector> DarknetDetector;

    cv::Mat CurrentFrame;
    cv::Size FrameSize;
};

