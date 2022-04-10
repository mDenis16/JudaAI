#pragma once



#include <atomic>


struct image_t;
struct bbox_t;



struct CDetectionData {
    cv::Mat CaptureFrame;
    std::shared_ptr<image_t> DetectionImage;
    cv::Mat Originalframe;
    std::vector<bbox_t> Results;
    cv::Mat DrawFrame;
    bool UpdatedDetection;
    uint64_t FrameId;
    bool Exiting;
    CDetectionData() : UpdatedDetection(false), Exiting(false) {}
};



class CJuda
{
public:
    CJuda() {};

    struct SMeasure {
        std::chrono::steady_clock::time_point Start, End;
        std::atomic<int> FpsCapture{};
        std::atomic<int> FpsDetection{};
        std::atomic<int> CaptureCounter{};
        std::atomic<int> DetectionCounter{};
    };

    SMeasure Measure;

    cv::VideoCapture CaptureSource;

    CTracker Tracker;

    CLineTracker LineTracker;

    track_kalman_t Kalman;

	void InitStepThreads();

    void InitDetector(const std::string names_file, const std::string cfg_file, const std::string weights_file);

    std::vector<std::string> ObjectNamesFromFile(std::string const filename);

    CStepThreadReplaceable<CDetectionData> Capture;
    CStepThreadReplaceable<CDetectionData> Drawing;
    CStepThreadReplaceable<CDetectionData> Detecting;
    CStepThreadReplaceable<CDetectionData> Preparing;
    CStepThreadReplaceable<CDetectionData> Showing;

    std::vector<cv::String> Names;

    void OnCapture();

    void OnPreProcessing();

    void OnDetecting();
    void OnDrawing();

    void OnShowing();
    
    std::shared_ptr<Detector> DarknetDetector;

    cv::Mat CurrentFrame;
    cv::Size FrameSize;

    std::atomic<bool> Exiting{};
};

