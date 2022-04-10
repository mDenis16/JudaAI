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
# define M_PI           3.14159265358979323846  /* pi */



void CJuda::InitStepThreads()
{

	const bool KeepSync = false;

	Capture.BindTo(std::bind(&CJuda::OnCapture, this));
	Capture.Object.SetSync(KeepSync);
	Preparing.BindTo(std::bind(&CJuda::OnPreProcessing, this));
	Preparing.Object.SetSync(KeepSync);


	Detecting.BindTo(std::bind(&CJuda::OnDetecting, this));
	Detecting.Object.SetSync(KeepSync);

	Drawing.BindTo(std::bind(&CJuda::OnDrawing, this));
	Drawing.Object.SetSync(KeepSync);

	Showing.BindTo(std::bind(&CJuda::OnShowing, this));
	Showing.Object.SetSync(KeepSync);


}
std::vector<std::string> CJuda::ObjectNamesFromFile(std::string const filename) {
	std::ifstream file(filename);
	std::vector<std::string> FileLines;
	if (!file.is_open()) return FileLines;
	for (std::string line; getline(file, line);) FileLines.push_back(line);
	return FileLines;
}

void CJuda::InitDetector(const std::string names_file, const std::string cfg_file, const std::string weights_file)
{
	DarknetDetector = std::make_shared<Detector>(cfg_file, weights_file);

	CaptureSource.set(cv::CAP_PROP_FRAME_WIDTH, 1024);
	CaptureSource.set(cv::CAP_PROP_FRAME_HEIGHT, 768);
	CaptureSource.open(0);
	CaptureSource.set(cv::CAP_PROP_FRAME_WIDTH, 1024);
	CaptureSource.set(cv::CAP_PROP_FRAME_HEIGHT, 768);
	Names = ObjectNamesFromFile(names_file);

}

void CJuda::OnCapture()
{
	uint64_t LastFrameId = 0;
	CDetectionData data;



	CaptureSource >> CurrentFrame;


	FrameSize = CurrentFrame.size();

	do {
		data = CDetectionData();

		CaptureSource >> data.CaptureFrame;

		FrameSize = data.CaptureFrame.size();

		Measure.CaptureCounter++;

		data.FrameId = LastFrameId++;

		if (data.CaptureFrame.empty() || Exiting) {
			data.Exiting = true;
			data.CaptureFrame = cv::Mat(FrameSize, CV_8UC3);
		}

		Drawing.Object.Send(data);
		Preparing.Object.Send(data);
	} while (!data.Exiting);

	std::cout << "OnCapture Exit thread \n";
}

void CJuda::OnPreProcessing() {
	std::shared_ptr<image_t> DetectionImage;
	CDetectionData Data;
	do {
		Data = Preparing.Object.Receive();

		DetectionImage = DarknetDetector->mat_to_image_resize(Data.CaptureFrame);
		Data.DetectionImage = DetectionImage;
		Detecting.Object.Send(Data);

	} while (!Data.Exiting);
	std::cout << " OnPreProcessing exit \n";
}

void CJuda::OnDetecting() {
	std::shared_ptr<image_t> DetectionImage;
	CDetectionData Data;
	do {
		Data = Detecting.Object.Receive();
		DetectionImage = Data.DetectionImage;

		std::vector<bbox_t> Res;

		if (DetectionImage)
			Res = DarknetDetector->detect_resized(*DetectionImage, FrameSize.width, FrameSize.height, 0.5f, true);
		Measure.DetectionCounter++;

		Data.UpdatedDetection = true;
		Data.Results = Res;
		Drawing.Object.Send(Data);
	} while (!Data.Exiting);
	std::cout << " OnDetecting exit \n";
}

void CJuda::OnDrawing() {
	CDetectionData Data;
	const bool EnableTrackKalman = true;

	do {
		
		auto OldCapturedFrame = Data.CaptureFrame;
		auto OldResults = Data.Results;

		Data = Drawing.Object.Receive();

		if (Data.UpdatedDetection) {
			Data.CaptureFrame = OldCapturedFrame;
			int frame_story = std::max(5, Measure.FpsCapture.load());
			Data.Results = EnableTrackKalman ? Kalman.correct(Data.Results) : DarknetDetector->tracking_id(Data.Results, true, 10, 60);

			Tracker.Update(Data.Results);
		}
		else {
			Data.Results = EnableTrackKalman ? Kalman.predict() : OldResults;
		}

		cv::Mat Frame2Draw = Data.CaptureFrame.clone();

		LineTracker.Update(Frame2Draw);


		if (Measure.FpsDetection >= 0 && Measure.FpsCapture >= 0) {
			std::string fps_str = "FPS detection: " + std::to_string(Measure.FpsDetection) + "   FPS capture: " + std::to_string(Measure.FpsCapture);
			putText(Frame2Draw, fps_str, cv::Point2f(10, 20), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, cv::Scalar(50, 255, 0), 2);
		}

		Data.DrawFrame = Frame2Draw;
		Showing.Object.Send(Data);

	} while (!Data.Exiting);
	std::cout << " Drawing exit \n";
}
void CJuda::OnShowing() {
	CDetectionData Data;
	do {

		Measure.End = std::chrono::steady_clock::now();
		float time_sec = std::chrono::duration<double>(Measure.End - Measure.Start).count();
		if (time_sec >= 1) {
			Measure.FpsDetection = Measure.DetectionCounter.load() / time_sec;
			Measure.FpsCapture = Measure.CaptureCounter.load() / time_sec;
			Measure.Start = Measure.End;
			Measure.DetectionCounter = 0;
			Measure.CaptureCounter = 0;
		}

		Data = Showing.Object.Receive();

		cv::imshow("JudaAI", Data.DrawFrame);

		int key = cv::waitKey(3);   
		if (key == 'p') while (true) if (cv::waitKey(100) == 'p') break;
		
		if (key == 27) { Exiting = true; }

	} while (!Data.Exiting);
}