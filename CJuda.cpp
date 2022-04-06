
#include <opencv2/opencv.hpp>            // C++
#include <opencv2/core/version.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <thread>

#include "CStepThreadReplaceable.h"
#include "CReplaceableObject.h"
#include "yolo_wrapper.h"
#include "CJuda.h"
std::chrono::steady_clock::time_point steady_start, steady_end;
std::atomic<int> fps_cap_counter(0), fps_det_counter(0);
std::atomic<int> current_fps_cap(0), current_fps_det(0);
std::atomic<bool> exit_flag(false);

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

void CJuda::InitDetector(const std::string names_file, const std::string cfg_file, const std::string weights_file)
{
    DarknetDetector = std::make_shared<Detector>(cfg_file, weights_file);

    CaptureSource.open(0);

}

void CJuda::OnCapture()
{
    uint64_t frame_id = 0;
    CDetectionData data;
    CaptureSource >> CurrentFrame;

    FrameSize = CurrentFrame.size();

    do {
        data = CDetectionData();

        CaptureSource >> data.cap_frame;
       
        
        data.frame_id = frame_id++;
        if (data.cap_frame.empty() ) {
            std::cout << " exit_flag: detection_data.cap_frame.size = " << data.cap_frame.size() << std::endl;
            data.exit_flag = true;
            cv::Size const frame_size = data.cap_frame.size();
            data.cap_frame = cv::Mat(frame_size, CV_8UC3);
        }

        //if (!detection_sync) {
        //    cap2draw.send(detection_data);       // skip detection
        //}
        Preparing.Object.Send(data);
    } while (!data.exit_flag);
    std::cout << "OnCapture Exit thread \n";
}

void CJuda::OnPreProcessing() {
    std::shared_ptr<image_t> det_image;
    CDetectionData detection_data;
    do {
        detection_data = Preparing.Object.Receive();

        det_image = DarknetDetector->mat_to_image_resize(detection_data.cap_frame);
        detection_data.det_image = det_image;
        Detecting.Object.Send(detection_data);    // detection

    } while (!detection_data.exit_flag);
    std::cout << " OnPreProcessing exit \n";
}

void CJuda::OnDetecting(){
    std::shared_ptr<image_t> det_image;
    CDetectionData detection_data;
    do {
        detection_data = Preparing.Object.Receive();
        det_image = detection_data.det_image;
        std::vector<bbox_t> result_vec;

        if (det_image)
            result_vec = DarknetDetector->detect_resized(*det_image, FrameSize.width, FrameSize.height, 0.5f, true);  // true
        //fps_det_counter++;
        //std::this_thread::sleep_for(std::chrono::milliseconds(150));

        detection_data.new_detection = true;
        detection_data.result_vec = result_vec;
        Drawing.Object.Send(detection_data);
    } while (!detection_data.exit_flag);
    std::cout << " OnDetecting exit \n";
}

void CJuda::OnDrawing() {
    std::queue<cv::Mat> track_optflow_queue;
    CDetectionData detection_data;
    do {

        // for Video-file
        //if (detection_sync) {
        //    detection_data = detect2draw.receive();
        //}
        //// for Video-camera
        //else
        {
            // get new Detection result if present
            if (Drawing.Object.IsObjectPresent()) {
                cv::Mat old_cap_frame = detection_data.cap_frame;   // use old captured frame
                detection_data = Drawing.Object.Receive();
                if (!old_cap_frame.empty()) detection_data.cap_frame = old_cap_frame;
            }
            // get new Captured frame
            else {
                std::vector<bbox_t> old_result_vec = detection_data.result_vec; // use old detections
                detection_data = Drawing.Object.Receive();
                detection_data.result_vec = old_result_vec;
            }
        }

        cv::Mat cap_frame = detection_data.cap_frame;
        cv::Mat draw_frame = detection_data.cap_frame.clone();
        std::vector<bbox_t> result_vec = detection_data.result_vec;

#ifdef TRACK_OPTFLOW
        if (detection_data.new_detection) {
            tracker_flow.update_tracking_flow(detection_data.cap_frame, detection_data.result_vec);
            while (track_optflow_queue.size() > 0) {
                draw_frame = track_optflow_queue.back();
                result_vec = tracker_flow.tracking_flow(track_optflow_queue.front(), false);
                track_optflow_queue.pop();
            }
        }
        else {
            track_optflow_queue.push(cap_frame);
            result_vec = tracker_flow.tracking_flow(cap_frame, false);
        }
        detection_data.new_detection = true;    // to correct kalman filter
#endif //TRACK_OPTFLOW

                        // track ID by using kalman filter
        //if (use_kalman_filter) {
        //    if (detection_data.new_detection) {
        //        result_vec = track_kalman.correct(result_vec);
        //    }
        //    else {
        //        result_vec = track_kalman.predict();
        //    }
        //}
        //// track ID by using custom function
        //else {
        //    int frame_story = std::max(5, current_fps_cap.load());
        //    result_vec = detector.tracking_id(result_vec, true, frame_story, 40);
        //}

        //if (use_zed_camera && !detection_data.zed_cloud.empty()) {
        //    result_vec = get_3d_coordinates(result_vec, detection_data.zed_cloud);
        //}

        ////small_preview.set(draw_frame, result_vec);
        ////large_preview.set(draw_frame, result_vec);
        //draw_boxes(draw_frame, result_vec, obj_names, current_fps_det, current_fps_cap);
        //show_console_result(result_vec, obj_names, detection_data.frame_id);
        //large_preview.draw(draw_frame);
        //small_preview.draw(draw_frame, true);

        detection_data.result_vec = result_vec;
        detection_data.draw_frame = draw_frame;
        Showing.Object.Send(detection_data);
       
    } while (!detection_data.exit_flag);
    std::cout << " Drawing exit \n";
}
void CJuda::OnShowing() {
    CDetectionData detection_data;
    do {

        steady_end = std::chrono::steady_clock::now();
        float time_sec = std::chrono::duration<double>(steady_end - steady_start).count();
        if (time_sec >= 1) {
            current_fps_det = fps_det_counter.load() / time_sec;
            current_fps_cap = fps_cap_counter.load() / time_sec;
            steady_start = steady_end;
            fps_det_counter = 0;
            fps_cap_counter = 0;
        }

        detection_data = Showing.Object.Receive();
        cv::Mat draw_frame = detection_data.draw_frame;

        //if (extrapolate_flag) {
        //    cv::putText(draw_frame, "extrapolate", cv::Point2f(10, 40), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.0, cv::Scalar(50, 50, 0), 2);
        //}

        cv::imshow("window name", draw_frame);
        int key = cv::waitKey(3);    // 3 or 16ms
       /* if (key == 'f') show_small_boxes = !show_small_boxes;*/
        if (key == 'p') while (true) if (cv::waitKey(100) == 'p') break;
        //if (key == 'e') extrapolate_flag = !extrapolate_flag;
        if (key == 27) { exit_flag = true; }

        //std::cout << " current_fps_det = " << current_fps_det << ", current_fps_cap = " << current_fps_cap << std::endl;
    } while (!detection_data.exit_flag);
}