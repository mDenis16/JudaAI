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


void CTracker::Update(std::vector<bbox_t>& box_list)
{
	for (const auto& box : box_list)
	{
		if (!TrackerMap[box.track_id].empty() && TrackerMap[box.track_id].back().obj_id != box.obj_id)
			TrackerMap[box.track_id].clear();


		TrackerMap[box.track_id].push_back(box);
	    
		if (TrackerMap[box.track_id].size() > 24)
			TrackerMap[box.track_id].pop_front();
	}
}

std::deque<bbox_t> CTracker::GetHistory(int id) {
	std::deque<bbox_t> history_bbox;

	auto& idx = TrackerMap[id];

	return idx;
}


