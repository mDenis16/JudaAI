#pragma once
#include <unordered_map>

class CTracker
{
public:

	void Update(std::vector<bbox_t>& box_list);

	std::deque<bbox_t> GetHistory(int id);

	std::unordered_map<int, std::deque<bbox_t>> TrackerMap;

};

