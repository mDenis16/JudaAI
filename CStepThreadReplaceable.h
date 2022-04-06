#pragma once   
#include <thread>
template<typename T>
class CReplaceableObject;

template<typename T>
class CStepThreadReplaceable {
public:

	CStepThreadReplaceable() {};

	void BindTo(std::function<void()> func) { Thread = std::thread(func); }

	std::thread Thread;
	CReplaceableObject<T> Object;
};