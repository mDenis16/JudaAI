#pragma once
#include <atomic>

template<typename T>
class CReplaceableObject {
    bool sync = false;
    std::atomic<T*> a_ptr;
public:

    CReplaceableObject() {};

    void Send(T const& _obj) {
        T* new_ptr = new T;
        *new_ptr = _obj;
        if (sync) {
            while (a_ptr.load()) std::this_thread::sleep_for(std::chrono::milliseconds(3));
        }
        std::unique_ptr<T> old_ptr(a_ptr.exchange(new_ptr));
    }

    T Receive() {
        std::unique_ptr<T> ptr;
        do {
            while (!a_ptr.load()) std::this_thread::sleep_for(std::chrono::milliseconds(3));
            ptr.reset(a_ptr.exchange(NULL));
        } while (!ptr);
        T obj = *ptr;
        return obj;
    }

    void SetSync(bool value) {
        sync = value;
    }

    bool IsObjectPresent() {
        return (a_ptr.load() != NULL);
    }

    CReplaceableObject(bool _sync) : sync(_sync), a_ptr(NULL)
    {}
};