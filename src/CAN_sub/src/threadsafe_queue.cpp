#include "CAN_sub/threadsafe_queue.hpp"

using namespace std;

template <class T> T threadsafe_queue<T>::front() {
    lock_guard<mutex> lg(m);

    T front_value = 0.0;

    if (!queue.empty()) {
        front_value = queue.front();
    }

    return front_value;
}

template <class T> void threadsafe_queue<T>::push(T element) {
    lock_guard<mutex> lg(m);

    queue.push(element);
    
}

template <class T> void threadsafe_queue<T>::pop() {
    lock_guard<mutex> lg(m);

    if (!queue.empty()) {
        queue.pop();
    }
    
}

template <class T> T threadsafe_queue<T>::dequeue() {
    lock_guard<mutex> lg(m);

    T front_value = 0.0;

    if (!queue.empty()) {
        front_value = queue.front();
        queue.pop();
    }

    return front_value;
}

template <class T> int threadsafe_queue<T>::size() {
    lock_guard<mutex> lg(m);

    return queue.size();
}

template <class T> bool threadsafe_queue<T>::empty() {
    lock_guard<mutex> lg(m);

    return queue.empty();
}


// template <class T> void threadsafe_queue<T>::swap(std::queue<T> swap_queue) {
//     lock_guard<mutex> lg(m);



//     if (!queue.empty()) {
//         queue.pop();
//     }
    
// }
