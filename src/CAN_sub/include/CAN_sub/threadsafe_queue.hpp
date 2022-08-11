#pragma once

#include <queue>
#include <mutex>

template <class T> class threadsafe_queue {
    public:
        //constructor
        threadsafe_queue() = default;

        T front();

        int size();

        bool empty();

        void push(T element);

        void pop();

        /*
            @brief -> retrieve front value and pop element
        */
        T dequeue();

        //void swap(std::queue<T> swap_queue);

        ~threadsafe_queue();
    private:

        std::queue<T> queue;
        std::mutex m;

};