#pragma once

#include <queue>
#include <mutex>

using namespace std;

template<class T>
class threadsafe_queue {

public:

	// constructor
	threadsafe_queue() = default;

	T front() {
		lock_guard <mutex> lg(m);

		T front_value = 0.0;

		if (!queue.empty()) {
			front_value = queue.front();
		}

		return front_value;
	}

	int size() {
		lock_guard <mutex> lg(m);

		return queue.size();
	}

	bool empty() {
		lock_guard <mutex> lg(m);

		return queue.empty();
	}

	void push(T element) {
		lock_guard <mutex> lg(m);

		queue.push(element);
	}

	void pop() {
		lock_guard <mutex> lg(m);

		if (!queue.empty()) {
			queue.pop();
		}
	}

	/*
		@brief -> retrieve front value and pop element
	*/
	T dequeue() {
		lock_guard <mutex> lg(m);

		T front_value = 0.0;

		if (!queue.empty()) {
			front_value = queue.front();
			queue.pop();
		}

		return front_value;
	}

	void swap(std::queue <T> swap_queue) {
		lock_guard <mutex> lg(m);

		if (!queue.empty()) {
			queue.pop();
		}

	}

	// ~threadsafe_queue();

private:

	std::queue <T> queue;
	std::mutex m;

};

// TODO: you can use object/struct return type to return an object with boolean and front_value for tryDequeue
