/*
Copyright 2016 Navtech Radar Limited
This file is part of iasdk which is released under The MIT License (MIT).
See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT 
for full license details.
*/

#pragma once

#include "common.h"

#include "threadedclass.h"

#include <functional>
#include <atomic>
#include <queue>
#include <mutex>
#include <condition_variable>

namespace
{
	const uint32_t MAXQUEUEDEPTH = 10000;
}

namespace Navtech
{
	template <class T>
	class ThreadedQueue : public ThreadedClass
	{
	public:
		explicit ThreadedQueue() = default;

		void Enqueue(const T& item, bool notify = true) {
			if (_stopping || _dequeueCallback == nullptr || _thread == nullptr || !_thread->joinable() || _queue.size() > MAXQUEUEDEPTH)
				return;

			std::lock_guard<std::mutex> lock(_queueMutex);
			_queue.push(item);

			if (notify)
				_condition.notify_all();
		}

		void SetDequeueCallback(std::function<void(const T&) > dequeueCallback = nullptr) {
			std::lock_guard<std::mutex> lock(_queueMutex);
			_dequeueCallback = dequeueCallback;
		}

		void Notify() {
			_condition.notify_all();
		}

	protected:
	protected:
		void DoWork() {
			std::unique_lock<std::mutex> lock(_queueMutex);

			if (!_queue.empty()) {
				T item = _queue.front();
				_queue.pop();
				lock.unlock();

				if (_dequeueCallback != nullptr)
					_dequeueCallback(item);
			}
			else
				lock.unlock();

			lock.lock();
			if (_queue.empty() && !_stopRequested) {
				_condition.wait(lock);
			}
			lock.unlock();
		}

		void PreStop(const bool finishWork) {
			_stopping = true;

			if (finishWork) {
				while (!Dequeue())
				{
				}
			}

			std::queue<T> empty;
			std::swap(_queue, empty);
			_condition.notify_all();
		}

		void PostStart() {
			_stopping = false;
		}

	private:
		std::atomic<bool> _stopping;
		std::queue<T> _queue;
		std::mutex _queueMutex;
		std::condition_variable _condition;

		std::function<void(const T&) > _dequeueCallback;

		bool Dequeue() {
			if (!_queue.empty()) {
				T item = _queue.front();
				_queue.pop();

				if (_dequeueCallback != nullptr)
					_dequeueCallback(item);
			}

			return _queue.empty();
		}
	};
}
