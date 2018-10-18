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
#include <thread>
#include <mutex>
#include <cstdint>
#include <atomic>

namespace Navtech
{
	class Timer : public ThreadedClass
	{
	public:
		std::function<void()> _callback = nullptr;
		std::mutex _callbackMutex;
		std::atomic<uint32_t> _timeoutms;
		std::atomic_bool _isEnabled;

	protected:
		void DoWork();

	public:
		explicit Timer();
		explicit Timer(uint32_t timeout);

		Timer(const Timer&) = delete;
		Timer& operator= (const Timer&) = delete;

		void SetCallback(std::function<void()> callback = nullptr);
		void Enable(const bool enable);
		uint32_t Timeoutms() const;
		void Timeoutms(const uint32_t newTimeout);
	};

	typedef std::shared_ptr<Timer> TimerPtr_t;
}
