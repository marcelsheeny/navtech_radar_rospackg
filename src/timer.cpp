/*
Copyright 2016 Navtech Radar Limited
This file is part of iasdk which is released under The MIT License (MIT).
See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT 
for full license details.
*/

#include "timer.h"

#include <chrono>
#include <thread>
#include <iostream>

Navtech::Timer::Timer() : Navtech::Timer::Timer(1000)
{}

Navtech::Timer::Timer(uint32_t timeout)
: _timeoutms(timeout),
	_isEnabled(false)
{}

void Navtech::Timer::SetCallback(std::function<void()> callback)
{
	std::lock_guard<std::mutex> lock(_callbackMutex);
	_callback = callback;
}

uint32_t Navtech::Timer::Timeoutms() const
{
	return _timeoutms;
}

void Navtech::Timer::Timeoutms(const uint32_t newTimeout)
{
	_timeoutms = newTimeout;
}

void Navtech::Timer::DoWork()
{
	auto now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();

	while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() < now + _timeoutms) {
		std::this_thread::sleep_for(std::chrono::microseconds(10000));

		if (std::abs(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() - now) > 15000) {
			now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
		}

		if (_stopRequested)
			break;
	}

	std::lock_guard<std::mutex> lock(_callbackMutex);
	if (_isEnabled && _callback != nullptr)
		_callback();
}

void Navtech::Timer::Enable(const bool enable)
{
	if (_isEnabled != enable) {
		_isEnabled = enable;

		if (_isEnabled)
			Start();
		else
			Stop();
	}
}
