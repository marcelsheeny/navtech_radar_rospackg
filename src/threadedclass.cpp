/*
Copyright 2016 Navtech Radar Limited
This file is part of iasdk which is released under The MIT License (MIT).
See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT 
for full license details.
*/

#include "threadedclass.h"

void Navtech::ThreadedClass::ThreadMethod()
{
	while (!_stopRequested) {
		DoWork();
	}
}

void Navtech::ThreadedClass::PostStart() {}

void Navtech::ThreadedClass::PreStop(const bool finishWork) {}

void Navtech::ThreadedClass::Start()
{
	if (_thread != nullptr && _thread->joinable())
		return;

	if (_thread == nullptr) {
		_stopRequested = false;
		_thread = std::make_shared<std::thread>(std::thread(&ThreadedClass::ThreadMethod, this));
	}

	std::this_thread::yield();
	PostStart();
}

void Navtech::ThreadedClass::Stop(const bool finishWork)
{
	if (_thread == nullptr)
		return;

	if (_stopRequested)
		return;

	_stopRequested = true;
	PreStop(finishWork);
	Join();
	_thread = nullptr;
}

void Navtech::ThreadedClass::Join(void)
{
	if (_thread == nullptr || !_thread->joinable()) return;

	_thread->join();
}
