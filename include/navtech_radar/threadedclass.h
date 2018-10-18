/*
Copyright 2016 Navtech Radar Limited
This file is part of iasdk which is released under The MIT License (MIT).
See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT 
for full license details.
*/

#pragma once

#include "common.h"

#include <thread>
#include <atomic>
#include <memory>

namespace Navtech
{
	class ThreadedClass
	{
	public:
		explicit ThreadedClass() = default;
		virtual ~ThreadedClass() {}

		ThreadedClass(const ThreadedClass&) = delete;
		ThreadedClass& operator= (const ThreadedClass&) = delete;

		virtual void Start();
		virtual void Stop(const bool finishWork = false);
		virtual void Join(void);

	protected:
		virtual void DoWork() = 0;
		virtual void PreStop(const bool finishWork = false);
		virtual void PostStart();

		std::shared_ptr<std::thread> _thread = nullptr;
		std::atomic<bool> _stopRequested;

	private:
		void ThreadMethod();
	};
}
