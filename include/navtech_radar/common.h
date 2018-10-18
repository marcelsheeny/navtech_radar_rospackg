/*
Copyright 2016 Navtech Radar Limited
This file is part of iasdk which is released under The MIT License (MIT).
See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT 
for full license details.
*/

#pragma once

#if defined(__linux__) || defined(__FreeBSD__)
#define __BSDSOCKETS__
#endif

#include <chrono>
#include <ctime>
#include <sstream>
#include <iostream>
#include <iomanip>

namespace Navtech
{
	class Helpers
	{
	public:
		static uint64_t Now() {
			return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
		}
		
		static void Log(const std::string& message)
		{
			auto now = std::chrono::system_clock::now();
			auto in_time_t = std::chrono::system_clock::to_time_t(now);

			auto nowSeconds = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
			auto nowMilliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
			auto millisecondsRemaining = nowMilliseconds - (nowSeconds * 1000);

			constexpr auto bufsize = 20;
			constexpr auto logDateBufSize = 30;
			char buf[bufsize];
			char logDateBuf[logDateBufSize];

			std::strftime(buf,bufsize,"%Y%m%d", std::gmtime(&in_time_t));
			std::strftime(logDateBuf,logDateBufSize,"%Y-%m-%dT%H:%M:%S", std::gmtime(&in_time_t));
			std::stringstream ss;
			ss << std::string(logDateBuf) << "." << std::setw(3) << std::setfill('0') << millisecondsRemaining << "Z";
			std::cout << ss.str() << " - " << message << std::endl;
		}
	};
}
