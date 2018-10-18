/*
Copyright 2016 Navtech Radar Limited
This file is part of iasdk which is released under The MIT License (MIT).
See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT 
for full license details.
*/

#pragma once

#include "common.h"

#include <string>
#include <cstdint>
#include <atomic>
#include <memory>
#include <vector>

#ifdef __BSDSOCKETS__
#include <netinet/in.h>
#elif _WIN32
#include <WinSock2.h>
#endif

namespace Navtech
{
	class TcpSocket
	{
	public:
		explicit TcpSocket(const std::string& destination, const uint16_t& port = 6317);
		~TcpSocket();
		explicit TcpSocket(const TcpSocket&) = delete;
		TcpSocket& operator= (const TcpSocket&) = delete;
		
		const bool IsValid() const;
		
		bool Create(uint32_t receiveTimeout = 0);
		bool Connect();
		bool Close(bool shutdown = false);
		uint32_t Send(const std::vector<uint8_t>& data);
		uint32_t Receive(std::vector<uint8_t>& data, int32_t bytesToRead, bool peek = false);
		void SetSendTimeout(uint32_t sendTimeout);
		
	private:
		std::atomic<int32_t> _sock;
		std::string _destination = "";
		uint16_t _port = 6317;
		sockaddr_in _addr = {0};
	};
	
	typedef std::shared_ptr<TcpSocket> TcpSocketPtr_t;
}
