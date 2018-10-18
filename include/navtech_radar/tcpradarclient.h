/*
Copyright 2016 Navtech Radar Limited
This file is part of iasdk which is released under The MIT License (MIT).
See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT 
for full license details.
*/

#pragma once

#include "common.h"

#include "cndpnetworkdatamessage.h"
#include "threadedqueue.h"
#include "tcpsocket.h"
#include "timer.h"

#include <string>
#include <cstdint>
#include <functional>
#include <atomic>
#include <thread>
#include <condition_variable>
#include <mutex>

namespace Navtech
{
	enum class ConnectionState
	{
		Disconnected,
		Connecting,
		Connected
	};
	
	class ConnectionInfo
	{
	public:
		explicit ConnectionInfo(const uint32_t uniqueId, const ConnectionState state) : State(state), UniqueId(uniqueId) {}
		explicit ConnectionInfo(const ConnectionInfo&) = delete;
		ConnectionInfo& operator= (const ConnectionInfo&) = delete;
		ConnectionState State = ConnectionState::Disconnected;
		uint32_t UniqueId = 0;
	};

	typedef std::shared_ptr<ConnectionInfo> ConnectionInfoPtr_t;
	
	class TcpRadarClient
	{
	public:
		explicit TcpRadarClient(const std::string& ipAddress, const uint16_t& port = 6317);
		explicit TcpRadarClient(const TcpRadarClient&) = delete;
		TcpRadarClient& operator= (const TcpRadarClient&) = delete;
		void Start();
		void Stop();
		void Send(const std::vector<uint8_t> data);
		void SetReceiveDataCallback(std::function<void(const CNDPDataMessagePtr_t&)> callback = nullptr);
		Navtech::ConnectionState GetConnectionState();
		
	private:
		std::shared_ptr<ThreadedQueue<CNDPDataMessagePtr_t>> _receiveDataQueue = nullptr;
		std::string _ipAddress = "";
		std::uint16_t _port = 6317;
		TcpSocketPtr_t _socket = nullptr;
		std::shared_ptr<std::thread> _connectThread = nullptr;
		std::shared_ptr<std::thread> _readThread = nullptr;
		TimerPtr_t _connectionCheckTimer = nullptr;
		ConnectionState _connectionState = ConnectionState::Disconnected;
		std::mutex _connectionStateMutex;
		std::condition_variable _connectCondition;
		std::mutex _connectMutex;
		std::atomic_bool _reading;
		std::atomic_bool _running;

		void SetConnectionState(const ConnectionState& state);
		void ConnectionCheckHandler();		
		void ConnectThread();
		void Connect();
		void ReadThread();
		inline bool HandleData();
	};
	
	typedef std::shared_ptr<TcpRadarClient> TcpRadarClientPtr_t;
}
