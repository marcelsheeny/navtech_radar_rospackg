/*
Copyright 2016 Navtech Radar Limited
This file is part of iasdk which is released under The MIT License (MIT).
See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT 
for full license details.
*/

#include "tcpradarclient.h"

#include <functional>
#include <algorithm>

namespace
{
	const uint16_t CONNECTION_CHECK_TIMEOUT = 5000;
	const uint16_t READ_TIMEOUT = 6;
	const uint16_t SEND_TIMEOUT = 1;
}

Navtech::TcpRadarClient::TcpRadarClient(const std::string& ipAddress, const uint16_t& port)
: _receiveDataQueue(std::make_shared<ThreadedQueue<CNDPDataMessagePtr_t>>()),
	_ipAddress(ipAddress),
	_port(port),
	_socket(std::make_shared<TcpSocket>(_ipAddress, _port)),
	_connectThread(nullptr),
	_readThread(nullptr),
	_connectionCheckTimer(std::make_shared<Timer>(CONNECTION_CHECK_TIMEOUT)),
	_connectionState(ConnectionState::Disconnected),
	_reading(false),
	_running(false)
{

}

void Navtech::TcpRadarClient::SetReceiveDataCallback(std::function<void(const CNDPDataMessagePtr_t&)> callback)
{
	_receiveDataQueue->SetDequeueCallback(callback);
}

Navtech::ConnectionState Navtech::TcpRadarClient::GetConnectionState()
{
	if (!_running) return ConnectionState::Disconnected;
	
	_connectionStateMutex.lock();
	auto state = _connectionState;
	_connectionStateMutex.unlock();
	return state;
}

void Navtech::TcpRadarClient::Start()
{
	if (_running) return;

	_receiveDataQueue->Start();

	_running = true;
	_connectThread = std::make_shared<std::thread>(std::bind(&TcpRadarClient::ConnectThread, this));
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	_connectCondition.notify_all();
	_connectionCheckTimer->SetCallback(std::bind(&TcpRadarClient::ConnectionCheckHandler, this));
	_connectionCheckTimer->Enable(true);
}

void Navtech::TcpRadarClient::Stop()
{
	if (!_running) return;

	_receiveDataQueue->Stop();

	_connectionCheckTimer->Enable(false);
	_connectionCheckTimer->SetCallback(nullptr);

	_running = false;
	_socket->Close(true);
	_connectCondition.notify_all();
	if (_connectThread != nullptr) {
		_connectThread->join();
		_connectThread = nullptr;
	}

	_reading = false;
	if (_readThread != nullptr) {
		_readThread->join();
		_readThread = nullptr;
	}
}

void Navtech::TcpRadarClient::SetConnectionState(const Navtech::ConnectionState& state)
{
	if (!_running) return;
	std::lock_guard<std::mutex> lock(_connectionStateMutex);
	if (_connectionState == state) {
		return;
	}
	_connectionState = state;

	std::string stateString;
	switch (state)
	{
	case Navtech::ConnectionState::Connected:
		stateString = "Connected";
		break;
	case Navtech::ConnectionState::Connecting:
		stateString = "Connecting";
		break;
	case Navtech::ConnectionState::Disconnected:
		stateString = "Disconnected";
		break;
	}

	Helpers::Log("TcpRadarClient - Connection State Changed [" + stateString + "] for [" + _ipAddress + ":" + std::to_string(_port) + "]");
}

void Navtech::TcpRadarClient::ConnectThread()
{
	while (_running) {
		std::unique_lock<std::mutex> lock(_connectMutex);
		_connectCondition.wait(lock);

		if (!_running)
			break;

		Connect();
	}

	Helpers::Log("TcpRadarClient - Connect Thread Finished");
}

void Navtech::TcpRadarClient::ConnectionCheckHandler()
{
	if (GetConnectionState() != ConnectionState::Disconnected) return;
	_connectCondition.notify_one();
}

void Navtech::TcpRadarClient::Connect()
{
	if (GetConnectionState() == ConnectionState::Connected) return;

	SetConnectionState(ConnectionState::Connecting);

	_socket->Close();
	_socket->Create(READ_TIMEOUT);
	_socket->SetSendTimeout(SEND_TIMEOUT);

	if (_socket->Connect()) {
		if (_readThread != nullptr) {
			_readThread->join();
			_readThread = nullptr;
		}
		SetConnectionState(ConnectionState::Connected);
		_reading = true;
		_readThread = std::make_shared<std::thread>(std::bind(&TcpRadarClient::ReadThread, this));
	}
	else {
		SetConnectionState(ConnectionState::Disconnected);
	}
}

void Navtech::TcpRadarClient::ReadThread()
{
	Helpers::Log("TcpRadarClient - Read Thread Started");
	
	while (_reading && _running) {
		std::vector<uint8_t> signature;
		int32_t bytesRead = _socket->Receive(signature, NDMSIGNATURELENGTH, true);

		if (bytesRead == 0)
			continue;

		if (bytesRead < 0 || !_reading || !_running) {
			SetConnectionState(ConnectionState::Disconnected);
			break;
		}

		auto result = true;
		for (auto i = 0u; i < NDMSIGNATURELENGTH; i++) {
			result &= NDMSIGNATUREBYTES[i] == signature[i];
		}

		if (!result) {
			bytesRead = _socket->Receive(signature, 1);
			if (bytesRead <= 0 || !_reading || !_running) {
				SetConnectionState(ConnectionState::Disconnected);
				Helpers::Log("TcpRadarClient - Read Failed Signature");
				break;
			}
			continue;
		}

		if (!HandleData() || !_reading || !_running) {
			SetConnectionState(ConnectionState::Disconnected);
			Helpers::Log("TcpRadarClient - Failed HandleData");
			break;
		}
	}
	_reading = false;
	Helpers::Log("TcpRadarClient - Read Thread Exited");
}

void Navtech::TcpRadarClient::Send(const std::vector<uint8_t> data)
{
	if (GetConnectionState() != ConnectionState::Connected) return;

	if (_socket->Send(data) != 0) {
		SetConnectionState(ConnectionState::Disconnected);
		Helpers::Log("TcpRadarClient - Send Failed");
	}
}

bool Navtech::TcpRadarClient::HandleData()
{
	CNDPNetworkDataHeaderStruct messageHeader;

	std::memset(&messageHeader, 0, messageHeader.HeaderLength());
	std::vector<uint8_t> data;
	int32_t bytesRead = _socket->Receive(data, messageHeader.HeaderLength());

	if (bytesRead <= 0 || !_reading || !_running) {
		SetConnectionState(ConnectionState::Disconnected);
		return false;
	}
	std::memcpy(&messageHeader, &data[0], bytesRead);
	if (messageHeader.HeaderIsValid() && messageHeader.PayloadLength() != 0) {
		std::vector<uint8_t> payloadData;
		int32_t bytesTransferred = _socket->Receive(payloadData, messageHeader.PayloadLength());
		if (bytesTransferred <= 0 || !_reading || !_running) {
			SetConnectionState(ConnectionState::Disconnected);
			return false;
		}
		bytesTransferred++;

		CNDPDataMessagePtr_t streamData = std::make_shared<NetworkDataMessage>(messageHeader, &payloadData[0], messageHeader.PayloadLength());
		_receiveDataQueue->Enqueue(streamData);
	}
	else if (messageHeader.HeaderIsValid()) {
		CNDPDataMessagePtr_t streamData = std::make_shared<NetworkDataMessage>(messageHeader);
		_receiveDataQueue->Enqueue(streamData);
	}

	return true;
}
