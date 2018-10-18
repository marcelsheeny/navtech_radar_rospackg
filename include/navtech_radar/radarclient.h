/*
Copyright 2016 Navtech Radar Limited
This file is part of iasdk which is released under The MIT License (MIT).
See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT 
for full license details.
*/

#pragma once

#include "common.h"

#include "/home/sapratshi/catkin_ws/src/navtech_radar/include/navtech_radar/tcpradarclient.h"

#include <cstdint>
#include <string>
#include <mutex>
#include <atomic>
#include <vector>

namespace Navtech
{
	class FFTData
	{
	public:
		float Angle = 0.0f;
		uint16_t Azimuth = 0;
		uint16_t SweepCounter = 0;
		uint32_t NTPSeconds = 0;
		uint32_t NTPSplitSeconds = 0;
		std::vector<uint8_t> Data;
	};
	typedef std::shared_ptr<FFTData> FFTDataPtr_t;
	
	class NavigationData
	{
	public:
		float Angle = 0.0f;
		uint16_t Azimuth = 0;
		uint32_t NTPSeconds = 0;
		uint32_t NTPSplitSeconds = 0;
		std::vector<std::tuple<float, uint16_t>> Peaks;
	};
	typedef std::shared_ptr<NavigationData> NavigationDataPtr_t;

	class ConfigurationData
	{
	public:
		uint16_t AzimuthSamples = 0;
		uint16_t EncoderSize = 0;
		uint16_t BinSize = 0;
		uint16_t RangeInBins = 0;
		uint16_t ExpectedRotationRate = 0;
	};
	typedef std::shared_ptr<ConfigurationData> ConfigurationDataPtr_t;
	
	class RadarClient
	{
	public:
		explicit RadarClient(const std::string& radarAddress, const uint16_t& port = 6317);
		explicit RadarClient(const RadarClient&) = delete;
		RadarClient& operator= (const RadarClient&) = delete;

		void Start();
		void Stop();
		
		void UpdateContourMap(const std::vector<uint8_t>& contourData);
		void StartFFTData();
		void StopFFTData();
		void StartNavigationData();
		void StopNavigationData();
		void SetNavigationThreshold(const uint16_t& threshold);
		void SetNavigationGainAndOffset(const float& gain, const float& offset);
		
		void SetFFTDataCallback(std::function<void(const FFTDataPtr_t&)> callback = nullptr);
		void SetNavigationDataCallback(std::function<void(const NavigationDataPtr_t&)> callback = nullptr);
		void SetConfigurationDataCallback(std::function<void(const ConfigurationDataPtr_t&)> callback = nullptr);
		
	private:
		TcpRadarClientPtr_t _radarClient = nullptr;
		std::atomic_bool _running;
		std::atomic_bool _sendRadarData;
		std::mutex _callbackMutex;
		std::function<void(const FFTDataPtr_t&)> _fftDataCallback = nullptr;
		std::function<void(const NavigationDataPtr_t&)> _navigationDataCallback = nullptr;
		std::function<void(const ConfigurationDataPtr_t&)> _configurationDataCallback = nullptr;
		
		uint16_t _encoderSize = 0;
		uint16_t _binSize = 0;
		uint16_t _rangeinbins = 0;
		uint16_t _azimuthSamples = 0;
		uint16_t _expectedRotationRate = 0;
		
		void HandleData(const CNDPDataMessagePtr_t& message);
		void HandleConfigurationMessage(const CNDPDataMessagePtr_t& configMessage);
		void HandleFFTDataMessage(const CNDPDataMessagePtr_t& fftDataMessage);		
		void HandleNavigationDataMessage(const CNDPDataMessagePtr_t& navigationMessage);
		void SendSimpleNetworkMessage(const CNDPNetworkDataMessageType& type);
	};
	
	typedef std::shared_ptr<RadarClient> RadarClientPtr_t;
}
