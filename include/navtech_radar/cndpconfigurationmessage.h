/*
Copyright 2016 Navtech Radar Limited
This file is part of iasdk which is released under The MIT License (MIT).
See file LICENSE.txt in project root or go to https://opensource.org/licenses/MIT 
for full license details.
*/

#pragma once

#include "common.h"

#include "cndpnetworkdatamessage.h"

#include <memory>
#include <vector>
#include <cstring>
#include <cstdint>

namespace Navtech
{
	namespace
	{
		const uint8_t	TCPNDMAZIMUTHSAMPLESOFFSET = 0;
		const uint8_t	TCPNDMAZIMUTHSAMPLESLENGTH = sizeof(uint16_t);
		const uint8_t	TCPNDMBINSIZEOFFSET = TCPNDMAZIMUTHSAMPLESOFFSET + TCPNDMAZIMUTHSAMPLESLENGTH;
		const uint8_t	TCPNDMBINSIZELENGTH = sizeof(uint16_t);
		const uint8_t	TCPNDMRANGEINBINSOFFSET = TCPNDMBINSIZEOFFSET + TCPNDMBINSIZELENGTH;
		const uint8_t	TCPNDMRANGEINBINSLENGTH = sizeof(uint16_t);
		const uint8_t	TCPNDMENCODERSIZEOFFSET = TCPNDMRANGEINBINSOFFSET + TCPNDMRANGEINBINSLENGTH;
		const uint8_t	TCPNDMENCODERSIZELENGTH = sizeof(uint16_t);
		const uint8_t	TCPNDMROTATIONSPEEDOFFSET = TCPNDMENCODERSIZEOFFSET + TCPNDMENCODERSIZELENGTH;
		const uint8_t	TCPNDMROTATIONSPEEDLENGTH = sizeof(uint16_t);
		const uint8_t	TCPNDMPACKETRATEOFFSET = TCPNDMROTATIONSPEEDOFFSET + TCPNDMROTATIONSPEEDLENGTH;
		const uint8_t	TCPNDMPACKETRATELENGTH = sizeof(uint16_t);
		const uint8_t	TCPNDMRANGEGAINOFFSET = TCPNDMPACKETRATEOFFSET + TCPNDMPACKETRATELENGTH;
		const uint8_t	TCPNDMRANGEGAINLENGTH = sizeof(uint32_t);
		const uint8_t	TCPNDMRANGEOFFSETOFFSET = TCPNDMRANGEGAINOFFSET + TCPNDMRANGEGAINLENGTH;
		const uint8_t	TCPNDMRANGEOFFSETLENGTH = sizeof(uint32_t);

		const uint8_t	TCPNDMCONFIGURATIONHEADERLENGTH = TCPNDMAZIMUTHSAMPLESLENGTH + TCPNDMBINSIZELENGTH + TCPNDMRANGEINBINSLENGTH
			+ TCPNDMENCODERSIZELENGTH + TCPNDMROTATIONSPEEDLENGTH + TCPNDMPACKETRATELENGTH
			+ TCPNDMRANGEGAINLENGTH + TCPNDMRANGEOFFSETLENGTH;
	}

#pragma pack(push)
#pragma pack(1)
	typedef struct CNDPConfigurationHeader {
		CNDPNetworkDataHeaderStruct header;
		uint16_t azimuthsamples;
		uint16_t binsize;
		uint16_t rangeinbins;
		uint16_t encodersize;
		uint16_t rotationspeed;
		uint16_t packetrate;
		uint32_t rangegain;
		uint32_t rangeoffset;

		void Init() {
			header.Init();
			header.messageid = static_cast<uint8_t>(CNDPNetworkDataMessageType::Configuration);
			azimuthsamples = 0;
			binsize = 0;
			rangeinbins = 0;
			encodersize = 0;
			rotationspeed = 0;
			packetrate = 0;
			rangegain = 1;
			rangeoffset = 0;
		}
		constexpr const std::size_t HeaderLength() const {
			return TCPNDMCONFIGURATIONHEADERLENGTH;
		}
		void SetProtocolBufferLength(std::size_t  length) {
			header.SetPayloadLength(TCPNDMCONFIGURATIONHEADERLENGTH + length);
		}
		void SetAzimuthSamples(std::uint16_t azimuthSamples) {
			azimuthsamples = htons(azimuthSamples);
		}
		void SetBinSize(std::uint16_t binSize) {
			binsize = htons(binSize);
		}
		void SetRangeInBins(std::uint16_t rangeInBins) {
			rangeinbins = htons(rangeInBins);
		}
		void SetEncoderSize(std::uint16_t encoderSize) {
			encodersize = htons(encoderSize);
		}
		void SetRotationSpeed(std::uint16_t rotationSpeed) {
			rotationspeed = htons(rotationSpeed);
		}
		void SetPacketRate(std::uint16_t packetRate) {
			packetrate = htons(packetRate);
		}
		void SetRangeGain(float rangeGain) {
			union v {
				float f;
				uint32_t i;
			};
			v testValue;
			testValue.f = rangeGain;
			rangegain = htonl(testValue.i);
		}
		void SetRangeOffset(float rangeOffset) {
			union v {
				float f;
				uint32_t i;
			};
			v testValue;
			testValue.f = rangeOffset;
			rangeoffset = htonl(testValue.i);
		}
		const std::vector<uint8_t> ToData() const {
			std::vector<uint8_t> data(HeaderLength());
			std::memcpy(&data[TCPNDMAZIMUTHSAMPLESOFFSET], &azimuthsamples, sizeof(azimuthsamples));
			std::memcpy(&data[TCPNDMBINSIZEOFFSET], &binsize, sizeof(binsize));
			std::memcpy(&data[TCPNDMRANGEINBINSOFFSET], &rangeinbins, sizeof(rangeinbins));
			std::memcpy(&data[TCPNDMENCODERSIZEOFFSET], &encodersize, sizeof(encodersize));
			std::memcpy(&data[TCPNDMROTATIONSPEEDOFFSET], &rotationspeed, sizeof(rotationspeed));
			std::memcpy(&data[TCPNDMPACKETRATEOFFSET], &packetrate, sizeof(packetrate));
			std::memcpy(&data[TCPNDMRANGEGAINOFFSET], &rangegain, sizeof(rangegain));
			std::memcpy(&data[TCPNDMRANGEOFFSETOFFSET], &rangeoffset, sizeof(rangeoffset));
			return data;
		}
	}
	CNDPConfigurationHeaderStruct;
#pragma pack(pop)
}
