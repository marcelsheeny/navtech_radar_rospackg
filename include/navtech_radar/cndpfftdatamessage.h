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
		const uint8_t	TCPNDMFFTDATAOFFSETOFFSET = 0;
		const uint8_t	TCPNDMFFTDATAOFFSETLENGTH = sizeof(uint16_t);
		const uint8_t	TCPNDMSWEEPCOUNTEROFFSET = TCPNDMFFTDATAOFFSETOFFSET + TCPNDMFFTDATAOFFSETLENGTH;
		const uint8_t	TCPNDMSWEEPCOUNTERLENGTH = sizeof(uint16_t);
		const uint8_t	TCPNDMAZIMUTHOFFSET = TCPNDMSWEEPCOUNTEROFFSET + TCPNDMSWEEPCOUNTERLENGTH;
		const uint8_t	TCPNDMAZIMUTHLENGTH = sizeof(uint16_t);
		const uint8_t	TCPNDMSECONDSOFFSET = TCPNDMAZIMUTHOFFSET + TCPNDMAZIMUTHLENGTH;
		const uint8_t	TCPNDMSECONDSLENGTH = sizeof(uint32_t);
		const uint8_t	TCPNDMSPLITSECONDSOFFSET = TCPNDMSECONDSOFFSET + TCPNDMSECONDSLENGTH;
		const uint8_t	TCPNDMSPLITSECONDSLENGTH = sizeof(uint32_t);
		const uint8_t	TCPNDMFFTDATAHEADERLENGTH = TCPNDMFFTDATAOFFSETLENGTH + TCPNDMSWEEPCOUNTERLENGTH + TCPNDMAZIMUTHLENGTH + TCPNDMSECONDSLENGTH + TCPNDMSPLITSECONDSLENGTH;
	}

#pragma pack(push)
#pragma pack(1)
	typedef struct CNDPNetworkDataFftDataHeader {
		CNDPNetworkDataHeaderStruct header;
		uint16_t fftdataoffset;
		uint16_t sweepcounter;
		uint16_t azimuth;
		uint32_t seconds;
		uint32_t splitseconds;
		void Init() {
			header.Init();
			header.messageid = static_cast<uint8_t>(CNDPNetworkDataMessageType::FFTData);
			fftdataoffset = htons(TCPNDMFFTDATAHEADERLENGTH);
			seconds = 0;
			splitseconds = 0;
		}
		constexpr const std::size_t HeaderLength() const {
			return TCPNDMFFTDATAHEADERLENGTH;
		}
		void SetFftDataLength(std::size_t  length) {
			header.SetPayloadLength(TCPNDMFFTDATAHEADERLENGTH + length);
		}
		void SetSweepCounter(std::uint16_t  counter) {
			sweepcounter = htons(counter);
		}
		void SetAzimuth(std::uint16_t  azi) {
			azimuth = htons(azi);
		}
		void SetSeconds(uint32_t  secondsToSet) {
			seconds = htonl(secondsToSet);
		}
		void SetSplitSeconds(uint32_t  splitSeconds) {
			splitseconds = ntohl(splitSeconds);
		}
		const std::vector<uint8_t> ToData() const {
			std::vector<uint8_t> data(header.PayloadLength());
			std::memcpy(&data[TCPNDMFFTDATAOFFSETOFFSET], &fftdataoffset, sizeof(fftdataoffset));
			std::memcpy(&data[TCPNDMSWEEPCOUNTEROFFSET], &sweepcounter, sizeof(sweepcounter));
			std::memcpy(&data[TCPNDMAZIMUTHOFFSET], &azimuth, sizeof(azimuth));
			std::memcpy(&data[TCPNDMSECONDSOFFSET], &seconds, sizeof(seconds));
			std::memcpy(&data[TCPNDMSPLITSECONDSOFFSET], &splitseconds, sizeof(splitseconds));
			return data;
		}
	}
	CNDPNetworkDataFftDataHeaderStruct;
#pragma pack(pop)
}
