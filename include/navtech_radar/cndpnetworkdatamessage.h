/*
Copyright 2016 Navtech Radar Limited
This file is part of iasdk which is released under The MIT License (MIT).
See file LICENSE.txt in project root or go to
https://opensource.org/licenses/MIT for full license details.
*/

#pragma once

#include "common.h"

#include <cstdint>
#include <cstring>
#include <memory>
#include <vector>

#ifdef __BSDSOCKETS__
#include <netinet/in.h>
#elif _WIN32
#include <winsock2.h>
#endif

namespace Navtech {
const uint8_t NDMSYNC_HEADER_LENGTH = 16;
const uint8_t NDMSIGNATURELENGTH = NDMSYNC_HEADER_LENGTH * sizeof(uint8_t);

namespace {
const uint32_t MAXPAYLOADSIZE = 1048576;
const uint8_t NDMSIGNATUREOFFSET = 0;

const uint8_t NDMVERSIONOFFSET = NDMSIGNATUREOFFSET + NDMSIGNATURELENGTH;
const uint8_t NDMVERSIONLENGTH = sizeof(uint8_t);
const uint8_t NDMMESSAGEIDOFFSET = NDMVERSIONOFFSET + NDMVERSIONLENGTH;
const uint8_t NDMMESSAGEIDLENGTH = sizeof(uint8_t);
const uint8_t NDMPAYLOADSIZEOFFSET = NDMMESSAGEIDOFFSET + NDMMESSAGEIDLENGTH;
const uint8_t NDMPAYLOADSIZELENGTH = sizeof(uint32_t);
const uint8_t NDMHEADERLENGTH = NDMSIGNATURELENGTH + NDMVERSIONLENGTH +
                                NDMMESSAGEIDLENGTH + NDMPAYLOADSIZELENGTH;
const uint8_t NDMPROTOCOLMINVERSION = 1;
const uint8_t NDMPROTOCOLVERSION = 1;
}  // namespace

const uint8_t NDMSIGNATUREBYTES[NDMSIGNATURELENGTH] = {
    0x00, 0x01, 0x03, 0x03, 0x07, 0x07, 0x0f, 0x0f,
    0x1f, 0x1f, 0x3f, 0x3f, 0x7f, 0x7f, 0xfe, 0xfe};

enum class CNDPNetworkDataMessageType : uint8_t {
  Invalid = 0,
  KeepAlive = 1,
  Configuration = 10,
  ConfigurationRequest = 20,
  StartFFTData = 21,
  StopFFTData = 22,
  StartHealthMsgs = 23,
  StopHealthMsgs = 24,
  ReCalibrateRfHealth = 25,
  StartTracks = 26,
  StopTracks = 27,
  TransmitOn = 28,
  TransmitOff = 29,
  FFTData = 30,
  HighPrecisionFFTData = 31,
  Health = 40,
  ContourUpdate = 50,
  TrackUpdate = 60,
  TrackerConfiguration = 70,
  TrackerPlaybackCommand = 71,
  TrackerSaveClutterMap = 72,
  TrackerLegacyHealthUnits = 73,
  TrackerDataSourceUpdate = 74,
  TrackerDistributionUpdate = 75,
  SystemRestart = 76,
  LoggingLevels = 90,
  LoggingLevelsRequest = 100,
  SetAutoTune = 110,
  StartNavData = 120,
  StopNavData = 121,
  SetNavThreshold = 122,
  NavigationData = 123,
  SetNavRangeOffsetAndGain = 124,
  CalibrateAccelerometer = 125,
  StartAccelerometer = 126,
  StopAccelerometer = 127,
  AccelerometerData = 128,
};

#pragma pack(push)
#pragma pack(1)
struct NetworkDataHeader {
  uint8_t signature[NDMSYNC_HEADER_LENGTH];
  uint8_t version;
  uint8_t messageid;
  uint32_t payloadsize;
  void Init() {
    std::memcpy(&signature, &NDMSIGNATUREBYTES, sizeof(signature));
    version = NDMPROTOCOLVERSION;
    messageid = static_cast<uint8_t>(CNDPNetworkDataMessageType::Invalid);
    payloadsize = 0;
  }
  void SetMessageId(CNDPNetworkDataMessageType messageType) {
    messageid = static_cast<uint8_t>(messageType);
  }
  void SetPayloadLength(std::size_t length) { payloadsize = htonl(length); }
  const std::size_t PayloadLength() const { return ntohl(payloadsize); }
  const std::size_t HeaderLength() const { return NDMHEADERLENGTH; }
  const bool HeaderIsValid() const {
    auto result = true;

    // Ensure signature is valid
    for (auto i = 0u; i < NDMSIGNATURELENGTH; i++) {
      result &= NDMSIGNATUREBYTES[i] == signature[i];
    }

    // Ensure version number is in valid range
    result &=
        (version >= NDMPROTOCOLMINVERSION && version <= NDMPROTOCOLVERSION);

    // Ensure Payload Length is not too large
    result &= (PayloadLength() <= MAXPAYLOADSIZE);

    return result;
  }
  const std::vector<uint8_t> ToData() const {
    std::vector<uint8_t> headerData(NDMHEADERLENGTH);
    std::memcpy(&headerData[0], &signature, sizeof(signature));
    std::memcpy(&headerData[NDMVERSIONOFFSET], &version, sizeof(version));
    std::memcpy(&headerData[NDMMESSAGEIDOFFSET], &messageid, sizeof(messageid));
    std::memcpy(&headerData[NDMPAYLOADSIZEOFFSET], &payloadsize,
                sizeof(payloadsize));
    return headerData;
  }
};
#pragma pack(pop)

class NetworkDataMessage {
 public:
  explicit NetworkDataMessage(const NetworkDataHeader header)
      : NetworkDataMessage(header, std::vector<uint8_t>()) {}

  explicit NetworkDataMessage(const NetworkDataHeader header,
                              const uint8_t* payload, const std::size_t size)
      : NetworkDataMessage(header,
                           std::vector<uint8_t>(payload, payload + size)) {}

  explicit NetworkDataMessage(const NetworkDataHeader header,
                              const std::vector<uint8_t>& data)
      : _header(header), _payload(data) {}

  ~NetworkDataMessage() {
    _payload.clear();
    std::vector<uint8_t>(_payload).swap(_payload);
  }

  explicit NetworkDataMessage(const NetworkDataMessage&) = delete;
  NetworkDataMessage& operator=(const NetworkDataMessage&) = delete;

  const std::vector<uint8_t> Payload() const { return _payload; }
  const CNDPNetworkDataMessageType MessageId() const {
    return static_cast<CNDPNetworkDataMessageType>(_header.messageid);
  }
  const uint32_t PayloadSize() const { return _header.PayloadLength(); }
  const uint8_t Version() const { return _header.version; }
  const bool MessageValid() const { return _header.HeaderIsValid(); }
  std::vector<uint8_t> MessageData() {
    auto header = _header.ToData();

    if (_payload.size() > 0)
      header.insert(header.end(), _payload.begin(), _payload.end());

    return header;
  }

 protected:
  const NetworkDataHeader _header;
  std::vector<uint8_t> _payload;
};

typedef std::shared_ptr<NetworkDataMessage> CNDPDataMessagePtr_t;

#pragma pack(push)
#pragma pack(1)
typedef struct CNDPNetworkDataHeader : public NetworkDataHeader {
} CNDPNetworkDataHeaderStruct;
#pragma pack(pop)
}  // namespace Navtech
