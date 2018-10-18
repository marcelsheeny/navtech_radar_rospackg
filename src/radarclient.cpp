/*
Copyright 2016 Navtech Radar Limited
This file is part of iasdk which is released under The MIT License (MIT).
See file LICENSE.txt in project root or go to
https://opensource.org/licenses/MIT
for full license details.
*/

#include "radarclient.h"
#include "cndpconfigurationmessage.h"
#include "cndpfftdatamessage.h"
#include "cndpnetworkdatamessage.h"

#include <cmath>
#include <cstring>

#ifdef __BSDSOCKETS__
#include <netinet/in.h>
#elif _WIN32
#include <WinSock2.h>
#endif

namespace {
const uint32_t RANGEMULTIPLIER = 1000000;
const float RANGEMULTIPLIERFLOAT = 1000000.0f;
const uint32_t NAVDATARECORDLENGTH = (sizeof(uint32_t) + sizeof(uint16_t));
} // namespace

Navtech::RadarClient::RadarClient(const std::string &radarAddress,
                                  const uint16_t &port)
    : _radarClient(std::make_shared<TcpRadarClient>(radarAddress, port)),
      _running(false), _sendRadarData(false) {}

void Navtech::RadarClient::SetFFTDataCallback(
    std::function<void(const FFTDataPtr_t &)> callback) {
  std::lock_guard<std::mutex> lock(_callbackMutex);
  _fftDataCallback = callback;
}

void Navtech::RadarClient::SetNavigationDataCallback(
    std::function<void(const NavigationDataPtr_t &)> callback) {
  std::lock_guard<std::mutex> lock(_callbackMutex);
  _navigationDataCallback = callback;
}

void Navtech::RadarClient::SetConfigurationDataCallback(
    std::function<void(const ConfigurationDataPtr_t &)> callback) {
  std::lock_guard<std::mutex> lock(_callbackMutex);
  _configurationDataCallback = callback;
}

void Navtech::RadarClient::Start() {
  if (_running)
    return;
  Helpers::Log("RadarClient - Starting");

  _radarClient->SetReceiveDataCallback(
      std::bind(&RadarClient::HandleData, this, std::placeholders::_1));
  _radarClient->Start();
  _running = true;

  Helpers::Log("RadarClient - Started");
}

void Navtech::RadarClient::Stop() {
  if (!_running)
    return;

  _running = false;

  _radarClient->Stop();
  Helpers::Log("RadarClient - Stopped");
}

void Navtech::RadarClient::StartFFTData() {
  Helpers::Log("RadarClient - Start FFT Data");
  SendSimpleNetworkMessage(CNDPNetworkDataMessageType::StartFFTData);
  _sendRadarData = true;
}

void Navtech::RadarClient::StopFFTData() {
  Helpers::Log("RadarClient - Stop FFT Data");
  SendSimpleNetworkMessage(CNDPNetworkDataMessageType::StopFFTData);
  _sendRadarData = false;
}

void Navtech::RadarClient::StartNavigationData() {
  Helpers::Log("RadarClient - Start Navigation Data");
  SendSimpleNetworkMessage(CNDPNetworkDataMessageType::StartNavData);
}

void Navtech::RadarClient::StopNavigationData() {
  Helpers::Log("RadarClient - Stop Navigation Data");
  SendSimpleNetworkMessage(CNDPNetworkDataMessageType::StopNavData);
}

void Navtech::RadarClient::SetNavigationThreshold(const uint16_t &threshold) {
  if (_radarClient == nullptr ||
      _radarClient->GetConnectionState() != ConnectionState::Connected)
    return;

  std::vector<uint8_t> buffer(sizeof(threshold));
  auto networkThreshold = ntohs(threshold);
  std::memcpy(buffer.data(), &networkThreshold, sizeof(networkThreshold));

  CNDPNetworkDataHeader header;
  header.Init();
  header.SetMessageId(CNDPNetworkDataMessageType::SetNavThreshold);
  auto message = std::make_shared<NetworkDataMessage>(header, buffer);
  _radarClient->Send(message->MessageData());
}

void Navtech::RadarClient::SetNavigationGainAndOffset(const float &gain,
                                                      const float &offset) {
  if (_radarClient == nullptr ||
      _radarClient->GetConnectionState() != ConnectionState::Connected)
    return;

  std::vector<uint8_t> buffer(sizeof(uint32_t) * 2);
  uint32_t netGain = htonl(gain * RANGEMULTIPLIER);
  uint32_t netOffset = htonl(offset * RANGEMULTIPLIER);
  std::memcpy(buffer.data(), &netGain, sizeof(netGain));
  std::memcpy(&buffer[sizeof(netGain)], &netOffset, sizeof(netOffset));

  CNDPNetworkDataHeader header;
  header.Init();
  header.SetMessageId(CNDPNetworkDataMessageType::SetNavRangeOffsetAndGain);
  auto message = std::make_shared<NetworkDataMessage>(header, buffer);
  _radarClient->Send(message->MessageData());
}

void Navtech::RadarClient::SendSimpleNetworkMessage(
    const Navtech::CNDPNetworkDataMessageType &type) {
  if (_radarClient == nullptr ||
      _radarClient->GetConnectionState() != ConnectionState::Connected)
    return;

  CNDPNetworkDataHeader header;
  header.Init();
  header.SetMessageId(type);
  auto message = std::make_shared<NetworkDataMessage>(header);
  _radarClient->Send(message->MessageData());
}

void Navtech::RadarClient::UpdateContourMap(
    const std::vector<uint8_t> &contourData) {
  if (_radarClient == nullptr ||
      _radarClient->GetConnectionState() != ConnectionState::Connected)
    return;

  auto contour = std::vector<uint8_t>();
  if (contourData.size() == 720) {
    contour.resize(contourData.size());
    auto resolution = _binSize / 10000.0;
    for (std::size_t i = 0; i < contourData.size(); i += 2) {
      auto result = (((contourData[i]) << 8) | ((contourData[i + 1])));
      result = (uint16_t)std::ceil(result / resolution);
      contour[i] = (result & 0xff00) >> 8;
      contour[i + 1] = result & 0x00ff;
    }
  }

  CNDPNetworkDataHeader header;
  header.Init();
  header.SetMessageId(CNDPNetworkDataMessageType::ContourUpdate);
  header.SetPayloadLength(contour.size());
  auto message = std::make_shared<NetworkDataMessage>(header, contour);
  _radarClient->Send(message->MessageData());
}

void Navtech::RadarClient::HandleData(const CNDPDataMessagePtr_t &message) {
  switch (message->MessageId()) {
  case CNDPNetworkDataMessageType::Configuration:
    HandleConfigurationMessage(message);
    break;
  case CNDPNetworkDataMessageType::FFTData:
    HandleFFTDataMessage(message);
    break;
  case CNDPNetworkDataMessageType::HighPrecisionFFTData:
    HandleFFTDataMessage(message);
    break;
  case CNDPNetworkDataMessageType::NavigationData:
    HandleNavigationDataMessage(message);
    break;
  default:
    Helpers::Log("RadarClient - Unhandled Message [" +
                 std::to_string(static_cast<uint32_t>(message->MessageId())) +
                 "]");
    break;
  }
}

void Navtech::RadarClient::HandleConfigurationMessage(
    const Navtech::CNDPDataMessagePtr_t &configMessage) {
  Helpers::Log("RadarClient - Handle Configuration Message");

  CNDPConfigurationHeaderStruct configHeader;
  std::memset(&configHeader, 0, sizeof(configHeader));
  std::memcpy(&configHeader, configMessage->MessageData().data(),
              configHeader.HeaderLength() + configHeader.header.HeaderLength());

  _azimuthSamples = ntohs(configHeader.azimuthsamples);
  _binSize = ntohs(configHeader.binsize);
  _rangeinbins = ntohs(configHeader.rangeinbins);
  _encoderSize = ntohs(configHeader.encodersize);
  _expectedRotationRate = ntohs(configHeader.rotationspeed) / 1000.0f;

  if (_sendRadarData)
    SendSimpleNetworkMessage(CNDPNetworkDataMessageType::StartFFTData);

  auto configurationData = std::make_shared<ConfigurationData>();
  configurationData->AzimuthSamples = _azimuthSamples;
  configurationData->BinSize = _binSize;
  configurationData->RangeInBins = _rangeinbins;
  configurationData->EncoderSize = _encoderSize;
  configurationData->ExpectedRotationRate = _expectedRotationRate;

  _callbackMutex.lock();
  auto configurationDataCallback = _configurationDataCallback;
  _callbackMutex.unlock();
  if (configurationDataCallback == nullptr)
    return;

  configurationDataCallback(configurationData);
}

void Navtech::RadarClient::HandleFFTDataMessage(
    const Navtech::CNDPDataMessagePtr_t &fftDataMessage) {
  _callbackMutex.lock();
  auto fftDataCallback = _fftDataCallback;
  _callbackMutex.unlock();
  if (fftDataCallback == nullptr)
    return;

  CNDPNetworkDataFftDataHeaderStruct fftDataHeader;
  std::memset(&fftDataHeader, 0, sizeof(fftDataHeader));
  auto messageData = fftDataMessage->MessageData();

  std::memcpy(&fftDataHeader, messageData.data(),
              fftDataHeader.HeaderLength() +
                  fftDataHeader.header.HeaderLength());

  auto fftData = std::make_shared<FFTData>();
  fftData->Azimuth = ntohs(fftDataHeader.azimuth);
  fftData->Angle = (fftData->Azimuth * 360.0f) / (float)_encoderSize;
  fftData->SweepCounter = ntohs(fftDataHeader.sweepcounter);
  fftData->NTPSeconds = fftDataHeader.seconds;
  fftData->NTPSplitSeconds = fftDataHeader.splitseconds;
  fftData->Data.resize(fftDataHeader.header.PayloadLength() -
                       fftDataHeader.HeaderLength());
  std::memcpy(fftData->Data.data(),
              &messageData[fftDataHeader.header.HeaderLength() +
                           fftDataHeader.HeaderLength()],
              fftData->Data.size());

  fftDataCallback(fftData);
}

void Navtech::RadarClient::HandleNavigationDataMessage(
    const Navtech::CNDPDataMessagePtr_t &navigationMessage) {
  _callbackMutex.lock();
  auto navigationDataCallback = _navigationDataCallback;
  _callbackMutex.unlock();
  if (navigationDataCallback == nullptr)
    return;

  uint16_t netAzimuth = 0;
  std::memcpy(&netAzimuth, &navigationMessage->Payload()[0],
              sizeof(netAzimuth));
  uint32_t netSeconds = 0;
  std::memcpy(&netSeconds, &navigationMessage->Payload()[sizeof(netAzimuth)],
              sizeof(netSeconds));
  uint32_t netSplitSeconds = 0;
  std::memcpy(
      &netSplitSeconds,
      &navigationMessage->Payload()[sizeof(netAzimuth) + sizeof(netSeconds)],
      sizeof(netSplitSeconds));

  auto navigationData = std::make_shared<NavigationData>();
  navigationData->Azimuth = ntohs(netAzimuth);
  navigationData->NTPSeconds = ntohl(netSeconds);
  navigationData->NTPSplitSeconds = ntohl(netSplitSeconds);
  navigationData->Angle =
      (navigationData->Azimuth * 360.0f) / (float)_encoderSize;

  auto offset =
      sizeof(netAzimuth) + sizeof(netSeconds) + sizeof(netSplitSeconds);
  auto peaksCount =
      (navigationMessage->PayloadSize() - offset) / NAVDATARECORDLENGTH;
  for (auto i = offset; i < (peaksCount * NAVDATARECORDLENGTH);
       i += NAVDATARECORDLENGTH) {
    uint32_t peakResolve = 0;
    std::memcpy(&peakResolve, &navigationMessage->Payload()[i],
                sizeof(peakResolve));
    uint16_t power = 0;
    std::memcpy(&peakResolve,
                &navigationMessage->Payload()[i + sizeof(peakResolve)],
                sizeof(peakResolve));
    navigationData->Peaks.push_back(std::make_tuple<float, uint16_t>(
        htonl(peakResolve) / RANGEMULTIPLIERFLOAT, htons(power)));
  }

  navigationDataCallback(navigationData);
}
