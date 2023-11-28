/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <gz/msgs/dataframe.pb.h>

#include <limits>
#include <list>
#include <random>
#include <string>
#include <tuple>
#include <vector>
#include <unordered_map>
#include <utility>
#include <fstream>
#include <sstream>
#include <iostream>
#include <ctime>
#include <algorithm>
#include <iterator>
#include <bits/stdc++.h>

#include <sdf/sdf.hh>
#include <gz/common/Profiler.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Rand.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/plugin/Register.hh>
#include "gz/sim/comms/MsgManager.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"
#include "RFComms_custom.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

struct FileConfiguration
{
  std::string currentDateTime() 
  {
    std::time_t t = std::time(nullptr);
    std::tm* now = localtime(&t);

    char buffer[128];
    strftime(buffer, sizeof(buffer), "%Y_%m_%d_%X", now);
    return buffer;
  }

  std::string commsAnalysisDirPath = "/home/haruki/Desktop/ugv_comms_in_gazebo/comms_analysis/";
  // std::string antennaGainsDirPath = "/home/haruki/Desktop/ugv_comms_in_gazebo/antenna_gains/";
  
  friend std::ostream &operator<<(std::ostream &_oss,
                                  const FileConfiguration &_config)
  {
    _oss << "Comms Analysis Directory Path: " << _config.commsAnalysisDirPath << std::endl;
        //  << "Antenna Gain Directory Path: " << _config.antennaGainsDirPath << std::endl;

    return _oss;
  }
};


/// \brief Parameters for simple log-normal fading model.
struct RangeConfiguration
{
  /// \brief Hard limit on range. 通信可能な最大距離
  double maxRange = 50.0;

  /// \brief Fading exponent.
  double fadingExponent = 2.5;

  /// \brief Received power at 1m (in dBm).
  double l0 = 40;

  /// \brief Standard deviation for received power.
  double sigma = 10;

  /// Output stream operator.
  /// \param[out] _oss Stream.
  /// \param[in] _config configuration to output.
  friend std::ostream &operator<<(std::ostream &_oss,
                                  const RangeConfiguration &_config)
  {
    _oss << "RF Configuration (range-based)" << std::endl
         << "-- max_range: " << _config.maxRange << std::endl
         << "-- fading_exponent: " << _config.fadingExponent << std::endl
         << "-- l0: " << _config.l0 << std::endl
         << "-- sigma: " << _config.sigma << std::endl;

    return _oss;
  }
};

/// \brief Radio configuration parameters.
///
/// Static parameters such as channel capacity and transmit power.
struct RadioConfiguration
{
  /// \brief Speed of Light (in GHz).
  double speedOfLight = 3.0 * pow(10.0, 8.0);

  /// \brief Center Frequency (in GHz).
  double centerFrequency = 60.0;
  
  /// \brief Capacity of radio in bits-per-second.
  double capacity = 2.16 * pow(10.0, 9.0);

  /// \brief Default transmit power in dBm. Default is 27dBm or 500mW.
  double txPower = -10.0;

  /// \brief Modulation scheme, e.g., QPSK (Quadrature Phase Shift Keyring).
  std::string modulation = "QPSK";

  /// \brief Noise floor of the radio in dBm.
  double thermalNoiseDensity = -147.0;

  /// \brief Directly Path of horn antenna gains.
  std::string antennaGainsDirPath = "/home/haruki/Desktop/ugv_comms_in_gazebo/antenna_gains";

  /// \brief Vertical antenna direction gain.
  std::vector<std::vector<double>> ePlane;

  /// \brief holizontal antenna direction gain.
  std::vector<std::vector<double>> hPlane;

  ignition::math::Quaterniond txAntennaRot = ignition::math::Quaterniond(M_PI/2, 0., -3*M_PI/4);

  ignition::math::Quaterniond rxAntennaRot = ignition::math::Quaterniond(M_PI/2, 0., M_PI/4);

  double maxAntennaGain;

  /// Output stream operator.
  /// \param _oss Stream.
  /// \param _config configuration to output.
  friend std::ostream &operator<<(std::ostream &_oss,
                                  const RadioConfiguration &_config)
  {
    _oss << "Radio Configuration" << std::endl
         << "-- center_frequency: " << _config.centerFrequency << std::endl
         << "-- capacity: " << _config.capacity << std::endl
         << "-- tx_power: " << _config.txPower << std::endl
         << "-- thermal_noise_density: " << _config.thermalNoiseDensity << std::endl
         << "-- modulation: " << _config.modulation << std::endl;

    return _oss;
  }
};

/// \brief Store radio state
///
/// Structure to hold radio state including the pose and book-keeping
/// necessary to implement bitrate limits.
struct RadioState
{
  /// \brief Timestamp of last update.
  double timeStamp;

  /// \brief Pose of the radio.
  ignition::math::Pose3<double> pose;

  /// \brief Angle of the radio.
  ignition::math::Quaterniond antennaRot;

  /// \brief Recent sent packet history.filePath
  std::list<std::pair<double, uint64_t>> bytesSent;

  /// \brief Accumulation of bytes sent in an epoch.
  uint64_t bytesSentThisEpoch = 0;

  /// \brief Recent received packet history.
  std::list<std::pair<double, uint64_t>> bytesReceived;

  /// \brief Accumulation of bytes received in an epoch.
  uint64_t bytesReceivedThisEpoch = 0;

  /// \brief Name of the model associated with the radio.
  std::string name;
};

/// \brief Type for holding RF power as a Normally distributed random variable.
struct RFPower
{
  /// \brief Expected value of RF power.
  double mean;

  /// \brief Variance of RF power.
  double variance;

  /// \brief double operator.
  /// \return the RFPower as a double.
  operator double() const
  {
    return mean;
  }
};

/// \brief Private RFComms_custom data class.
class ignition::gazebo::systems::RFComms_custom::Implementation
{
  /// \brief Attempt communication between two nodes.
  ///
  /// The radio configuration, transmitter and receiver state, and
  /// packet size are all used to compute the probability of successful
  /// communication (i.e., based on both SNR and bitrate
  /// limitations). This probability is then used to determine if the
  /// packet is successfully communicated.
  ///
  /// \param[in out] _txState Current state of the transmitter.
  /// \param[in out] _rxState Current state of the receiver.
  /// \param[in] _numBytes Size of the packet.
  /// \return std::tuple<bool, double> reporting if the packet should be
  /// delivered and the received signal strength (in dBm).
  public: std::tuple<bool, double, double, double, double> AttemptSend(RadioState &_txState,
                                               RadioState &_rxState,
                                               const uint64_t &_numBytes);

  public: double RssiToSnr(int i, std::vector<double> _eachRssi);
  
  public: double AttemptMeasure(RadioState &_txState,
                                RadioState &_rxState);

  public: std::tuple<bool, double> AttemptSend2(std::vector<double> &_SINR, const uint64_t &_numBytes);

  /// \brief Convert from e_plane.csv and h_plane.csv to two dimensional array.
  public: std::vector<std::vector<double>> FileToArray(std::string _filePath) const;

  private: double PoseToGain(const RadioState &_initialPointState,
                             const RadioState &_terminalPointState) const;

  /// \brief Convert degree to radian.
  public: double DegreeToRadian(double _degree) const;

  /// \brief Convert degree to radian.
  public: double RadianToDegree(double _radian) const;

  /// \brief Convert from dBm to power.
  /// \param[in] _dBm Input in dBm.
  /// \return Power in watts (W).
  private: double DbmToPow(double _dBm) const;

  /// \brief Compute the bit error rate (BER).
  /// \param[in] _power Rx power (dBm).
  /// \param[in] _noise Noise value (dBm).
  /// \return Based on rx_power, noise value, and modulation, compute the bit
  // error rate (BER).
  private: double QPSKPowerToBER(double _power,
                                 double _noise) const;

  private: double PowerToBER(double _power,
                             double _noise,
                             std::string _modulation) const;
  
  private: double PowerToThroughput(double _power) const;

  /// \brief Function to compute the pathloss between two antenna poses.
  /// \param[in] _txPower Tx power.
  /// \param[in] _txState Radio state of the transmitter.
  /// \param[in] _rxState Radio state of the receiver.
  /// \return The RFPower pathloss distribution of the two antenna poses. https://en.wikipedia.org/wiki/Log-distance_path_loss_model
  private: RFPower FreeSpaceReceivedPower(const double &_txPower,
                                          const RadioState &_txState,
                                          const RadioState &_rxState) const;
  
  private: RFPower LogNormalReceivedPower(const double &_txPower,
                                          const RadioState &_txState,
                                          const RadioState &_rxState) const;
  
  private: RFPower BasedOn3gppR16ReceivedPower(const double &_txPower,
                                          const RadioState &_txState,
                                          const RadioState &_rxState) const;

  /// \brief Range configuration.
  public: RangeConfiguration rangeConfig;

  /// \brief Radio configuration.
  public: RadioConfiguration radioConfig;

  /// \brief File configuration.
  public: FileConfiguration fileConfig;

  /// \brief A map where the key is the address and the value its radio state.
  public: std::unordered_map<std::string, RadioState> radioStates;

  /// \brief Duration of an epoch (seconds).
  public: double epochDuration = 1.0;

  /// \brief Random device to seed random engine
  public: std::random_device rd{};

  /// \brief Random number generator.
  public: std::default_random_engine rndEngine{rd()};

  /// \brief Writing file.
  public: std::ofstream writing_file;

};

/////////////////////////////////////////////
std::vector<std::vector<double>> RFComms_custom::Implementation::FileToArray(std::string _filePath) const
{ 
  std::ifstream file(_filePath);
  std::vector<std::vector<double>> array;

  if (file)
  {
    std::string line;
    while (getline(file, line))
    {
      std::vector<double> row;
      std::stringstream ss(line);

      std::string value;
      while (std::getline(ss, value, ','))
      {
        row.push_back(std::stod(value));
      }
      array.push_back(row);
    }
  }
  else
  {
    ignerr << "Cannot read " << _filePath << std::endl;
  }
  
  return array;
}

/////////////////////////////////////////////
double RFComms_custom::Implementation::DegreeToRadian(double _degree) const
{
  return _degree * M_PI/180.0;
}

double RFComms_custom::Implementation::RadianToDegree(double _radian) const
{
  return _radian * 180.0/M_PI; 
}

/////////////////////////////////////////////
double RFComms_custom::Implementation::DbmToPow(double _dBm) const
{
  return 0.001 * pow(10., _dBm / 10.);
}


/////////////////////////////////////////////
double RFComms_custom::Implementation::PoseToGain(
  const RadioState &_initialPointState,
  const RadioState &_terminalPointState) const
{
  auto direction = _initialPointState.antennaRot.RotateVectorReverse(_terminalPointState.pose.Pos() - _initialPointState.pose.Pos());
  
  // igndbg << "direction: " << direction << " [" << _initialPointState.name << "]" << std::endl;

  auto eDirection = direction;
  eDirection.Y(0.0);

  // igndbg << eDirection << std::endl;

  double theta = round((this->RadianToDegree(acos(eDirection.Normalized().Dot(ignition::math::Vector3d::UnitX))))*10.)/10.;
  if (eDirection.Z() < 0.0)
  {
    theta = -theta;
  }

  auto itEPlane = std::find_if(
    std::begin(this->radioConfig.ePlane), std::end(this->radioConfig.ePlane),
    [&](const auto& row) {
      return row.at(0) == theta;
    }
  );

  auto hDirection = direction;
  hDirection.Z(0.0);

  // igndbg << hDirection << std::endl;

  double varphi = round(this->RadianToDegree(acos(hDirection.Normalized().Dot(ignition::math::Vector3d::UnitX)))*10.)/10.;
  if (hDirection.Y() < 0.0)
  {
    varphi = -varphi;
  }

  auto itHPlane = std::find_if(
    std::begin(this->radioConfig.hPlane), std::end(this->radioConfig.hPlane),
    [&](const auto& row) {
      return row.at(0) == varphi;
    }
  );

  if (itEPlane == std::end(this->radioConfig.ePlane) || itHPlane == std::end(this->radioConfig.hPlane)) 
  {
    return -std::numeric_limits<double>::infinity();
  }

  const double antennaGain = this->radioConfig.ePlane[std::distance(std::begin(this->radioConfig.ePlane), itEPlane)][1]
                             + this->radioConfig.hPlane[std::distance(std::begin(this->radioConfig.hPlane), itHPlane)][1]
                             - this->radioConfig.maxAntennaGain; // 半値角から考えるのであれば maxAntennaGain->10log10(M_PI)??

  // igndbg << "[" << _initialPointState.name << "]" << std::endl;
  // igndbg << "theta: " << theta << " varphi: " << varphi << std::endl;
  // igndbg << "AntennaGain(E-Plane): " << this->radioConfig.ePlane[std::distance(std::begin(this->radioConfig.ePlane), itEPlane)][1] << std::endl;
  // igndbg << "AntennaGain(H-Plane): " << this->radioConfig.hPlane[std::distance(std::begin(this->radioConfig.hPlane), itHPlane)][1] << std::endl;
  // igndbg << "AntennaGain3d: " << antennaGain << std::endl;
                            
  return antennaGain;
}

////////////////////////////////////////////
double RFComms_custom::Implementation::QPSKPowerToBER(
  double _power, double _noise) const
{
  return erfc(sqrt(_power / _noise)); // これあってる？？
}

double RFComms_custom::Implementation::PowerToBER(
  double _power, double _noise, std::string _modulation) const
{ 
  double SNR = 10.0 * log10(_power/_noise);

  if (_modulation.compare("QPSK") == 0)
  {
    return 0.5 * erfc(sqrt(SNR/2.0));
  } 
  
  if (_modulation.compare("16QAM"))
  {
    return 0.375 * erfc(sqrt(SNR/10.0));
  } 
  
  if (_modulation.compare("64QAM"))
  {
    return (sqrt(64.0) - 1.0)/(log2(64.0) * pow(2.0, log2(64.0)/2.0 - 1.0)) * erfc(sqrt(SNR * (3.0/(2.0 * (64.0 - 1.0)))));
  }

  if (_modulation.compare("256QAM"))
  {
    return (sqrt(256.0) - 1.0)/(log2(256.0) * pow(2.0, log2(256.0)/2.0 - 1.0)) * erfc(sqrt(SNR * (3.0/(2.0 * (256.0 - 1.0)))));
  }

  return std::numeric_limits<double>::lowest();
}

// depend on NICT
double RFComms_custom::Implementation::PowerToThroughput(
  double _power) const
{ 
  // std::cout << _power << std::endl;
  if (_power > -51.0)
  {
    return 6.0; // in Gbps
  } 
  else if (_power > -55.0)
  {
    return 4.7;
  } 
  else if (_power > -58.5)
  {
    return 2.7;
  }
  else if (_power > -61.5)
  {
    return 2.15;
  }
  else if (_power > -63.5)
  {
    return 1.1;
  }
  else if (_power > -65.5)
  {
    return 0.5;
  }
  else {
    return 0.0;
  }
}


/////////////////////////////////////////////
RFPower RFComms_custom::Implementation::FreeSpaceReceivedPower(
  const double &_txPower, const RadioState &_txState,
  const RadioState &_rxState) const
{
  const double kRange = _txState.pose.Pos().Distance(_rxState.pose.Pos());

  if (this->rangeConfig.maxRange > 0.0 && kRange > this->rangeConfig.maxRange) // 通信可能範囲外の場合
    return {-std::numeric_limits<double>::infinity(), 0.0};

  const double kPL = 20. * log10(4. * M_PI * kRange * this->radioConfig.centerFrequency/this->radioConfig.speedOfLight);

  return {_txPower - kPL, pow(this->rangeConfig.sigma, 2.)};
}

/////////////////////////////////////////////
RFPower RFComms_custom::Implementation::LogNormalReceivedPower(
  const double &_txPower, const RadioState &_txState,
  const RadioState &_rxState) const
{
  const double kRange = _txState.pose.Pos().Distance(_rxState.pose.Pos());

  if (this->rangeConfig.maxRange > 0.0 && kRange > this->rangeConfig.maxRange) // 通信可能範囲外の場合
    return {-std::numeric_limits<double>::infinity(), 0.0};

  const double kPL = this->rangeConfig.l0 +
    10 * this->rangeConfig.fadingExponent * log10(kRange); // 対数距離経路損失モデル https://en.wikipedia.org/wiki/Log-distance_path_loss_model

  return {_txPower - kPL, pow(this->rangeConfig.sigma, 2.)};
}

/////////////////////////////////////////////
RFPower RFComms_custom::Implementation::BasedOn3gppR16ReceivedPower(
  const double &_txPower, const RadioState &_txState,
  const RadioState &_rxState) const
{
  const double kRange = _txState.pose.Pos().Distance(_rxState.pose.Pos()); // 現在はd_3dとd_2dは同じ
 
  if (this->rangeConfig.maxRange > 0.0 && kRange > this->rangeConfig.maxRange) // 通信可能範囲外の場合
    return {-std::numeric_limits<double>::infinity(), 0.0};

  const double speedOfLight = 3. * pow(10., 8.); // speed of light

  const double baseStationHeight = _txState.pose.Pos()[2]; // 物体の中心になるので注意
  const double ugvHeight = _rxState.pose.Pos()[2]; // 物体の中心になるので注意

  const double effectiveEnvironmentRange = 1.;
  const double breakPointRange = 4. * (_txState.pose.Pos()[2] - effectiveEnvironmentRange) * (_rxState.pose.Pos()[2] - effectiveEnvironmentRange) * (this->radioConfig.centerFrequency/this->radioConfig.speedOfLight);

  double kPL; // https://www.etsi.org/deliver/etsi_tr/138900_138999/138901/16.01.00_60/tr_138901v160100p.pdf
  
  if (kRange < breakPointRange)
  {
    kPL = 28. + 22. * log10(kRange) + 20. * log10(this->radioConfig.centerFrequency);
  } else {
    kPL = 28. + 40. * log10(kRange) + 20. * log10(this->radioConfig.centerFrequency) - 9. * log10(pow(breakPointRange, 2.) + (baseStationHeight - ugvHeight));
  }

  double losProbability; // los確率
  if (kRange < 18.0)
  {
    losProbability = 1.;
  } else { 
    losProbability = (18./kRange) + exp(-kRange/63.) * (1. - 18./kRange);
  }

  const bool isLos = ignition::math::Rand::DblUniform() < losProbability;

  if (!isLos)
  {
    kPL = std::max(kPL, 13.54 + 39.08 * log(kRange) + 20. * log10(this->radioConfig.centerFrequency) - 0.6 * (ugvHeight - 1.5));
    return {_txPower - kPL, pow(3., 2.)};
  }

  return {_txPower - kPL, pow(4., 2.)};
}

/////////////////////////////////////////////
std::tuple<bool, double, double, double, double> RFComms_custom::Implementation::AttemptSend(
  RadioState &_txState, RadioState &_rxState, const uint64_t &_numBytes)
{
  double now = _txState.timeStamp;

  // Maintain running window of bytes sent over the last epoch, e.g., 1s.
  while (!_txState.bytesSent.empty() &&
         _txState.bytesSent.front().first <= now - this->epochDuration)
  {
    _txState.bytesSentThisEpoch -= _txState.bytesSent.front().second;
    _txState.bytesSent.pop_front();
  }

  // igndbg << "Bytes sent: " <<  _txState.bytesSentThisEpoch << " + "
  //        << _numBytes << " = "
  //        << _txState.bytesSentThisEpoch + _numBytes << std::endl;

  // Compute prospective accumulated bits along with time window
  // (including this packet).
  double bitsSent = (_txState.bytesSentThisEpoch + _numBytes) * 8;

  // Check current epoch bitrate vs capacity and fail to send accordingly
  if (bitsSent > this->radioConfig.capacity * this->epochDuration)
  {
    ignwarn << "Bitrate limited: [" << _txState.name << "] " << bitsSent
            << " bits sent (limit: "
            << this->radioConfig.capacity * this->epochDuration << ")"
            << std::endl;
    return std::make_tuple(false, -std::numeric_limits<double>::max(), 1.0, 1.0, std::numeric_limits<double>::lowest());
  }

  // Record these bytes.
  _txState.bytesSent.push_back(std::make_pair(now, _numBytes));
  _txState.bytesSentThisEpoch += _numBytes;

  // Get each antenna gain based on the angle of radiation/arrival.
  const double txAntennaGain = this->PoseToGain(_txState, _rxState);
  const double rxAnntennaGain = this->PoseToGain(_rxState, _txState);

  // Get the received power based on TX power and position of each node.
  // auto rxPowerDist =
  //   this->FreeSpaceReceivedPower(this->radioConfig.txPower, _txState, _rxState);

  auto rxPowerDist =
    this->LogNormalReceivedPower(this->radioConfig.txPower + txAntennaGain + rxAnntennaGain, _txState, _rxState);
  
  // auto rxPowerDist =
  //   this->BasedOn3gppR16ReceivedPower(this->radioConfig.txPower, _txState, _rxState);

  double rxPower = rxPowerDist.mean;
  if (rxPowerDist.variance > 0.0)
  {
    std::normal_distribution<> d{rxPowerDist.mean, sqrt(rxPowerDist.variance)};
    rxPower = d(this->rndEngine);
  }

  // Based on rx_power, noise value, and modulation, compute the bit
  // error rate (BER).
  double ber = this->QPSKPowerToBER(
    this->DbmToPow(rxPower), this->DbmToPow(this->radioConfig.thermalNoiseDensity));

  // double ber = this->PowerToBER(
  //   this->DbmToPow(rxPower), this->DbmToPow(this->radioConfig.thermalNoiseDensity), this->radioConfig.modulation);

  double packetDropProb = 1.0 - exp(_numBytes * log(1 - ber));

  igndbg << "TX power (dBm): " << this->radioConfig.txPower << "\n" <<
            "RX power (dBm): " << rxPower << "\n" <<
            "BER: " << ber << "\n" <<
            "# Bytes: " << _numBytes << "\n" <<
            "PER: " << packetDropProb << std::endl;

  double randDraw = ignition::math::Rand::DblUniform(); // 一様分布
  bool packetReceived = randDraw > packetDropProb;

  if (!packetReceived)
    return std::make_tuple(false, -std::numeric_limits<double>::max(), ber, packetDropProb, 0.0);

  // Maintain running window of bytes received over the last epoch, e.g., 1s.
  while (!_rxState.bytesReceived.empty() &&
         _rxState.bytesReceived.front().first <= now - this->epochDuration)
  {
    _rxState.bytesReceivedThisEpoch -= _rxState.bytesReceived.front().second;
    _rxState.bytesReceived.pop_front();
  }

  // igndbg << "bytes received: " << _rxState.bytesReceivedThisEpoch
  //        << " + " << _numBytes
  //       << " = " << _rxState.bytesReceivedThisEpoch + _numBytes << std::endl;

  // Compute prospective accumulated bits along with time window
  // (including this packet).
  double bitsReceived = (_rxState.bytesReceivedThisEpoch + _numBytes) * 8;

  // Check current epoch bitrate vs capacity and fail to send accordingly.
  if (bitsReceived > this->radioConfig.capacity * this->epochDuration)
  {
    ignwarn << "Bitrate limited: [" << _rxState.name << "] " <<  bitsReceived
            << " bits received (limit: "
            << this->radioConfig.capacity * this->epochDuration << ")"
            << std::endl;
    return std::make_tuple(false, -std::numeric_limits<double>::max(), ber, packetDropProb, 0.0);
  }

  double throughput = this->PowerToThroughput(rxPower); // NICTのデータより算出

  // Record these bytes.

  _rxState.bytesReceived.push_back(std::make_pair(now, _numBytes));
  _rxState.bytesReceivedThisEpoch += _numBytes;

  return std::make_tuple(true, rxPower, ber, packetDropProb, throughput);
}

double RFComms_custom::Implementation::AttemptMeasure(
  RadioState &_txState, RadioState &_rxState)
{
  const double txAntennaGain = this->PoseToGain(_txState, _rxState);
  
  auto rxPowerDist =
  this->LogNormalReceivedPower(this->radioConfig.txPower + txAntennaGain, _txState, _rxState);

  // auto rxPowerDist =
  //   this->BasedOn3gppR16ReceivedPower(this->radioConfig.txPower, _txState, _rxState);

  double rxPower = rxPowerDist.mean;
  if (rxPowerDist.variance > 0.0)
  {
    std::normal_distribution<> d{rxPowerDist.mean, sqrt(rxPowerDist.variance)};
    rxPower = d(this->rndEngine);
  }

  const double rxAnntennaGain = this->PoseToGain(_rxState, _txState);

  return rxPower + rxAnntennaGain;
}

std::tuple<bool, double> RFComms_custom::Implementation::AttemptSend2(
  std::vector<double> &_SINR, const uint64_t &_numBytes)
{
  return std::make_tuple(true, 0.1);
}

double RFComms_custom::Implementation::RssiToSnr(
  int i, std::vector<double> _eachRssi)
{
  double noiseFloor= this->DbmToPow(this->radioConfig.thermalNoiseDensity)*this->radioConfig.capacity;

  double interferencePower = 0.0;
  for (auto &_otherRssi : _eachRssi)
  {
    if (_eachRssi[i] != _otherRssi)
      interferencePower += this->DbmToPow(_otherRssi);
  }

  igndbg << "interference_power[mW]: " << interferencePower << std::endl;
  igndbg << "noiseFloor[mW]: " << noiseFloor << std::endl;

  double SINR = 10*log10(this->DbmToPow(_eachRssi[i])/(noiseFloor + interferencePower));

  igndbg << "SINR[dB]: " << SINR << std::endl;

  return SINR;
}

//////////////////////////////////////////////////
RFComms_custom::RFComms_custom()
  : dataPtr(ignition::utils::MakeUniqueImpl<Implementation>())
{
}

//////////////////////////////////////////////////
void RFComms_custom::Load(const Entity &/*_entity*/,
    std::shared_ptr<const sdf::Element> _sdf,
    EntityComponentManager &/*_ecm*/,
    EventManager &/*_eventMgr*/)
{
  
  if (_sdf->HasElement("file_config"))
  {
    sdf::ElementPtr elem = _sdf->Clone()->GetElement("file_config");
    this->dataPtr->fileConfig.commsAnalysisDirPath = 
      elem->Get<std::string>("comms_analysis", this->dataPtr->fileConfig.commsAnalysisDirPath).first;
  }
  
  
  if (_sdf->HasElement("range_config"))
  {
    sdf::ElementPtr elem = _sdf->Clone()->GetElement("range_config");

    this->dataPtr->rangeConfig.maxRange =
      elem->Get<double>("max_range", this->dataPtr->rangeConfig.maxRange).first;

    this->dataPtr->rangeConfig.fadingExponent =
      elem->Get<double>("fading_exponent",
        this->dataPtr->rangeConfig.fadingExponent).first;

    this->dataPtr->rangeConfig.l0 =
      elem->Get<double>("l0", this->dataPtr->rangeConfig.l0).first;

    this->dataPtr->rangeConfig.sigma =
      elem->Get<double>("sigma", this->dataPtr->rangeConfig.sigma).first;
  }

  if (_sdf->HasElement("radio_config"))
  {
    sdf::ElementPtr elem = _sdf->Clone()->GetElement("radio_config");

    this->dataPtr->radioConfig.centerFrequency =
      elem->Get<double>("center_frequency", this->dataPtr->radioConfig.centerFrequency).first;
    
    this->dataPtr->radioConfig.capacity =
      elem->Get<double>("capacity", this->dataPtr->radioConfig.capacity).first;

    this->dataPtr->radioConfig.txPower =
      elem->Get<double>("tx_power", this->dataPtr->radioConfig.txPower).first;

    this->dataPtr->radioConfig.modulation =
      elem->Get<std::string>("modulation",
        this->dataPtr->radioConfig.modulation).first;

    this->dataPtr->radioConfig.thermalNoiseDensity =
      elem->Get<double>("thermal_noise_density",
        this->dataPtr->radioConfig.thermalNoiseDensity).first;

    this->dataPtr->radioConfig.antennaGainsDirPath =
      elem->Get<std::string>("antenna_gains_dir_path",
        this->dataPtr->radioConfig.antennaGainsDirPath).first;
    
    this->dataPtr->radioConfig.ePlane = 
      this->dataPtr->FileToArray(this->dataPtr->radioConfig.antennaGainsDirPath + "e_plane.csv");
    double maxGainEPlane = -std::numeric_limits<double>::infinity();
    for (const auto& row : this->dataPtr->radioConfig.ePlane)
    {
      maxGainEPlane = std::max(maxGainEPlane, row[1]);
    }

    this->dataPtr->radioConfig.hPlane = 
      this->dataPtr->FileToArray(this->dataPtr->radioConfig.antennaGainsDirPath + "h_plane.csv");
    double maxGainHPlane = -std::numeric_limits<double>::infinity();
    for (const auto& row : this->dataPtr->radioConfig.hPlane)
    {
      maxGainHPlane = std::max(maxGainHPlane, row[1]);
    }
  
    this->dataPtr->radioConfig.maxAntennaGain = (maxGainHPlane + maxGainEPlane)/2.; // 各面の最大値の平均 

    // this->dataPtr->radioConfig.txAntennaRot =
    //   elem->Get<ignition::math::Quaterniond>("tx_antenna_rot",
    //     this->dataPtr->radioConfig.txAntennaRot).first;

    // this->dataPtr->radioConfig.rxAntennaRot =
    //   elem->Get<ignition::math::Quaterniond>("rx_antenna_rot",
    //     this->dataPtr->radioConfig.rxAntennaRot).first;
        
  }

  // Generate  files
  std::string commsAnalysisFilePath = this->dataPtr->fileConfig.commsAnalysisDirPath + this->dataPtr->fileConfig.currentDateTime() + ".csv";
  this->dataPtr->writing_file.open(commsAnalysisFilePath, std::ios::out);
  this->dataPtr->writing_file << "X[m],Y[m],Z[m],From,RSSI[dBm],SINR[dB],Throughput[Gbps]" << std::endl;

  igndbg << "File configuration:" << std::endl
         << this->dataPtr->fileConfig << std::endl;

  igndbg << "Range configuration:" << std::endl
         << this->dataPtr->rangeConfig << std::endl;

  igndbg << "Radio configuration:" << std::endl
         << this->dataPtr->radioConfig << std::endl;

}

//////////////////////////////////////////////////
void RFComms_custom::Step(
      const UpdateInfo &_info,
      const comms::Registry &_currentRegistry,
      comms::Registry &_newRegistry,
      EntityComponentManager &_ecm)
{
  // Update ratio states.
  for (auto & [address, content] : _currentRegistry)
  {
    // Associate entity if needed.
    if (content.entity == kNullEntity)
    {
      auto entities = gazebo::entitiesFromScopedName(content.modelName, _ecm);
      if (entities.empty())
        continue;

      auto entityId = *(entities.begin());
      if (entityId == kNullEntity)
        continue;

      _newRegistry[address].entity = entityId;

      enableComponent<components::WorldPose>(_ecm, entityId);
    }
    else
    {
      // Update radio state.
      const auto kPose = gazebo::worldPose(content.entity, _ecm);
      this->dataPtr->radioStates[address].pose = kPose;
      this->dataPtr->radioStates[address].timeStamp =
        std::chrono::duration<double>(_info.simTime).count();
      this->dataPtr->radioStates[address].name = content.modelName;

      if (this->dataPtr->radioStates[address].name == "ugv")
      {
        this->dataPtr->radioStates[address].antennaRot = kPose.Rot() * this->dataPtr->radioConfig.rxAntennaRot;
      }
      else 
      {
        this->dataPtr->radioStates[address].antennaRot = kPose.Rot() * this->dataPtr->radioConfig.txAntennaRot;
      }
    }
  }

  int i = 0, j = 0;
  std::vector<double> eachRssi;
  for (auto & [address, content] : _currentRegistry)
  {
    // Reference to the outbound queue for this address.
    auto &outbound = content.outboundMsgs;
    // The source address needs to be attached to a robot.
    auto itSrc = this->dataPtr->radioStates.find(address);   
    if (itSrc != this->dataPtr->radioStates.end())
    {
      // All these messages need to be processed.
      for (const auto &msg : outbound)
      {
        // The destination address needs to be attached to a robot.
        auto itDst = this->dataPtr->radioStates.find(msg->dst_address());
        if (itDst == this->dataPtr->radioStates.end())
          continue;
        double rssi = this->dataPtr->AttemptMeasure(itSrc->second, itDst->second);
        eachRssi.push_back(rssi);
        ++i;
      }
    }
  }

  for (auto & [address, content] : _currentRegistry)
  {
    auto &outbound = content.outboundMsgs;
    auto itSrc = this->dataPtr->radioStates.find(address);   
    if (itSrc != this->dataPtr->radioStates.end())
    {
      for (const auto &msg : outbound)
      {
        auto itDst = this->dataPtr->radioStates.find(msg->dst_address());
        
        if (itDst == this->dataPtr->radioStates.end())
          continue;
        
        double SINR = this->dataPtr->RssiToSnr(j ,eachRssi);
        
        auto inboundMsg = std::make_shared<ignition::msgs::Dataframe>(*msg);

        // Add rssi and SINR.
        auto *commsAnalysisPtr = inboundMsg->mutable_header()->add_data();
        commsAnalysisPtr->set_key("RSSI/SINR: " + address);
        commsAnalysisPtr->add_value(std::to_string(eachRssi[j]) + "/" + std::to_string(SINR));
        
        _newRegistry[msg->dst_address()].inboundMsgs.push_back(inboundMsg);

        if (eachRssi[j] != -std::numeric_limits<double>::infinity())
        {
          ignition::math::Pose3 ugvPose = this->dataPtr->radioStates[msg->dst_address()].pose;
          this->dataPtr->writing_file << ugvPose.Pos().X() << ","
                                      << ugvPose.Pos().Y() << ","
                                      << ugvPose.Pos().Z() << ","
                                      << address << ","
                                      << eachRssi[j] << ","
                                      << SINR << std::endl;
        }
        ++j;
      }
    }
    // Clear the outbound queue.
    _newRegistry[address].outboundMsgs.clear();
  }
}

IGNITION_ADD_PLUGIN(RFComms_custom,
                    ignition::gazebo::System,
                    comms::ICommsModel::ISystemConfigure,
                    comms::ICommsModel::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(RFComms_custom,
                          "ignition::gazebo::systems::RFComms_custom")
