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
#include <cstdlib>

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

  /// \brief Communication duration from launching association.
  double associateDuration = 1000;

  /// \brief Threshold of rx power to launching setup.
  double thresholdPower = -65.5;
  
  /// \brief Max antenna gain calculated by 3D antenna patterns.
  double maxAntennaGain;

  /// \brief Max antenna attenuation.
  double maxAntennaAttenuation = 30.;

  /// \brief Delay from expired comms to launching new setup.
  double switchingDelay;

  /// \brief Interval of broadcasting beacons (s)
  double beaconInterval;

  /// \brief Interval of plotting (s)
  double plotInterval = 0.001;

  /// \brief Whether communication with AP (like ground station) is sequential. 
  bool isScheduled = false;

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

  /// \brief Pose of the radio.
  ignition::math::Vector3d antennaPos;

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

  /// \brief 
  double scheduledPlotTime = 0.;

  /// \brief Received Signal Strngth Indicator.
  std::unordered_map<std::string, double> timeReceivedBeacon;

  double scheduledBeaconTransmittionTime = 0.;
  
  std::string scheduledAssociateWith = "";

  int scheduledAssociationIndex = 0;

  /// \brief Received Signal Strngth Indicator.
  std::unordered_map<std::string, double> rssi;

  /// \brief Name of the model communicating.
  std::string sequentialSrcNode = "";

  /// \brief Name of the model communicating.
  std::string srcNode = "";

  /// \brief Time expired association.
  double timeout = 0.;

  /// \brief Whether communication is being setup.
  bool isBeingSetup = false;

  std::string currentSetupWith = "";

  /// \brief Time setup phase completed.
  double timeSetupCompleted = 0;

  /// \brief Whether communication was launched.
  bool isAssociated = false;

  std::string currentAssociateWith = "";

  /// \brief Whether communication was launched.
  double totalDataTransferred = 0.;

  /// \brief .
  std::vector<std::pair<double, double>> totalTimeItsLevel = {{-39., 0.},
                                               {-45., 0.},
                                               {-49., 0.},
                                               {-51., 0.},
                                               {-55., 0.},
                                               {-58., 0.},
                                               {-61., 0.}};

  double previousTimeStamp = 0.;

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

  public: double RssiToSinr(std::string address, RadioState _rxState);
  
  public: double MeasureRssi(RadioState &_txState,
                                RadioState &_rxState);

  /// \brief Convert from e_plane.csv and h_plane.csv to two dimensional array.
  public: std::vector<std::vector<double>> FileToArray(std::string _filePath) const;

  private: double PoseToGain(const RadioState &_initialPointState,
                             const RadioState &_terminalPointState,
                             const ignition::math::Quaterniond &_currentAntennaRot) const;

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

  private: double SinrToBer(double _sinr,
                             std::string _modulation) const;
  
  

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

  /// \brief Convert RSSI to throuput based on examination NICT operated.
  /// \param[in] _rssi received signal strength indicator（RSSI）.
  public: double RssiToThrouputInGbps(double _rssi) const;

  public: std::pair<int, double> RssiToTotalTimeItsLevel(double _rssi, double _timestepInSeconds, std::vector<std::pair<double, double>> &_totalTimeItsLevel) const;

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
    ignerr << "Cannot read " << _filePath << std::endl;
  
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
// double RFComms_custom::Implementation::DbmToPow(double _dBm) const
// {
//   return 0.001 * pow(10., _dBm / 10.);
// }

double RFComms_custom::Implementation::DbmToPow(double _dBm) const
{
  return pow(10., _dBm / 10.);
}


///////////////////////////////////////////// 
double RFComms_custom::Implementation::PoseToGain(
  const RadioState &_initialPointState,
  const RadioState &_terminalPointState,
  const ignition::math::Quaterniond &_currentAntennaRot) const
{

  // (送信側/受信側)から見た(受信側/送信側)のベクトル(_terminalPointState.pose.Pos() - _initialPointState.pose.Pos())を \
  (送信側/受信側)アンテナの回転だけ逆回転させる(_currentAntennaRot.RotateVectorReverse)ことで \
  実質的に(送信側/受信側)アンテナから見た(受信側/送信側)のベクトルを算出する

  // 例えば、送信側が(x y z)(r p y)=(0 0 0)(0 0 0)でアンテナ角が(r p y)=(0 0 1/4*pi) \
  受信側が(x y z)(r p y)=(3 3 0)(0 0 0)でアンテナ角が(r p y)=(0 0 -3/4*pi)の場合 \
  得られるベクトル direction は送信側/受信側でいずれも(x y z)=(3√2 0 0)となる

  auto direction = _currentAntennaRot.RotateVectorReverse(_terminalPointState.pose.Pos() - _initialPointState.pose.Pos());
  
  // igndbg << "direction: " << direction << " [" << _initialPointState.name << "]" << std::endl;

  // E面における(放射角/到来角）、thetaを空間ベクトルの内積の公式より算出する
  auto eDirection = direction;
  eDirection.Y(0.0);

  double theta = round((this->RadianToDegree(acos(eDirection.Normalized().Dot(ignition::math::Vector3d::UnitX))))*10.)/10.;
  if (eDirection.Z() < 0.0)
  {
    theta = -theta;
  }

  // 算出したthetaをE面のアンテナパターンにあてはめる
  auto itEPlane = std::find_if(
    std::begin(this->radioConfig.ePlane), std::end(this->radioConfig.ePlane),
    [&](const auto& row) {
      return row.at(0) == theta;
    }
  );

  // H面における(放射角/到来角）、varphiを空間ベクトルの内積の公式より算出する
  auto hDirection = direction;
  hDirection.Z(0.0);

  double varphi = round(this->RadianToDegree(acos(hDirection.Normalized().Dot(ignition::math::Vector3d::UnitX)))*10.)/10.;
  if (hDirection.Y() < 0.0)
  {
    varphi = -varphi;
  }

  // 算出したvarphiをH面のアンテナパターンにあてはめる
  auto itHPlane = std::find_if(
    std::begin(this->radioConfig.hPlane), std::end(this->radioConfig.hPlane),
    [&](const auto& row) {
      return row.at(0) == varphi;
    }
  );

  // igndbg << "[" << _initialPointState.name << "]" << std::endl;
  // igndbg << "theta: " << theta << " varphi: " << varphi << std::endl;

  // いずれかでもあてはまらなかった場合、アンテナ利得は-infになる
  if (itEPlane == std::end(this->radioConfig.ePlane) || itHPlane == std::end(this->radioConfig.hPlane)) 
    return -std::numeric_limits<double>::infinity();

  // アンテナ素子の指向性利得　= アンテナ素子の最大指向性利得(dBi) - E面のアンテナ素子の放射パターンによる減衰(dB)  - H面のアンテナ素子の放射パターンによる減衰(dB)で算出する \
  プログラム上はアンテナ素子の最大指向性利得(dBi) - E面のアンテナ素子の放射パターンによる減衰(dB) + アンテナ素子の最大指向性利得(dBi) - H面のアンテナ素子の放射パターンによる減衰(dB) - アンテナ素子の最大指向性利得(dBi)
  double antennaGainEPlane = this->radioConfig.ePlane[std::distance(std::begin(this->radioConfig.ePlane), itEPlane)][1];
  double antennaAttenuationEPlane = this->radioConfig.maxAntennaGain - antennaGainEPlane;
  if (antennaAttenuationEPlane > this->radioConfig.maxAntennaAttenuation)
  {
    antennaAttenuationEPlane = this->radioConfig.maxAntennaAttenuation;
    // ignwarn << "E面で下限値を超えました" << std::endl;
  }
  
  double antennaGainHPlane = this->radioConfig.hPlane[std::distance(std::begin(this->radioConfig.hPlane), itHPlane)][1];
  double antennaAttenuationHPlane = this->radioConfig.maxAntennaGain - antennaGainEPlane;
  if (antennaAttenuationHPlane > this->radioConfig.maxAntennaAttenuation)
  {
    antennaAttenuationHPlane = this->radioConfig.maxAntennaAttenuation;
    // ignwarn << "H面で下限値を超えました" << std::endl;
  }

  double antennaGain = this->radioConfig.maxAntennaGain - antennaAttenuationEPlane -antennaAttenuationHPlane;

  double minAntennaGain = this->radioConfig.maxAntennaGain - this->radioConfig.maxAntennaAttenuation;

  if (antennaGain < minAntennaGain)
  {
    antennaGain = minAntennaGain;
    // ignwarn << "3Dで下限値を超えました" << std::endl;
  }

  // igndbg << "AntennaGain(E-Plane): " << antennaGainEPlane << std::endl;
  // igndbg << "AntennaGain(H-Plane): " << antennaGainHPlane << std::endl;
  // igndbg << "AntennaGain3d: " << antennaGain << std::endl;
                      
  return antennaGain;
}

/////////////////////////////////////////////
RFPower RFComms_custom::Implementation::FreeSpaceReceivedPower(
  const double &_txPower, const RadioState &_txState,
  const RadioState &_rxState) const
{
  const double speedOfLight = 3.0 * pow(10.0, 8.0);
  const double kRange = _txState.pose.Pos().Distance(_rxState.pose.Pos());

  if (this->rangeConfig.maxRange > 0.0 && kRange > this->rangeConfig.maxRange) // 通信可能範囲外の場合
    return {-std::numeric_limits<double>::infinity(), 0.0};

  const double kPL = 20. * log10(4. * M_PI * kRange * this->radioConfig.centerFrequency/speedOfLight);

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

double RFComms_custom::Implementation::MeasureRssi(
  RadioState &_txState, RadioState &_rxState)
{
  ignition::math::Quaterniond txCurrentAntennaRot = _txState.antennaRot*this->radioConfig.txAntennaRot;

  ignition::math::Quaterniond rxCurrentAntennaRot = _rxState.antennaRot*this->radioConfig.rxAntennaRot;
  
  const double txAntennaGain = this->PoseToGain(_txState, _rxState, txCurrentAntennaRot);
  // const double txAntennaGain = 0;
  
  auto rxPowerDist =
  this->LogNormalReceivedPower(this->radioConfig.txPower + txAntennaGain, _txState, _rxState);

  double rxPower = rxPowerDist.mean;
  // igndbg << "rxPower: " << rxPower << std::endl;
  if (rxPowerDist.variance > 0.0)
  {
    std::normal_distribution<> d{rxPowerDist.mean, sqrt(rxPowerDist.variance)};
    rxPower = d(this->rndEngine);
  }

  const double rxAnntennaGain = this->PoseToGain(_rxState, _txState, rxCurrentAntennaRot);
  // const double rxAnntennaGain = 0;
  // igndbg << "rxAntennaGain: " << rxAnntennaGain << std::endl;

  return rxPower + rxAnntennaGain;
}

double RFComms_custom::Implementation::RssiToSinr(
  std::string _address, RadioState _rxState)
{
  double rssi;
  double interferencePower = 0.;
  for (const auto& pair : _rxState.rssi)
  {
    double pow = this->DbmToPow(pair.second);
    if (_address == pair.first)
      rssi = pow;
    else
      interferencePower += pow;
  }

  double noiseFloor= this->DbmToPow(this->radioConfig.thermalNoiseDensity)*this->radioConfig.capacity;

  // igndbg << _address << "'s rssi[mW]: " << rssi << std::endl;
  // igndbg << "interference_power[mW]: " << interferencePower << std::endl;
  // igndbg << "noiseFloor[mW]: " << noiseFloor << std::endl;

  return 10*log10(rssi/(interferencePower + noiseFloor));
}

////////////////////////////////////////////
double RFComms_custom::Implementation::QPSKPowerToBER(
  double _power, 
  double _noise) const
{
  return erfc(sqrt(_power / _noise)); // これあってる？？
}

double RFComms_custom::Implementation::SinrToBer(
  double _sinr,
  std::string _modulation) const
{ 
  if (_modulation.compare("QPSK") == 0)
    return 0.5 * erfc(sqrt(_sinr/2.0));
  if (_modulation.compare("16QAM"))
    return 0.375 * erfc(sqrt(_sinr/10.0)); 
  if (_modulation.compare("64QAM"))
    return (sqrt(64.0) - 1.0)/(log2(64.0) * pow(2.0, log2(64.0)/2.0 - 1.0)) * erfc(sqrt(_sinr * (3.0/(2.0 * (64.0 - 1.0)))));
  if (_modulation.compare("256QAM"))
    return (sqrt(256.0) - 1.0)/(log2(256.0) * pow(2.0, log2(256.0)/2.0 - 1.0)) * erfc(sqrt(_sinr * (3.0/(2.0 * (256.0 - 1.0)))));
  return std::numeric_limits<double>::lowest();
}

// NICTから提供いただいたテーブルを元に作成
double RFComms_custom::Implementation::RssiToThrouputInGbps(
  double _rssi) const
{ 
  if (_rssi > -51.0)
    return 6.0;
  else if (_rssi > -55.0)
    return 4.7;
  else if (_rssi > -58.5)
    return 2.7;
  else if (_rssi > -61.5)
    return 2.15;
  else if (_rssi > -63.5)
    return 1.1;
  else if (_rssi > -65.5)
    return 0.5;
  else 
    return 0.0;
}

std::pair<int, double> RFComms_custom::Implementation::RssiToTotalTimeItsLevel(
  double _rssi,
  double _timestepInSeconds,
  std::vector<std::pair<double, double>> &_totalTimeItsLevel) const
{
  int index = _totalTimeItsLevel.size();
  for (auto &pair: _totalTimeItsLevel)
  {
    index += -1;
    ignwarn << pair.first << std::endl;
    if (_rssi > pair.first)
    {
      pair.second += _timestepInSeconds;
      ignwarn << index << ": " << pair.second << " s" << std::endl;
      return std::make_pair(index, pair.second);
    }
  }
  return std::make_pair(-1, 0.);
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
    
    this->dataPtr->radioConfig.associateDuration = 
      elem->Get<double>("associate_duration",
        this->dataPtr->radioConfig.associateDuration).first;

    this->dataPtr->radioConfig.thresholdPower =
      elem->Get<double>("threshold_power",
          this->dataPtr->radioConfig.thresholdPower).first;
    
    this->dataPtr->radioConfig.beaconInterval =
      elem->Get<double>("beacon_interval",
          this->dataPtr->radioConfig.beaconInterval).first;

    this->dataPtr->radioConfig.plotInterval =
      elem->Get<double>("plot_interval",
          this->dataPtr->radioConfig.plotInterval).first;
    
    if (this->dataPtr->radioConfig.plotInterval < this->dataPtr->radioConfig.beaconInterval)
    {
      ignwarn << "Plot interval (" << this->dataPtr->radioConfig.plotInterval << ") is changed to beacon interval (" <<  this->dataPtr->radioConfig.beaconInterval << ")";
      this->dataPtr->radioConfig.plotInterval = this->dataPtr->radioConfig.beaconInterval; 
    }
      
    this->dataPtr->radioConfig.switchingDelay =
      elem->Get<double>("switching_delay",
          this->dataPtr->radioConfig.switchingDelay).first;

    this->dataPtr->radioConfig.isScheduled =
      elem->Get<bool>("is_scheduled",
          this->dataPtr->radioConfig.isScheduled).first;
    
    igndbg << "isScheduled: " << this->dataPtr->radioConfig.isScheduled << std::endl;
    
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

    // 最大のアンテナ利得は各面の最大利得を足して2で割ったものとする
    this->dataPtr->radioConfig.maxAntennaGain = (maxGainHPlane + maxGainEPlane)/2.; // 各面の最大値の平均

    // 減衰の下限値
    // this->dataPtr->radioConfig.maxAntennaAttenuation = 30.

  }


  // Generate  files
  std::string commsAnalysisFilePath = this->dataPtr->fileConfig.commsAnalysisDirPath + this->dataPtr->fileConfig.currentDateTime() + ".csv";
  this->dataPtr->writing_file.open(commsAnalysisFilePath, std::ios::out);
  // this->dataPtr->writing_file << "Time[s],X[m],Y[m],Z[m],From,RSSI[dBm],SINR[dB],Throughput[Gbps],TotalDataTransferred[GB]";
  this->dataPtr->writing_file << "Time[s],DstName,DstX[m],DstY[m],DstZ[m],SrcName,SrcX[m],SrcY[m],SrcZ[m],RSSI[dBm],SINR[dB],-39dBm,-45dBm,-49dBm,-51dBm,-55dBm,-58dBm,-61dBm";

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
  // 1ステップ時間を取得する（結果の出力はpublisher.ccの送信間隔にも依存するが、この送信間隔が1ステップ時間より短くなると処理が追いつかなくなるので注意）
  std::chrono::steady_clock::duration timestep = _info.dt;
  double timestepInSeconds = std::chrono::duration<double>(timestep).count();
  if (timestepInSeconds > this->dataPtr->radioConfig.beaconInterval)
    ignerr << "Timestep is longer than beacon interval." << std::endl;

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
      this->dataPtr->radioStates[address].antennaPos = kPose.Pos();
      this->dataPtr->radioStates[address].antennaRot = kPose.Rot();
      this->dataPtr->radioStates[address].timeStamp =
        std::chrono::duration<double>(_info.simTime).count();
      this->dataPtr->radioStates[address].name = content.modelName;
    }
  }

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
        std::string srcAddress = itSrc->first;
        std::string srcAddressBase = srcAddress.substr(0, srcAddress.length()-1);
        
        // The destination address needs to be attached to a robot.
        auto itDst = this->dataPtr->radioStates.find(msg->dst_address());
        if (itDst == this->dataPtr->radioStates.end())
          continue;
        
        std::string dstAddress = itDst->first;
        std::string dstAddressBase = dstAddress.substr(0, dstAddress.length()-1);

        itSrc->second.scheduledAssociateWith = dstAddressBase + std::to_string(itSrc->second.scheduledAssociationIndex);
        itDst->second.scheduledAssociateWith = srcAddressBase + std::to_string(itDst->second.scheduledAssociationIndex);

        // auto itReceivedBeacon = itDst->second.timeReceivedBeacon.find(srcAddress);
        // if (itReceivedBeacon == itDst->second.timeReceivedBeacon.end())
        // {
        //   itDst->second.timeReceivedBeacon.insert_or_assign(srcAddress, 0.);
        //   itDst->second.sequentialSrcNode = srcAddressBase + std::to_string(0);
        // } 
        // else 
        // {
        //   // setup及びassociatedフェーズではない場合
        //   if (itDst->second.srcNode == "")
        //   {
        //     igndbg << "timeStamp/timeBeaconReceived[" << srcAddress << "]: " << itDst->second.timeStamp << "/" << itReceivedBeacon->second << std::endl;
        //     // タイムスタンプがビーコン受信時間以上の場合
        //     if (itDst->second.timeStamp >= itReceivedBeacon->second)
        //       // 次のビーコン受信時間を設定して以降の処理を継続する
        //       itReceivedBeacon->second = itDst->second.timeStamp + this->dataPtr->radioConfig.beaconInterval;
        //     else
        //       // 以降の処理をスキップする
        //       continue;
        //   }
        // }

        bool isSrcBusy = itSrc->second.currentSetupWith != "" && itSrc->second.currentSetupWith != dstAddress ||
                         itSrc->second.currentAssociateWith != "" && itSrc->second.currentAssociateWith != dstAddress;
        
        bool isDstBusy = itDst->second.currentSetupWith != "" && itDst->second.currentSetupWith != srcAddress ||
                         itDst->second.currentAssociateWith != "" && itDst->second.currentAssociateWith != srcAddress;
        
        if (isSrcBusy || isDstBusy)
          continue;
      
        // 
        bool isSetup = itSrc->second.currentSetupWith == dstAddress && itDst->second.currentSetupWith == srcAddress;
        if (isSetup)
        {
          igndbg << "Setup time: " << srcAddress << " <=== " << itSrc->second.timeStamp << "/" << itSrc->second.timeSetupCompleted << " ===> " << dstAddress << std::endl;
        }

        // 
        bool isAssociated = itSrc->second.currentAssociateWith == dstAddress && itDst->second.currentAssociateWith == srcAddress;
        if (isAssociated)
        {
          igndbg << "Associated time: " << srcAddress << " <=== " << itSrc->second.timeStamp << "/" << itSrc->second.timeout << " ===> " << dstAddress << std::endl;
        }
        
        // 
        if (!isSetup && !isAssociated)
        {
          igndbg << "Beacon transmmit time: " << srcAddress << " <=== " << itSrc->second.timeStamp << "/" << itSrc->second.scheduledBeaconTransmittionTime << " ===> " << dstAddress << std::endl;
          if (itSrc->second.timeStamp >= itSrc->second.scheduledBeaconTransmittionTime)
            itSrc->second.scheduledBeaconTransmittionTime = itSrc->second.timeStamp + this->dataPtr->radioConfig.beaconInterval;
          else
            continue;
        }

        // RSSIの測定
        double rssi = this->dataPtr->MeasureRssi(itSrc->second, itDst->second);
        // bool isCommsAnalysisPlottableInRssi = std::isfinite(rssi);
        // if (itDst->second.srcNode == srcAddress || itDst->second.isBeingSetup == false
        igndbg << "RSSI: " << srcAddress << " <===> " << dstAddress << " -> " << rssi << " dBm" << std::endl;


        itDst->second.rssi.insert_or_assign(srcAddress, rssi);

        // 指定先のファイルにモビリティの座標を描画する
        // bool isCommsAnalysisPlottableInSrcNode = itDst->second.srcNode == "" && itDst->second.sequentialSrcNode == srcAddress ||
        //                    itDst->second.srcNode != "" && itDst->second.srcNode == srcAddress;
          
        bool isCommsAnalysisPlottable = (itDst->second.scheduledAssociateWith == srcAddress || itDst->second.currentAssociateWith == srcAddress) &&
                                        itDst->second.timeStamp > itDst->second.scheduledPlotTime;
        if (isCommsAnalysisPlottable)
        {
          itDst->second.scheduledPlotTime += this->dataPtr->radioConfig.plotInterval; 
             
          ignition::math::Pose3 dstPose = itDst->second.pose;
          this->dataPtr->writing_file << std::endl 
                                      << itDst->second.timeStamp << ","
                                      << dstAddress << ","
                                      << dstPose.Pos().X() << ","
                                      << dstPose.Pos().Y() << ","
                                      << dstPose.Pos().Z();
        }

        // Disassociationを行う
        bool isDisassociation = isAssociated && (rssi < this->dataPtr->radioConfig.thresholdPower || itSrc->second.timeStamp > itSrc->second.timeout);
        if (isDisassociation)
        {
          itSrc->second.currentAssociateWith = "";
          itDst->second.currentAssociateWith = "";
          ignwarn << srcAddress << " was disassociated with " << dstAddress << " at " << itSrc->second.timeStamp << std::endl;
        
          if (this->dataPtr->radioConfig.isScheduled)
          {
            itSrc->second.scheduledAssociationIndex += 1;
            auto itScheduledSrc = this->dataPtr->radioStates.find(dstAddressBase + std::to_string(itSrc->second.scheduledAssociationIndex));
            if (itScheduledSrc == this->dataPtr->radioStates.end())
              itSrc->second.scheduledAssociationIndex = 0;
            ignwarn << srcAddress << " will associate with " << dstAddressBase << itSrc->second.scheduledAssociationIndex << std::endl;

            itDst->second.scheduledAssociationIndex += 1;
            auto itScheduledDst = this->dataPtr->radioStates.find(srcAddressBase + std::to_string(itDst->second.scheduledAssociationIndex));
            if (itScheduledDst == this->dataPtr->radioStates.end())
              itDst->second.scheduledAssociationIndex = 0;
            ignwarn << dstAddress << " will associate with " << srcAddressBase << itDst->second.scheduledAssociationIndex << std::endl;
            }
        }

        // associatedフェーズで、タイムアウトしている場合、通信を終了する
        // if (itDst->second.isAssociated && itDst->second.srcNode == srcAddress && itDst->second.timeStamp > itDst->second.timeout)
        // {
        //   itDst->second.srcNode = ""; // リンク先の設定を解除する
        //   itDst->second.isAssociated = false; // 通信を終了する
        //   ignwarn << srcAddress << " <===> " << dstAddress << " was expired." << std::endl;

        //   // 0, 1, 2,...と接続先を順番に切り替える場合、次の接続先を指定する
        //   if (this->dataPtr->radioConfig.isScheduled)
        //     itDst->second.sequentialSrcNode = srcAddressBase + std::to_string(std::atoi(&itDst->second.sequentialSrcNode.back()) + 1);
        // }

        // RSSIが閾値以上かつ他の地上局とリンクしていない場合、setupフェーズに移行する
        // if (itDst->second.isBeingSetup == false && itDst->second.srcNode == "" && rssi > this->dataPtr->radioConfig.thresholdPower)
        // {
        //   // 0, 1, 2,...と接続先を順番に切り替える場合、次に接続するAPではない限り、以降の処理を行わない
        //   if (this->dataPtr->radioConfig.isScheduled && srcAddress != itDst->second.sequentialSrcNode)
        //     continue;

        //   itDst->second.srcNode = srcAddress; // リンク先を設定する
        //   itDst->second.timeSetupCompleted = itDst->second.timeStamp + this->dataPtr->radioConfig.switchingDelay; // setupが完了するまでの時間を設定する
        //   itDst->second.isBeingSetup = true; // setupフェーズのフラグを立てる
        //   ignwarn << srcAddress << " <===> " << dstAddress << " is being setup." << std::endl;
        // }

        bool isAssociationRequestable = !isSrcBusy &&
                                        !isDstBusy &&
                                        !isAssociated && 
                                        !isSetup &&
                                        (!this->dataPtr->radioConfig.isScheduled || itSrc->second.scheduledAssociateWith == dstAddress) &&
                                        (!this->dataPtr->radioConfig.isScheduled || itDst->second.scheduledAssociateWith == srcAddress) &&
                                        rssi > this->dataPtr->radioConfig.thresholdPower;
        if (isAssociationRequestable)
        {
          itSrc->second.currentSetupWith = dstAddress;
          itDst->second.currentSetupWith = srcAddress;
          itSrc->second.timeSetupCompleted = itSrc->second.timeStamp + this->dataPtr->radioConfig.switchingDelay;
          ignwarn << srcAddress << " is being setup with " << dstAddress << " from " << itSrc->second.timeStamp << " to " << itSrc->second.timeSetupCompleted << std::endl;
        }
        
        // 設定したリンク先と一致し、setupフェーズが終了した場合、associatedフェーズに移行する
        // if (itDst->second.isBeingSetup && itDst->second.srcNode == srcAddress && itDst->second.timeStamp > itDst->second.timeSetupCompleted)
        // {
        //   itDst->second.timeout = itDst->second.timeStamp + this->dataPtr->radioConfig.associateDuration; // タイムアウトを設定
        //   itDst->second.isAssociated = true;
        //   itDst->second.isBeingSetup = false;
        //   ignwarn << srcAddress << " <===> " << dstAddress << " was setup." << std::endl;
        // }

        bool isAssociationRespondable = isSetup && itSrc->second.timeStamp > itSrc->second.timeSetupCompleted;
        if (isAssociationRespondable)
        {
          itSrc->second.currentSetupWith = "";
          itDst->second.currentSetupWith = "";

          itSrc->second.currentAssociateWith = dstAddress;
          itDst->second.currentAssociateWith = srcAddress;

          itSrc->second.timeout = itSrc->second.timeStamp + this->dataPtr->radioConfig.associateDuration;
          ignwarn << srcAddress << " will be associated with " << dstAddress << " from " << itSrc->second.timeStamp << " to " << itSrc->second.timeout << std::endl;
        }

        // 設定したリンク先と一致し、associatedフェーズの場合、SINRを算出して結果を出力する
        // if (itDst->second.isAssociated && itDst->second.srcNode == srcAddress)
        // {
        //   igndbg << "timeStamp/timeout[" << itDst->second.srcNode << "]: " << itDst->second.timeStamp << "/" << itDst->second.timeout << std::endl;

        //   // SINRの算出
        //   double sinr = this->dataPtr->RssiToSinr(srcAddress, itDst->second);
        //   igndbg << "SINR[" << srcAddress << "][dB]: " << sinr << std::endl;

        //   // Throughputの算出
        //   // double throughputInGbps = this->dataPtr->RssiToThrouputInGbps(rssi);
        //   // igndbg << "Throuput[" << srcAddress << "][Gbps]: " << throughputInGbps << std::endl;

        //   // 累積データ転送量の算出
        //   // double dataTransferredInTimestep = throughputInGbps*timestepInSeconds;
        //   // itDst->second.totalDataTransferred += dataTransferredInTimestep;
        //   // igndbg << "Total data transferred: " << itDst->second.totalDataTransferred << std::endl;

        //   double timeStampDuration = itDst->second.timeStamp - itDst->second.previousTimeStamp; // publishの間隔が短くなるほど小さくなる
        //   for (auto &pair: itDst->second.totalTimeItsLevel)
        //   {
        //     if (rssi > pair.first)
        //     {
        //       pair.second += timeStampDuration;
        //       break;
        //     }
        //   }


        //   for (auto &pair: itDst->second.totalTimeItsLevel)
        //   {
        //     ignwarn << rssi << "dBm -> " << pair.first << " dBm(level) -> " << pair.second << " s" <<std::endl; 
        //   }

        //   auto inboundMsg = std::make_shared<ignition::msgs::Dataframe>(*msg);
        //   auto *commsAnalysisPtr = inboundMsg->mutable_header()->add_data();
        //   commsAnalysisPtr->set_key("RSSI/SINR: " + address);
        //   commsAnalysisPtr->add_value(std::to_string(rssi) + "/" + std::to_string(sinr));
        //   _newRegistry[msg->dst_address()].inboundMsgs.push_back(inboundMsg);

        //   if (isCommsAnalysisPlottable)
        //   {
        //     this->dataPtr->writing_file << "," 
        //                                 << srcAddress << ","
        //                                 << rssi << ","
        //                                 << sinr << ",";
        //     for (auto &pair: itDst->second.totalTimeItsLevel)
        //     {
        //       this->dataPtr->writing_file << pair.second << ",";
        //     }
        //   }
        // }

        if (isAssociated && std::isfinite(rssi))
        {
          double timeStampDuration = itDst->second.timeStamp - itDst->second.previousTimeStamp; // publishの間隔が短くなるほど小さくなる
          
          // SINRの算出
          double sinr = this->dataPtr->RssiToSinr(srcAddress, itDst->second);
          igndbg << "SINR: " << srcAddress << " <===> " << dstAddress << " -> " << sinr << " dB" << std::endl;

          // Throughputの算出
          double throughputInGbps = this->dataPtr->RssiToThrouputInGbps(rssi);
          igndbg << "Throuput: " << srcAddress << " <===> " << dstAddress << " -> " << throughputInGbps << " Gbps" << std::endl;

          // 累積データ転送量の算出
          // double dataTransferredInTimestep = throughputInGbps*timeStampDuration;
          // itDst->second.totalDataTransferred += dataTransferredInTimestep;
          // igndbg << "Total data transferred: " << srcAddress << " <===> " << dstAddress << " -> " << itDst->second.totalDataTransferred << " Gb" << std::endl;

          for (auto &pair: itDst->second.totalTimeItsLevel)
          {
            if (rssi > pair.first)
            {
              pair.second += timeStampDuration;
              break;
            }
          }

          // 送信メッセージをターミナルに表示
          // auto inboundMsg = std::make_shared<ignition::msgs::Dataframe>(*msg);
          // auto *commsAnalysisPtr = inboundMsg->mutable_header()->add_data();
          // commsAnalysisPtr->set_key("RSSI/SINR: " + address);
          // commsAnalysisPtr->add_value(std::to_string(rssi) + "/" + std::to_string(sinr));
          // _newRegistry[msg->dst_address()].inboundMsgs.push_back(inboundMsg);

          if (isCommsAnalysisPlottable)
          {
            ignition::math::Pose3 srcPose = itSrc->second.pose;       
            this->dataPtr->writing_file << ","
                                        << srcAddress << ","
                                        << srcPose.Pos().X() << ","
                                        << srcPose.Pos().Y() << ","
                                        << srcPose.Pos().Z() << ","
                                        << rssi << ","
                                        << sinr << ",";
            for (auto &pair: itDst->second.totalTimeItsLevel)
            {
              this->dataPtr->writing_file << pair.second << ",";
            }
          }
        }
        
        // SetupフェーズまたはAssociatedフェーズのときに累積時間算出のためにpreviousTimeStampを算出
        bool isTimeStampRenewable = isSetup || isAssociated;
        if (isTimeStampRenewable)
          itDst->second.previousTimeStamp = itDst->second.timeStamp;
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
