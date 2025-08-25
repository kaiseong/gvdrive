// Copyright (c) 2025 Raion Robotics Inc.
//
// Any unauthorized copying, alteration, distribution, transmission,
// performance, display or use of this material is prohibited.
//
// All rights reserved.

#ifndef GVDrive_HPP_
#define GVDrive_HPP_

#include <mutex>
#include <string>
#include <atomic>
#include <thread>
#include <cmath>
#include "soem/soem.h"
#include <iostream>
#include <map>
#include <fstream>
#include <vector>
#include <array>
#include <sys/mman.h>
#include <time.h>
#include <errno.h>
#include "yaml-cpp/yaml.h"

namespace GVTerrain
{

namespace robot
{

struct Convert
{
  static constexpr std::string_view product = "GVdrive ILM115x25 Star Serial";
  static constexpr double peakMaxCurrent = 45;       // A
  static constexpr double contMaxCurrent = 20.0;       // A
  static constexpr double torqueScale = 1000.0;

  static constexpr double resolution = 524288; // pow(2,19) 
  static constexpr double ratedCurrent = 14.1;       // A
  static constexpr double torqueConst = 0.281;      // Nm/A 
  // gearRatio를 곱해야 토크상수임 0.281 * 7.15 = 2.00915
  static constexpr double ratedTorque = ratedCurrent * torqueConst;   // Nm
  static constexpr double gearRatio = 7.15;
  static constexpr double unitToDeg = 360.0 / resolution;
  static constexpr double unitToRad = 2 * M_PI / resolution;
  static constexpr double unitToJointPos = unitToRad / gearRatio;
  static constexpr double unitToMotorPos = 1 / unitToJointPos;
  static constexpr double unitToJointVel = unitToJointPos;
  static constexpr double unitToMotorVel = 1 / unitToJointVel;
  static constexpr double unitToJointToq = ratedTorque * gearRatio / torqueScale;
  static constexpr double unitToMotorToq = 1 / unitToJointToq;

  static constexpr double resolution_M = 65536; // pow(2,16) 
  static constexpr double ratedCurrent_M = 10.5;       // A
  static constexpr double torqueConst_M = 0.246;      // Nm/A 
  // gearRatio를 곱해야 토크상수임 0.246 * 10 = 2.46
  static constexpr double ratedTorque_M = ratedCurrent_M * torqueConst_M;   // Nm
  static constexpr double gearRatio_M = 10;
  static constexpr double unitToDeg_M = 360.0 / resolution_M;
  static constexpr double unitToRad_M = 2 * M_PI / resolution_M;
  static constexpr double unitToJointPos_M = unitToRad_M / gearRatio_M;
  static constexpr double unitToMotorPos_M = 1 / unitToJointPos_M;
  static constexpr double unitToJointVel_M = unitToJointPos_M;
  static constexpr double unitToMotorVel_M = 1 / unitToJointVel_M;
  static constexpr double unitToJointToq_M = ratedTorque_M * gearRatio_M / torqueScale;
  static constexpr double unitToMotorToq_M = 1 / unitToJointToq_M;

  static constexpr double resolution_HM = 65536; // pow(2,16) 
  static constexpr double ratedCurrent_HM = 10.5;       // A
  static constexpr double torqueConst_HM = 0.246;      // Nm/A 
  // gearRatio를 곱해야 토크상수임 0.246 * 22 = 
  static constexpr double ratedTorque_HM = ratedCurrent_HM * torqueConst_HM;   // Nm
  static constexpr double gearRatio_HM = 22;
  static constexpr double unitToDeg_HM = 360.0 / resolution_HM;
  static constexpr double unitToRad_HM = 2 * M_PI / resolution_HM;
  static constexpr double unitToJointPos_HM = unitToRad_HM / gearRatio_HM;
  static constexpr double unitToMotorPos_HM = 1 / unitToJointPos_HM;
  static constexpr double unitToJointVel_HM = unitToJointPos_HM;
  static constexpr double unitToMotorVel_HM = 1 / unitToJointVel_HM;
  static constexpr double unitToJointToq_HM = ratedTorque_HM * gearRatio_HM / torqueScale;
  static constexpr double unitToMotorToq_HM = 1 / unitToJointToq_HM;

  static constexpr double resolution_S = 65536; // pow(2,16) 
  static constexpr double ratedCurrent_S = 5.1;       // A
  static constexpr double torqueConst_S = 0.104;      // Nm/A 
  // gearRatio를 곱해야 토크상수임 0.104 * 7.33 = 
  static constexpr double ratedTorque_S = ratedCurrent_S * torqueConst_S;   // Nm
  static constexpr double gearRatio_S = 7.33;
  static constexpr double unitToDeg_S = 360.0 / resolution_S;
  static constexpr double unitToRad_S = 2 * M_PI / resolution_S;
  static constexpr double unitToJointPos_S = unitToRad_S / gearRatio_S;
  static constexpr double unitToMotorPos_S = 1 / unitToJointPos_S;
  static constexpr double unitToJointVel_S = unitToJointPos_S;
  static constexpr double unitToMotorVel_S = 1 / unitToJointVel_S;
  static constexpr double unitToJointToq_S = ratedTorque_S * gearRatio_S / torqueScale;
  static constexpr double unitToMotorToq_S = 1 / unitToJointToq_S;



  int32_t offset;
  int32_t PositionActualValue;
  int16_t TorqueActualValue;
  int32_t VelocityActualSensor;
  uint8_t Statusword;
  uint8_t Temperature;
  uint8_t ErrorCode;
};

struct __attribute__ ((packed)) RxPDO
{
  int32_t TargetPosition;
  int32_t TargetVelocity;
  int16_t TargetTorque;
  uint8_t ControlWord;
  uint8_t ModesofOperation;
  uint16_t ImpedancePGain;
  uint16_t ImpedanceDGain;
};

struct __attribute__ ((packed)) TxPDO
{
  int32_t PositionActualValue;
  int16_t TorqueActualValue;
  int32_t VelocityActualSensor;
  uint8_t Statusword;
  uint8_t Temperature;
  uint8_t ErrorCode;
};



enum OPMODE
{
  PWM_DUTY_ZERO = 0,
  IMPEDANCE_CONTROL = 7,
  CYCLIC_SYNC_POSITION = 8,
  CYCLIC_SYNC_VELOCITY = 9,
  CYCLIC_SYNC_TORQUE = 10,
  PWM_HALF_DUTY = 11,
  OPEN_LOOP_VOLTAGE_TEST = 12,
  ZERO_CURRENT_CONTROL_TEST = 13,
  OPEN_LOOP_CURRENT_TEST = 14,
  COMMUTATION = 15,
};

enum CiA402CONTROLWORD
{
  DISABLE_VOLTAGE = 0,
  ENABLE_OPERATION = 15,
  FAULT_CLEAR = 128
};

enum CiA402StatusWord : int
{
  NOT_READY_TO_SWITCH_ON = 0,
  READY_TO_SWITCH_ON = 33,
  SWITCHED_ON = 35,
  OPERATION_ENABLED = 39
};

enum UserMode
{
  CALIBRATION = 0,
  CONTROL = 1
};

enum ConnectMode
{
  NOBOARD = 0,
  RAIHUB = 1
};

enum ControlMode
{
  POSITION = 0,//대형구동기
  VELOCITY = 1,
  TORQUE = 2,
  IMPEDANCE = 3,

  POSITION_M = 4,//중형구동기
  VELOCITY_M = 5,
  TORQUE_M = 6,
  IMPEDANCE_M = 7,

  POSITION_HM = 8,//고감속중형구동기
  VELOCITY_HM = 9,
  TORQUE_HM = 10,
  IMPEDANCE_HM = 11,

  POSITION_S = 12,//소형구동기
  VELOCITY_S = 13,
  TORQUE_S = 14,
  IMPEDANCE_S = 15
};

struct Motor
{
  std::string name;
  int slaveIndex;
  ControlMode control_mode;
  double initialposition = 0.0;
  double targetTorque = 0.0;
  double targetPosition = 0.0;
  double targetVelocity = 0.0;
  double Position = 0.0;
  double Torque = 0.0;
  double Velocity = 0.0;
  uint8_t status = 0;
  uint8_t temperature = 0;
  uint8_t errorCode = 0;
  uint16_t ImpedancePGain = 0;
  uint16_t ImpedanceDGain = 0;
};


class GVdrive
{
public:
  explicit GVdrive(const std::string& interface, const std::string& config_path);
  
  ~GVdrive();

  bool scan();
  bool con();
  void setControlMode(UserMode new_mode);
  void setConnectMode(ConnectMode new_mode);
  // I/O
  void setTargetPosition(const std::string& motor_name, double position);
  void setTargetTorque(const std::string& motor_name, double torque);
  void setTargetVelocity(const std::string& motor_name, double velocity);
  double getPosition(const std::string& motor_name);
  double getVelocity(const std::string& motor_name);
  double getTorque(const std::string& motor_name);
  // bool loop(double frequency);
  void* loop(double frequency);
  bool startEthercat();
  bool shutdown();
  
protected:
  static void* loopFunc(void* arg);

  pthread_t etherCatThread; 
  // std::thread etherCatThread;
  std::mutex driveMutex;
  std::string selectedInterface;
  

  /// counter
  uint8_t loop_cnt = 0;

  /// ethercat
  char IOmap_[4096];
  bool ready = false;

  uint8_t start_cnt=3;
  std::atomic_bool runEtherCatLoop = true;
  std::vector<Motor> motors;
  std::map<std::string, int> motor_map_;

  robot::OPMODE ModesofOperation;
  robot::CiA402CONTROLWORD controlWord = robot::CiA402CONTROLWORD::DISABLE_VOLTAGE;
  UserMode control_mode;
  ConnectMode connect_mode;
  uint32_t clock_count=0;
  double loopFrequency=1000.0;
  double initialposition=0.0;
  void control();
  void calibration();
  bool load_config(const std::string& config_path);
};

}
}
#endif  // GVDrive_HPP_