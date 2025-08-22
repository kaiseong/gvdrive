
#include "GVdrive.hpp"
#include <sstream>
#include <pthread.h>
#include <sched.h>

#define CPU_CORE 3


namespace GVTerrain::robot {
  

  GVdrive::GVdrive(const std::string& interface, const std::string& config_path):selectedInterface(interface)
  {
    if (!load_config(config_path)) {
      throw std::runtime_error("Failed to load or motor configuration file");
    }
  }

  bool GVdrive::load_config(const std::string& config_path){
    try{
      YAML::Node config =YAML::LoadFile(config_path);
      const auto& motors_node = config["motors"];
      if (!motors_node){
        std::cerr <<"yaml file doesn't contain 'motors' key"<<std::endl;
        return false;
      }
      motors.clear();
      motor_map_.clear();
      for (const auto& node : motors_node) {
        Motor m;
        m.name = node["name"].as<std::string>();
        m.slaveIndex = node["slave_index"].as<int>();
        m.ImpedancePGain = node["p_gain"].as<uint16_t>();
        m.ImpedanceDGain = node["d_gain"].as<uint16_t>();
        std::string mode_str = node["control_mode"].as<std::string>();
        if (mode_str == "torque") {
          m.control_mode = TORQUE;
        }
        else if (mode_str == "impedance"){
          m.control_mode = IMPEDANCE;        
        }
        else if (mode_str == "position") {
        m.control_mode = POSITION;
        } 
        else if (mode_str == "velocity") {
        m.control_mode = VELOCITY;
        } 

        else if (mode_str == "torque_m") {
          m.control_mode = TORQUE_M;
        }
        else if (mode_str == "impedance_m"){
          m.control_mode = IMPEDANCE_M;        
        }
        else if (mode_str == "position_m") {
        m.control_mode = POSITION_M;
        } 
        else if (mode_str == "velocity_m") {
        m.control_mode = VELOCITY_M;
        } 

        else if (mode_str == "torque_hm") {
          m.control_mode = TORQUE_HM;
        }
        else if (mode_str == "impedance_hm"){
          m.control_mode = IMPEDANCE_HM;        
        }
        else if (mode_str == "position_hm") {
        m.control_mode = POSITION_HM;
        } 
        else if (mode_str == "velocity_hm") {
        m.control_mode = VELOCITY_HM;
        } 

        else if (mode_str == "torque_s") {
          m.control_mode = TORQUE_S;
        }
        else if (mode_str == "impedance_s"){
          m.control_mode = IMPEDANCE_S;        
        }
        else if (mode_str == "position_s") {
        m.control_mode = POSITION_S;
        } 
        else if (mode_str == "velocity_s") {
        m.control_mode = VELOCITY_S;
        } 

        else {
        m.control_mode = TORQUE;
        std::cerr << "Warning: Unknown control mode '" << mode_str << std::endl;
    }

        motors.push_back(m);
        motor_map_[m.name] = motors.size() - 1;
      }
      std::cout << "Load Sueccess" << motors.size() << "motors from " << config_path << std::endl;
      return true;
    }catch(const YAML::Exception& e){
      std::cerr << "Error while parsing YAML file: "<< e.what() << std::endl;
      return false;
    }
    return false;
  }
  
  bool GVdrive::scan() {
    if (ec_init(selectedInterface.c_str())) {
      slaveCount = ec_config_init(false);
      std::cout<<"initialized ethercat"<<std::endl;      
    } else {
      std::cerr << "ethercat initialization failed"<<std::endl;
      return false;
    }
    return true;
  }

  bool GVdrive::con() {
    if(!startEthercat()){
      std::cerr << "startEtherCat() failed"<<std::endl;
      return false;
    }

    if (pthread_create(&etherCatThread, nullptr, GVdrive::loopFunc, this) != 0) {
      perror("Failed to create EtherCAT thread");
      return false;
    }
    return true;
  }

  void GVdrive::setMode(UserMode new_mode){
    GVdrive::mode = new_mode;
  }

  void GVdrive::setTargetPosition(const std::string& motor_name, double position)
  {
      std::lock_guard<std::mutex> lock(driveMutex);
      if (motor_map_.count(motor_name)) {
          motors[motor_map_[motor_name]].targetPosition = position;
      } 
      else {
          std::cerr << "Warning: Motor '" << motor_name << "' not found." << std::endl;
      }
  }

  void GVdrive::setTargetTorque(const std::string& motor_name, double torque)
  {
      std::lock_guard<std::mutex> lock(driveMutex);
      if (motor_map_.count(motor_name)) {
          motors[motor_map_[motor_name]].targetTorque = torque;
      } else {
          std::cerr << "Warning: Motor '" << motor_name << "' not found." << std::endl;
      }
  }

  void GVdrive::setTargetVelocity(const std::string& motor_name, double velocity)
  {
      std::lock_guard<std::mutex> lock(driveMutex);
      if (motor_map_.count(motor_name)) {
          motors[motor_map_[motor_name]].targetVelocity = velocity;
      } else {
          std::cerr << "Warning: Motor '" << motor_name << "' not found." << std::endl;
      }
  }

  double GVdrive::getPosition(const std::string& motor_name)
  {
      std::lock_guard<std::mutex> lock(driveMutex);
      if (motor_map_.count(motor_name)) {
          return motors[motor_map_.at(motor_name)].Position;
      }
      std::cerr << "Warning: Motor '" << motor_name << "' not found in getPosition." << std::endl;
      return 0.0;
  }

  double GVdrive::getVelocity(const std::string& motor_name)
  {
      std::lock_guard<std::mutex> lock(driveMutex);
      if (motor_map_.count(motor_name)) {
          return motors[motor_map_.at(motor_name)].Velocity;
      }
      std::cerr << "Warning: Motor '" << motor_name << "' not found in getVelocity." << std::endl;
      return 0.0;
  }

  double GVdrive::getTorque(const std::string& motor_name)
  {
      std::lock_guard<std::mutex> lock(driveMutex);
      if (motor_map_.count(motor_name)) {
          return motors[motor_map_.at(motor_name)].Torque;
      }
      std::cerr << "Warning: Motor '" << motor_name << "' not found in getTorque." << std::endl;
      return 0.0;
  }

  bool GVdrive::startEthercat() {
    ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
    ec_config_overlap_map(&IOmap_);
    ec_configdc();
    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
    
    ec_slave[0].state = EC_STATE_OPERATIONAL;
    ec_send_overlap_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    ec_writestate(0);
    //////////// wait for all slaves to reach OP state ////////////////////////////
    uint8_t count = 5;
    uint16_t timeout = 50000;
    do {
      ec_send_overlap_processdata();
      ec_receive_processdata(EC_TIMEOUTRET);
      ec_statecheck(0, EC_STATE_OPERATIONAL, timeout);
    } while (count-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));
    
    if (ec_slave[0].state == EC_STATE_OPERATIONAL) {
      std::cout<<"all slaves are operational"<<std::endl;
    }
    else{
      ec_readstate();
      std::cerr << "not all slaves reached operational state" << std::endl;
    }
    std::cout << "ec_slavecount : "<<ec_slavecount << std::endl;


    for (uint8_t i = 3; i <= ec_slavecount; i++) {
      auto rx = reinterpret_cast<robot::RxPDO *>(ec_slave[i].outputs);
      rx->TargetPosition = 0;
      rx->TargetVelocity = 0;
      rx->TargetTorque = 0;
      rx->ControlWord = robot::CiA402CONTROLWORD::FAULT_CLEAR;
      rx->ModesofOperation = robot::OPMODE::PWM_DUTY_ZERO;
      rx->ImpedancePGain = 0;
      rx->ImpedanceDGain = 0;
    }
    ready = true;

    std::map<uint8_t, int> initialErrorCodes;

    // Helper function to get error descriptions
    auto getErrorDescription = [](int errorCode) -> std::string {
      std::stringstream ss;
      ss << "Errors: ";
      if (errorCode & (1 << 0)) { ss << "[Over Current] "; }
      if (errorCode & (1 << 1)) { ss << "[Over Temperature] "; }
      if (errorCode & (1 << 2)) { ss << "[No Encoder Offset] "; }
      return ss.str();
    };
    // Helper function to convert to binary string
    auto toBinaryString = [](int value) -> std::string {
      std::string binary;
      for (int i = 3; i >= 0; i--) {  // Changed to 4 bits for integer
        binary += (value & (1 << i)) ? '1' : '0';
        if (i % 4 == 0 && i != 0) {
          binary += ' ';                           // Add space every 4 bits
        }
      }
      return binary;
    };
    
    // First pass: record initial error codes
    for (uint8_t i = 3; i <= ec_slavecount; i++) {
      auto tx = reinterpret_cast<robot::TxPDO *>(ec_slave[i].inputs);
      if (tx->ErrorCode != 0) {
        initialErrorCodes[i] = tx->ErrorCode;
        std::cerr << std::to_string(i) + ": fault code "
                            + std::to_string((int) tx->ErrorCode)
                            + " (binary: " + toBinaryString(tx->ErrorCode) + ") "
                            + getErrorDescription(tx->ErrorCode) <<std::endl;
      }
    }
    
    // Clear faults loop
    bool noErrorForAllSlaves;
    int faultClearCnt = 100;
    do {
      ec_send_overlap_processdata();
      ec_receive_processdata(EC_TIMEOUTRET);
      noErrorForAllSlaves = true;
      for (uint8_t i = 3; i <= ec_slavecount; i++) {
        auto tx = reinterpret_cast<robot::TxPDO *>(ec_slave[i].inputs);
        auto rx = reinterpret_cast<robot::RxPDO *>(ec_slave[i].outputs);
        rx->TargetPosition = 0;
        rx->TargetVelocity = 0;
        rx->TargetTorque = 0;
        rx->ControlWord = robot::CiA402CONTROLWORD::FAULT_CLEAR;
        rx->ModesofOperation = robot::OPMODE::PWM_DUTY_ZERO;
        rx->ImpedancePGain = 0;
        rx->ImpedanceDGain = 0;
        // If fault was present but now cleared
        if (tx->ErrorCode == 0 && initialErrorCodes.count(i) > 0) {
          std::cerr <<  std::to_string(i) + " (was: "
                              + std::to_string(initialErrorCodes[i]) + ", binary: "
                              + toBinaryString(initialErrorCodes[i]) + ")" << std::endl;
          initialErrorCodes.erase(i);
        }
        if (tx->ErrorCode != 0) {
          noErrorForAllSlaves = false;
        }
      }
    } while (faultClearCnt-- && !noErrorForAllSlaves);
  
    // First pass: record initial error codes
    if (faultClearCnt == -1) {
      std::cerr << "not all faults cleared for all slaves." <<std::endl;
  
      for (uint8_t i = 3; i <= ec_slavecount; i++) //here is 0819 test 4->3
      {
        auto tx = reinterpret_cast<robot::TxPDO *>(ec_slave[i].inputs);
        if (tx->ErrorCode != 0) {
          initialErrorCodes[i] = tx->ErrorCode;
          std::cerr << 
                          std::to_string(i) + ": fault code "
                              + std::to_string((int) tx->ErrorCode) + ", (binary: "
                              + toBinaryString(tx->ErrorCode) + ") "
                              + getErrorDescription(tx->ErrorCode) <<std::endl;
        }
      }
    } else {
      std::cout << "all faults cleared for all slaves." <<std::endl;
    }
  
    for (uint8_t i = 3; i <= ec_slavecount; i++) {
      auto rx = reinterpret_cast<robot::RxPDO *>(ec_slave[i].outputs);
      rx->TargetPosition = 0;
      rx->TargetVelocity = 0;
      rx->TargetTorque = 0;
      rx->ControlWord = robot::CiA402CONTROLWORD::DISABLE_VOLTAGE;
      rx->ModesofOperation = robot::OPMODE::PWM_DUTY_ZERO;
      rx->ImpedancePGain = 0;
      rx->ImpedanceDGain = 0;
    }

    if(GVdrive::mode == CONTROL){
      ec_send_overlap_processdata();
      int workCount = ec_receive_processdata(EC_TIMEOUTRET);

      if (workCount == (slaveCount-2) * 3) {
        for(auto &motor : motors){
          auto tx = reinterpret_cast<robot::TxPDO *>(ec_slave[motor.slaveIndex].inputs);
          std::lock_guard guard(driveMutex);
          motor.initialposition = tx->PositionActualValue*Convert::unitToJointPos*180.0/M_PI;
          motor.Position=motor.initialposition;
          motor.targetPosition=motor.initialposition;
          motor.Torque = tx->TorqueActualValue*Convert::unitToJointToq;
          motor.Velocity = tx->VelocityActualSensor*Convert::unitToJointVel;
          motor.status = tx->Statusword;
          motor.temperature = tx->Temperature;
          motor.errorCode = tx->ErrorCode;
        }
      }
    }
    std::cout<< "startEthercat() complete"<<std::endl;

    return true;  
  }
  
  bool GVdrive::shutdown(){
    runEtherCatLoop = false;
    if (etherCatThread){
      pthread_join(etherCatThread, nullptr);
      etherCatThread=0;
    }

    for (uint8_t i = 3; i <= ec_slavecount; ++i) {
        auto rx = reinterpret_cast<robot::RxPDO *>(ec_slave[i].outputs);
        rx->TargetPosition = 0;
        rx->TargetVelocity = 0;
        rx->TargetTorque   = 0;
        rx->ImpedancePGain = 0;
        rx->ImpedanceDGain = 0;
        rx->ModesofOperation = robot::OPMODE::PWM_DUTY_ZERO;
        rx->ControlWord = robot::CiA402CONTROLWORD::DISABLE_VOLTAGE;
      }
    
    bool slave_state = false;
    const int max_stop_iter = 100;
    uint8_t stop_cnt = 0; 
    while ( !slave_state && stop_cnt < max_stop_iter) {
      ec_send_overlap_processdata();
      ec_receive_processdata(EC_TIMEOUTRET);
      slave_state = true;
      for (uint8_t i = 3; i <= ec_slavecount; ++i) {
        auto tx = reinterpret_cast<robot::TxPDO *>(ec_slave[i].inputs);
        if (!tx) continue;
        if ( (tx->Statusword & 0x006F) != 0x0040 ) {
          slave_state = false;
          break;
        }
      }
      stop_cnt++;
    }
    if (!slave_state){
      std::cerr << "Failed to disable all slaves" << std::endl;
    }

    ec_slave[0].state = EC_STATE_SAFE_OP;
    ec_writestate(0);
    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE*4);
    
    ec_slave[0].state = EC_STATE_INIT;
    ec_writestate(0);
    ec_statecheck(0, EC_STATE_INIT, EC_TIMEOUTSTATE*4);

    ec_close();
    return true;
  }

  void* GVdrive::loop(double frequency) {

    // Calculate the period as duration in seconds
    long period_ns=static_cast<long>(1e9/frequency);

    // Determine the next tick time points
    struct timespec next;
    clock_gettime(CLOCK_MONOTONIC, &next);
    
    while (runEtherCatLoop) {
      switch (GVdrive::mode){
        case CALIBRATION:
          GVdrive::calibration();
        break;
        case CONTROL:
          GVdrive::control();
        break;        
      }
    
    
    next.tv_nsec+=period_ns;
    while(next.tv_nsec>=1000000000L){
      next.tv_nsec -=1000000000L;
      next.tv_sec+=1;
    }
    int ret;
    do{ 
      ret=clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
    }while(ret&&errno==EINTR);
  }

  return nullptr;
  }

  void* GVdrive::loopFunc(void* arg){
    GVdrive* self =static_cast<GVdrive*>(arg);

    cpu_set_t cpu_set;
    CPU_ZERO(&cpu_set);
    CPU_SET(CPU_CORE, &cpu_set);

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
      perror("memory lockall failed");
      exit(-1);
    }

    if(pthread_setaffinity_np(pthread_self(), sizeof(cpu_set), &cpu_set) != 0){
      perror("pthread_setaffinity_np failed");
    }

    struct sched_param schedParam;
    schedParam.sched_priority = 99;
    if(pthread_setschedparam(pthread_self(), SCHED_FIFO, &schedParam)!=0){
      perror("pthread_setschedparam failed");
    }

    return self->loop(self->loopFrequency);
  }

  void GVdrive::calibration(){
    for(int i=3;i<=ec_slavecount; i++)
    {
      auto rx = reinterpret_cast<robot::RxPDO *>(ec_slave[i].outputs);
      rx->ModesofOperation = robot::OPMODE::CYCLIC_SYNC_TORQUE;
      rx->TargetPosition = 0.0;
      rx->TargetVelocity = 0.0;
      rx->ControlWord = robot::CiA402CONTROLWORD::ENABLE_OPERATION;

      {
        std::lock_guard guard(driveMutex);
        rx->TargetTorque = 0.0;
      }
      rx->ImpedancePGain = 100;
      rx->ImpedanceDGain = 0.1;
    }

    {
      ec_send_overlap_processdata();
      int workCount = ec_receive_processdata(EC_TIMEOUTRET);
            
      if (workCount == (slaveCount-2) * 3) {
        for(int i=3;i<=slaveCount;i++){
          std::lock_guard guard(driveMutex);
          auto tx = reinterpret_cast<robot::TxPDO *>(ec_slave[i].inputs);
          long double rad = tx->PositionActualValue*Convert::unitToJointPos;
          std::cout<<"slave Id: "<<i<<" degree: "<<rad*180.0/M_PI<<std::endl;
        }
      }
    }
      
    }
    
  void GVdrive::control(){

    for(auto &motor:motors){
      auto rx = reinterpret_cast<robot::RxPDO *>(ec_slave[motor.slaveIndex].outputs);
      switch (motor.control_mode){
        case TORQUE:
          rx->ModesofOperation = robot::OPMODE::CYCLIC_SYNC_TORQUE;
          rx->ControlWord = robot::CiA402CONTROLWORD::ENABLE_OPERATION;
          rx->TargetVelocity = 0.0;
          rx->TargetPosition =0.0;
          {
            std::lock_guard guard(driveMutex);
            rx->TargetTorque = motor.targetTorque*robot::Convert::unitToMotorToq;
          }
          break;
        case IMPEDANCE:
          rx->ModesofOperation = robot::OPMODE::IMPEDANCE_CONTROL;
          rx->ControlWord = robot::CiA402CONTROLWORD::ENABLE_OPERATION;
          rx->TargetVelocity = 0.0;
          rx->TargetTorque = 0.0;
          {
            std::lock_guard guard(driveMutex);
            rx->TargetPosition =motor.targetPosition/180.0*M_PI*robot::Convert::unitToMotorPos;
          }
          break;
        case VELOCITY:
          rx->ModesofOperation = robot::OPMODE::CYCLIC_SYNC_VELOCITY;
          rx->ControlWord = robot::CiA402CONTROLWORD::ENABLE_OPERATION;
          rx->TargetTorque = 0.0;
          rx->TargetPosition =0.0;
          {
            std::lock_guard guard(driveMutex);
            rx->TargetVelocity = motor.targetVelocity*robot::Convert::unitToMotorVel;
          }
          break;
        case POSITION:
          rx->ModesofOperation = robot::OPMODE::CYCLIC_SYNC_POSITION;
          rx->ControlWord = robot::CiA402CONTROLWORD::ENABLE_OPERATION;
          rx->TargetVelocity = 0.0;
          rx->TargetTorque = 0.0;
          {
            std::lock_guard guard(driveMutex);
            rx->TargetPosition =motor.targetPosition/180.0*M_PI*robot::Convert::unitToMotorPos;
          }
          break;

        case TORQUE_M:
          rx->ModesofOperation = robot::OPMODE::CYCLIC_SYNC_TORQUE;
          rx->ControlWord = robot::CiA402CONTROLWORD::ENABLE_OPERATION;
          rx->TargetVelocity = 0.0;
          rx->TargetPosition =0.0;
          {
            std::lock_guard guard(driveMutex);
            rx->TargetTorque = motor.targetTorque*robot::Convert::unitToMotorToq_M;
          }
          break;
        case IMPEDANCE_M:
          rx->ModesofOperation = robot::OPMODE::IMPEDANCE_CONTROL;
          rx->ControlWord = robot::CiA402CONTROLWORD::ENABLE_OPERATION;
          rx->TargetVelocity = 0.0;
          rx->TargetTorque = 0.0;
          {
            std::lock_guard guard(driveMutex);
            rx->TargetPosition =motor.targetPosition/180.0*M_PI*robot::Convert::unitToMotorPos_M;
          }
          break;
        case VELOCITY_M:
          rx->ModesofOperation = robot::OPMODE::CYCLIC_SYNC_VELOCITY;
          rx->ControlWord = robot::CiA402CONTROLWORD::ENABLE_OPERATION;
          rx->TargetTorque = 0.0;
          rx->TargetPosition =0.0;
          {
            std::lock_guard guard(driveMutex);
            rx->TargetVelocity = motor.targetVelocity*robot::Convert::unitToMotorVel_M;
          }
          break;
        case POSITION_M:
          rx->ModesofOperation = robot::OPMODE::CYCLIC_SYNC_POSITION;
          rx->ControlWord = robot::CiA402CONTROLWORD::ENABLE_OPERATION;
          rx->TargetVelocity = 0.0;
          rx->TargetTorque = 0.0;
          {
            std::lock_guard guard(driveMutex);
            rx->TargetPosition =motor.targetPosition/180.0*M_PI*robot::Convert::unitToMotorPos_M;
          }
          break;

        case TORQUE_HM:
          rx->ModesofOperation = robot::OPMODE::CYCLIC_SYNC_TORQUE;
          rx->ControlWord = robot::CiA402CONTROLWORD::ENABLE_OPERATION;
          rx->TargetVelocity = 0.0;
          rx->TargetPosition =0.0;
          {
            std::lock_guard guard(driveMutex);
            rx->TargetTorque = motor.targetTorque*robot::Convert::unitToMotorToq_HM;
          }
          break;
        case IMPEDANCE_HM:
          rx->ModesofOperation = robot::OPMODE::IMPEDANCE_CONTROL;
          rx->ControlWord = robot::CiA402CONTROLWORD::ENABLE_OPERATION;
          rx->TargetVelocity = 0.0;
          rx->TargetTorque = 0.0;
          {
            std::lock_guard guard(driveMutex);
            rx->TargetPosition =motor.targetPosition/180.0*M_PI*robot::Convert::unitToMotorPos_HM;
          }
          break;
        case VELOCITY_HM:
          rx->ModesofOperation = robot::OPMODE::CYCLIC_SYNC_VELOCITY;
          rx->ControlWord = robot::CiA402CONTROLWORD::ENABLE_OPERATION;
          rx->TargetTorque = 0.0;
          rx->TargetPosition =0.0;
          {
            std::lock_guard guard(driveMutex);
            rx->TargetVelocity = motor.targetVelocity*robot::Convert::unitToMotorVel_HM;
          }
          break;
        case POSITION_HM:
          rx->ModesofOperation = robot::OPMODE::CYCLIC_SYNC_POSITION;
          rx->ControlWord = robot::CiA402CONTROLWORD::ENABLE_OPERATION;
          rx->TargetVelocity = 0.0;
          rx->TargetTorque = 0.0;
          {
            std::lock_guard guard(driveMutex);
            rx->TargetPosition =motor.targetPosition/180.0*M_PI*robot::Convert::unitToMotorPos_HM;
          }
          break;

        case TORQUE_S:
          rx->ModesofOperation = robot::OPMODE::CYCLIC_SYNC_TORQUE;
          rx->ControlWord = robot::CiA402CONTROLWORD::ENABLE_OPERATION;
          rx->TargetVelocity = 0.0;
          rx->TargetPosition =0.0;
          {
            std::lock_guard guard(driveMutex);
            rx->TargetTorque = motor.targetTorque*robot::Convert::unitToMotorToq_S;
          }
          break;
        case IMPEDANCE_S:
          rx->ModesofOperation = robot::OPMODE::IMPEDANCE_CONTROL;
          rx->ControlWord = robot::CiA402CONTROLWORD::ENABLE_OPERATION;
          rx->TargetVelocity = 0.0;
          rx->TargetTorque = 0.0;
          {
            std::lock_guard guard(driveMutex);
            rx->TargetPosition =motor.targetPosition/180.0*M_PI*robot::Convert::unitToMotorPos_S;
          }
          break;
        case VELOCITY_S:
          rx->ModesofOperation = robot::OPMODE::CYCLIC_SYNC_VELOCITY;
          rx->ControlWord = robot::CiA402CONTROLWORD::ENABLE_OPERATION;
          rx->TargetTorque = 0.0;
          rx->TargetPosition =0.0;
          {
            std::lock_guard guard(driveMutex);
            rx->TargetVelocity = motor.targetVelocity*robot::Convert::unitToMotorVel_S;
          }
          break;
        case POSITION_S:
          rx->ModesofOperation = robot::OPMODE::CYCLIC_SYNC_POSITION;
          rx->ControlWord = robot::CiA402CONTROLWORD::ENABLE_OPERATION;
          rx->TargetVelocity = 0.0;
          rx->TargetTorque = 0.0;
          {
            std::lock_guard guard(driveMutex);
            rx->TargetPosition =motor.targetPosition/180.0*M_PI*robot::Convert::unitToMotorPos_S;
          }
          break;

        default:
          rx->ModesofOperation = robot::OPMODE::CYCLIC_SYNC_POSITION;
          rx->ControlWord = robot::CiA402CONTROLWORD::DISABLE_VOLTAGE;
          break;
      }
      
      rx->ImpedancePGain = motor.ImpedancePGain;
      rx->ImpedanceDGain = motor.ImpedanceDGain;
      }

    
      ec_send_overlap_processdata();
      int workCount = ec_receive_processdata(EC_TIMEOUTRET);

    if (workCount == (slaveCount-2) * 3) {
      for(auto &motor : motors){
        auto tx = reinterpret_cast<robot::TxPDO *>(ec_slave[motor.slaveIndex].inputs);
        std::lock_guard guard(driveMutex);
        motor.Position = tx->PositionActualValue*Convert::unitToJointPos*180.0/M_PI;
        motor.Torque = tx->TorqueActualValue*Convert::unitToJointToq;
        motor.Velocity = tx->VelocityActualSensor*Convert::unitToJointVel;
        motor.status = tx->Statusword;
        motor.temperature = tx->Temperature;
        motor.errorCode = tx->ErrorCode;
      }
    }
  }
}